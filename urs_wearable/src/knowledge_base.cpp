#include <chrono>
#include <set>
#include <stdexcept>
#include <thread>

#include <boost/tokenizer.hpp>

#include "urs_wearable/knowledge_base.h"
#include "urs_wearable/GetPlan.h"

void KnowledgeBase::publish()
{
  std::lock_guard<std::mutex> lock(state_pub_mutex_);
  if (state_pub_)
  {
    urs_wearable::State state;

    auto predicate_map_lt = predicate_map_.lock_table();

    // If there are updating instances running,
    // we exit the function because it will be called by some updating instant anyway.
    if (updating_instances_)
    {
      predicate_map_lt.unlock();
      return;
    }

    // Get a vector of predicates from predicate_map_
    state.predicates.reserve(predicate_map_lt.size());
    for (const auto& m : predicate_map_lt)
    {
      state.predicates.insert(state.predicates.end(), m.second.begin(), m.second.end());
    }
    predicate_map_lt.unlock();

    // Get a vector of locations from location_map_
    auto location_table_lt = location_table_.map_.lock_table();
    for (const auto& m : location_table_lt)
    {
      urs_wearable::Location l;
      l.location_id = m.first;
      l.pose = m.second;
      state.locations.push_back(l);
    }
    location_table_lt.unlock();

    // Publish
    state_pub_->publish(state);
    ros::spinOnce();
  }
}

void KnowledgeBase::replan()
{
  // If there are updating instances running,
  // we exit the function because it will be called by some updating instant anyway.
  if (updating_instances_)
  {
    return;
  }

  // Gather the executors we are going to re-plan for
  std::vector<executor_id_type> executor_id_list;
  auto executor_map_lt = executor_map_.lock_table();
  for (const auto& m : executor_map_lt)
  {
    executor_id_list.push_back(m.first);
  }
  executor_map_lt.unlock();

  // Do the re-planning for each executor
  for (const auto id : executor_id_list)
  {
    executor_map_.update_fn(id, [this](struct Executor& executor)
    {
      urs_wearable::GetPlan get_plan_srv;
      get_plan_srv.request.problem_def = getProblemDef(executor.goal);

      if (ros::service::call(PLANNER_SERVICE_NAME, get_plan_srv))
      {
        if (executor.plan != get_plan_srv.response.plan)
        {
          executor.plan = get_plan_srv.response.plan;
          executor.plan_has_changed = true;
        }
      }
      else
      {
        ROS_ERROR("Re-planning: Call %s failed", PLANNER_SERVICE_NAME.c_str());
      }
    });
  }
}

void KnowledgeBase::getPlan(executor_id_type executor_id, const std::vector<urs_wearable::Predicate>& goal, std::vector<std::string>& plan)
{
  urs_wearable::GetPlan get_plan_srv;
  get_plan_srv.request.problem_def = getProblemDef(goal);

  if (ros::service::call(PLANNER_SERVICE_NAME, get_plan_srv))
  {
    struct Executor executor = {goal, get_plan_srv.response.plan, false};
    if (executor_map_.insert(executor_id, executor))
    {
      plan = executor.plan;
    }
  }
  else
  {
    ROS_ERROR("Executor %u: Call %s failed", executor_id, PLANNER_SERVICE_NAME.c_str());
  }
}

bool KnowledgeBase::getPlanIfPlanHasChanged(executor_id_type executor_id, std::vector<std::string>& plan)
{
  bool ret(false);

  executor_map_.update_fn(executor_id, [&plan, &ret](struct Executor& executor)
  {
    ret = executor.plan_has_changed;
    if (executor.plan_has_changed)
    {
      plan = executor.plan;
      executor.plan_has_changed = false;
    }
  });

  return ret;
}

/***************************************************************************************************/
/**** The methods below need modifications if predicates, objects, or actions have been changed ****/
/***************************************************************************************************/

// This method needs to be modified if there is a change in
// - predicate
void KnowledgeBase::upsertPredicates(const std::vector<urs_wearable::Predicate>& new_preds)
{
  updating_instances_++;

  // Verdict: if + update_fn is faster than upsert especially when there are a large number of objects
  // Tested with inserting 100 predicates to the same key in cuckoohash_map<int, std::vector<urs_wearable::Predicate>>
  // 6710 | 6507 | 6899 us  - using upsert
  // 6519 | 6203 | 6497 us  - using if + update_fn
  // Tested with inserting 100,000 predicates to the same key in cuckoohash_map<int, std::vector<urs_wearable::Predicate>>
  // 63558 us - using upsert
  // 48123 us - using if + update_fn
  for (const auto& new_pred : new_preds)
  {
    switch (new_pred.type)
    {
      case urs_wearable::Predicate::TYPE_ACTIVE_REGION:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_ACTIVE_REGION, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.predicate_active_region.location_id_sw.value == cur_preds_it->predicate_active_region.location_id_sw.value
                && new_pred.predicate_active_region.location_id_ne.value == cur_preds_it->predicate_active_region.location_id_ne.value)
            {
              // If ERASE_WHEN_FALSE is set and the truth value is false, then erase it
              if (urs_wearable::PredicateActiveRegion::ERASE_WHEN_FALSE && !new_pred.predicate_active_region.truth_value)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->predicate_active_region.truth_value = new_pred.predicate_active_region.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base unless its truth value is false and ERASE_WHEN_FALSE is set
          if (!existed && (!urs_wearable::PredicateActiveRegion::ERASE_WHEN_FALSE || new_pred.predicate_active_region.truth_value))
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (!urs_wearable::PredicateActiveRegion::ERASE_WHEN_FALSE || new_pred.predicate_active_region.truth_value)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_ACTIVE_REGION, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_DRONE_ABOVE:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_DRONE_ABOVE, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.predicate_drone_above.drone_id.value == cur_preds_it->predicate_drone_above.drone_id.value
                && new_pred.predicate_drone_above.location_id.value == cur_preds_it->predicate_drone_above.location_id.value)
            {
              // If ERASE_WHEN_FALSE is set and the truth value is false, then erase it
              if (urs_wearable::PredicateDroneAbove::ERASE_WHEN_FALSE && !new_pred.predicate_drone_above.truth_value)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->predicate_drone_above.truth_value = new_pred.predicate_drone_above.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base unless its truth value is false and ERASE_WHEN_FALSE is set
          if (!existed && (!urs_wearable::PredicateDroneAbove::ERASE_WHEN_FALSE || new_pred.predicate_drone_above.truth_value))
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (!urs_wearable::PredicateDroneAbove::ERASE_WHEN_FALSE || new_pred.predicate_drone_above.truth_value)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_DRONE_ABOVE, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_DRONE_AT:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_DRONE_AT, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.predicate_drone_at.drone_id.value == cur_preds_it->predicate_drone_at.drone_id.value
                && new_pred.predicate_drone_at.location_id.value == cur_preds_it->predicate_drone_at.location_id.value)
            {
              // If ERASE_WHEN_FALSE is set and the truth value is false, then erase it
              if (urs_wearable::PredicateDroneAt::ERASE_WHEN_FALSE && !new_pred.predicate_drone_at.truth_value)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->predicate_drone_at.truth_value = new_pred.predicate_drone_at.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base unless its truth value is false and ERASE_WHEN_FALSE is set
          if (!existed && (!urs_wearable::PredicateDroneAt::ERASE_WHEN_FALSE || new_pred.predicate_drone_at.truth_value))
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (!urs_wearable::PredicateDroneAt::ERASE_WHEN_FALSE || new_pred.predicate_drone_at.truth_value)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_DRONE_AT, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_KEY_AT:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_KEY_AT, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.predicate_key_at.key_id.value == cur_preds_it->predicate_key_at.key_id.value
                && new_pred.predicate_key_at.location_id.value == cur_preds_it->predicate_key_at.location_id.value)
            {
              // If ERASE_WHEN_FALSE is set and the truth value is false, then erase it
              if (urs_wearable::PredicateKeyAt::ERASE_WHEN_FALSE && !new_pred.predicate_key_at.truth_value)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->predicate_key_at.truth_value = new_pred.predicate_key_at.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base unless its truth value is false and ERASE_WHEN_FALSE is set
          if (!existed && (!urs_wearable::PredicateKeyAt::ERASE_WHEN_FALSE || new_pred.predicate_key_at.truth_value))
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (!urs_wearable::PredicateKeyAt::ERASE_WHEN_FALSE || new_pred.predicate_key_at.truth_value)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_KEY_AT, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_KEY_PICKED:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_KEY_PICKED, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.predicate_key_picked.key_id.value == cur_preds_it->predicate_key_picked.key_id.value)
            {
              // If ERASE_WHEN_FALSE is set and the truth value is false, then erase it
              if (urs_wearable::PredicateKeyPicked::ERASE_WHEN_FALSE && !new_pred.predicate_key_picked.truth_value)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->predicate_key_picked.truth_value = new_pred.predicate_key_picked.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base unless its truth value is false and ERASE_WHEN_FALSE is set
          if (!existed && (!urs_wearable::PredicateKeyPicked::ERASE_WHEN_FALSE || new_pred.predicate_key_picked.truth_value))
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (!urs_wearable::PredicateKeyPicked::ERASE_WHEN_FALSE || new_pred.predicate_key_picked.truth_value)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_KEY_PICKED, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_KEY_WITH:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_KEY_WITH, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.predicate_key_with.key_id.value == cur_preds_it->predicate_key_with.key_id.value
                && new_pred.predicate_key_with.drone_id.value == cur_preds_it->predicate_key_with.drone_id.value)
            {
              // If ERASE_WHEN_FALSE is set and the truth value is false, then erase it
              if (urs_wearable::PredicateKeyWith::ERASE_WHEN_FALSE && !new_pred.predicate_key_with.truth_value)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->predicate_key_with.truth_value = new_pred.predicate_key_with.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base unless its truth value is false and ERASE_WHEN_FALSE is set
          if (!existed && (!urs_wearable::PredicateKeyWith::ERASE_WHEN_FALSE || new_pred.predicate_key_with.truth_value))
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (!urs_wearable::PredicateKeyWith::ERASE_WHEN_FALSE || new_pred.predicate_key_with.truth_value)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_KEY_WITH, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_TOOK_OFF:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_TOOK_OFF, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.predicate_took_off.drone_id.value == cur_preds_it->predicate_took_off.drone_id.value)
            {
              // If ERASE_WHEN_FALSE is set and the truth value is false, then erase it
              if (urs_wearable::PredicateTookOff::ERASE_WHEN_FALSE && !new_pred.predicate_took_off.truth_value)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->predicate_took_off.truth_value = new_pred.predicate_took_off.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base unless its truth value is false and ERASE_WHEN_FALSE is set
          if (!existed && (!urs_wearable::PredicateTookOff::ERASE_WHEN_FALSE || new_pred.predicate_took_off.truth_value))
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (!urs_wearable::PredicateTookOff::ERASE_WHEN_FALSE || new_pred.predicate_took_off.truth_value)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_TOOK_OFF, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;
    }
  }

  // If there are no other instances updating the table, we consider this current state final
  // and proceed to clean up invalid location references, publish the consistent KB, and re-plan.
  if (--updating_instances_ == 0)
  {
    /**** Clean up invalid location references (idea) ****/
    // 1. Check for other references to the removed location.
    // Remove the location id from removed_location_id_set if it is still referred.

    // 2. Use the remaining id in removed_location_id_set to remove the locations in location_table_

    // 3. Clean up auto-generated predicates that might still use the removed location id(s)
    // There are no such predicates yet.
    /*****************************************************/

    std::thread thread_publish(&KnowledgeBase::publish, this);
    std::thread thread_replan(&KnowledgeBase::replan, this);

    thread_publish.detach();
    thread_replan.detach();
  }
}

// This method needs to be modified if there is a change in
// - predicate
// - object
std::string KnowledgeBase::getProblemDef(const std::vector<urs_wearable::Predicate>& goal)
{
  std::vector<urs_wearable::Predicate> current_predicates;

  // Get current predicates
  while (true)
  {
    auto predicate_map_lt = predicate_map_.lock_table();

    if (updating_instances_ == 0)
    {
      // Verdict: Copying a map is slower than converting it to a vector
      // Tested with a cuckoohash_map with 1,000,000 integers
      // 267179 us  - using vector with reserve
      // 272954 us  - using vector
      // 903037 us  - using cuckoohash_map copy constructor
      // 921961 us  - using cuckoohash_map assignment operator
      // 1566583 us - copying to std::unordered_map element-by-element
      // 2345838 us - copying to cuckoohash_map element-by-element
      current_predicates.reserve(predicate_map_lt.size());
      for (const auto& m : predicate_map_lt)
      {
        current_predicates.insert(current_predicates.end(), m.second.begin(), m.second.end());
      }

      // Add unknown goal predicates as false in :init, otherwise some plans can't be made
      for (const auto& goal_pred : goal)
      {
        bool found = false;

        // Verdict: It is faster when no exception is thrown
        // Tested with 10,000 iterations
        // 19 us    - No exception thrown, just using if statement
        // 17342 us - Throw std::exception and catch by std::exception
        // 18828 us - Throw std::out_of_range and catch by std::exception
        // 18904 us - Throw std::out_of_range and catch by std::out_of_range
        try
        {
          // This could throw std::out_of_range if key goal_pred.type is not found in the map
          const std::vector<urs_wearable::Predicate>& preds = predicate_map_lt.at(goal_pred.type);
          switch (goal_pred.type)
          {
            case urs_wearable::Predicate::TYPE_ACTIVE_REGION:
              for (const auto& pred : preds)
              {
                if (goal_pred.predicate_active_region.location_id_ne.value == pred.predicate_active_region.location_id_ne.value
                    && goal_pred.predicate_active_region.location_id_sw.value == pred.predicate_active_region.location_id_sw.value)
                {
                  found = true;
                  break;
                }
              }
              break;

            case urs_wearable::Predicate::TYPE_DRONE_ABOVE:
              for (const auto& pred : preds)
              {
                if (goal_pred.predicate_drone_above.drone_id.value == pred.predicate_drone_above.drone_id.value
                    && goal_pred.predicate_drone_above.location_id.value == pred.predicate_drone_above.location_id.value)
                {
                  found = true;
                  break;
                }
              }
              break;

            case urs_wearable::Predicate::TYPE_DRONE_AT:
              for (const auto& pred : preds)
              {
                if (goal_pred.predicate_drone_at.drone_id.value == pred.predicate_drone_at.drone_id.value
                    && goal_pred.predicate_drone_at.location_id.value == pred.predicate_drone_at.location_id.value)
                {
                  found = true;
                  break;
                }
              }
              break;

            case urs_wearable::Predicate::TYPE_KEY_AT:
              for (const auto& pred : preds)
              {
                if (goal_pred.predicate_key_at.key_id.value == pred.predicate_key_at.key_id.value
                    && goal_pred.predicate_key_at.location_id.value == pred.predicate_key_at.location_id.value)
                {
                  found = true;
                  break;
                }
              }
              break;

            case urs_wearable::Predicate::TYPE_KEY_PICKED:
              for (const auto& pred : preds)
              {
                if (goal_pred.predicate_key_picked.key_id.value == pred.predicate_key_picked.key_id.value)
                {
                  found = true;
                  break;
                }
              }
              break;

            case urs_wearable::Predicate::TYPE_KEY_WITH:
              for (const auto& pred : preds)
              {
                if (goal_pred.predicate_key_with.key_id.value == pred.predicate_key_with.key_id.value
                    && goal_pred.predicate_key_with.drone_id.value == pred.predicate_key_with.drone_id.value)
                {
                  found = true;
                  break;
                }
              }
              break;

            case urs_wearable::Predicate::TYPE_TOOK_OFF:
              for (const auto& pred : preds)
              {
                if (goal_pred.predicate_took_off.drone_id.value == pred.predicate_took_off.drone_id.value)
                {
                  found = true;
                  break;
                }
              }
              break;
          }
        }
        catch (const std::exception& e) {}

        // If the goal is not found in predicate_map_, add the unknown goal to current_predicates as false
        if (!found)
        {
          current_predicates.push_back(goal_pred);
          urs_wearable::Predicate& p = current_predicates.back();
          p.predicate_active_region.truth_value
          = p.predicate_drone_above.truth_value
          = p.predicate_drone_at.truth_value
          = p.predicate_key_at.truth_value
          = p.predicate_key_picked.truth_value
          = p.predicate_key_with.truth_value
          = p.predicate_took_off.truth_value
          = false;
        }
      }

      predicate_map_lt.unlock();
      break;
    }

    // Wait until updating_instances_ == 0
    predicate_map_lt.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  /**** Create PDDL problem definition ****/
  // Create an object set for every object type we have
  std::set<std::uint8_t> object_drone_id_set;
  std::set<std::uint8_t> object_key_id_set;
  std::set<std::uint8_t> object_location_id_set;

  // The following loop takes care of the following things
  // 1. Get :init string and add objects used to their respective object set (when i == 0)
  // 2. Get :goal string and add objects used to their respective object set (when i == 1)
  std::string init_str;
  std::string goal_str;
  for (int i : {0, 1})
  {
    const std::vector<urs_wearable::Predicate>* preds;
    std::string* preds_string;

    if (i == 0)
    {
      preds = &current_predicates;
      preds_string = &init_str;
    }
    else
    {
      preds = &goal;
      preds_string = &goal_str;
    }

    for (const auto& pred : *preds)
    {
      std::string s;
      switch (pred.type)
      {
        case urs_wearable::Predicate::TYPE_ACTIVE_REGION:
          s = " (" + urs_wearable::PredicateActiveRegion::NAME
            + " " + urs_wearable::ObjectLocationID::TYPE + std::to_string(pred.predicate_active_region.location_id_sw.value)
            + " " + urs_wearable::ObjectLocationID::TYPE + std::to_string(pred.predicate_active_region.location_id_ne.value)
            + ")";
          *preds_string += (pred.predicate_active_region.truth_value) ? s : " (not" + s + ")";

          object_location_id_set.insert(pred.predicate_active_region.location_id_sw.value);
          object_location_id_set.insert(pred.predicate_active_region.location_id_ne.value);
          break;

        case urs_wearable::Predicate::TYPE_DRONE_ABOVE:
          s = " (" + urs_wearable::PredicateDroneAbove::NAME
            + " " + urs_wearable::ObjectDroneID::TYPE + std::to_string(pred.predicate_drone_above.drone_id.value)
            + " " + urs_wearable::ObjectLocationID::TYPE + std::to_string(pred.predicate_drone_above.location_id.value)
            + ")";
          *preds_string += (pred.predicate_drone_above.truth_value) ? s : " (not" + s + ")";

          object_drone_id_set.insert(pred.predicate_drone_above.drone_id.value);
          object_location_id_set.insert(pred.predicate_drone_above.location_id.value);
          break;

        case urs_wearable::Predicate::TYPE_DRONE_AT:
          s = " (" + urs_wearable::PredicateDroneAt::NAME
            + " " + urs_wearable::ObjectDroneID::TYPE + std::to_string(pred.predicate_drone_at.drone_id.value)
            + " " + urs_wearable::ObjectLocationID::TYPE + std::to_string(pred.predicate_drone_at.location_id.value)
            + ")";
          *preds_string += (pred.predicate_drone_at.truth_value) ? s : " (not" + s + ")";

          object_drone_id_set.insert(pred.predicate_drone_at.drone_id.value);
          object_location_id_set.insert(pred.predicate_drone_at.location_id.value);
          break;

        case urs_wearable::Predicate::TYPE_KEY_AT:
          s = " (" + urs_wearable::PredicateKeyAt::NAME
            + " " + urs_wearable::ObjectKeyID::TYPE + std::to_string(pred.predicate_key_at.key_id.value)
            + " " + urs_wearable::ObjectLocationID::TYPE + std::to_string(pred.predicate_key_at.location_id.value)
            + ")";
          *preds_string += (pred.predicate_key_at.truth_value) ? s : " (not" + s + ")";

          object_key_id_set.insert(pred.predicate_key_at.key_id.value);
          object_location_id_set.insert(pred.predicate_key_at.location_id.value);
          break;

        case urs_wearable::Predicate::TYPE_KEY_PICKED:
          s = " (" + urs_wearable::PredicateKeyPicked::NAME
            + " " + urs_wearable::ObjectKeyID::TYPE + std::to_string(pred.predicate_key_picked.key_id.value)
            + ")";
          *preds_string += (pred.predicate_key_picked.truth_value) ? s : " (not" + s + ")";

          object_key_id_set.insert(pred.predicate_key_picked.key_id.value);
          break;

        case urs_wearable::Predicate::TYPE_KEY_WITH:
          s = " (" + urs_wearable::PredicateKeyWith::NAME
            + " " + urs_wearable::ObjectKeyID::TYPE + std::to_string(pred.predicate_key_with.key_id.value)
            + " " + urs_wearable::ObjectDroneID::TYPE + std::to_string(pred.predicate_key_with.drone_id.value)
            + ")";
          *preds_string += (pred.predicate_key_with.truth_value) ? s : " (not" + s + ")";

          object_key_id_set.insert(pred.predicate_key_with.key_id.value);
          object_drone_id_set.insert(pred.predicate_key_with.drone_id.value);
          break;

        case urs_wearable::Predicate::TYPE_TOOK_OFF:
          s = " (" + urs_wearable::PredicateTookOff::NAME
            + " " + urs_wearable::ObjectDroneID::TYPE + std::to_string(pred.predicate_took_off.drone_id.value)
            + ")";
          *preds_string += (pred.predicate_took_off.truth_value) ? s : " (not" + s + ")";

          object_drone_id_set.insert(pred.predicate_took_off.drone_id.value);
          break;
      }
    }
  }

  // Create :objects string from all object sets we have
  std::string objects_str;
  if (object_drone_id_set.size())
  {
    for (const auto i : object_drone_id_set)
    {
      objects_str += " " + urs_wearable::ObjectDroneID::TYPE + std::to_string(i);
    }
    objects_str += " - " + urs_wearable::ObjectDroneID::TYPE;
  }
  else  // Add a dummy object, otherwise CPA might have problem parsing the problem definition
  {
    objects_str += " " + urs_wearable::ObjectDroneID::TYPE + "_dummy - " + urs_wearable::ObjectDroneID::TYPE;
  }

  if (object_key_id_set.size())
  {
    for (const auto i : object_key_id_set)
    {
      objects_str += " " + urs_wearable::ObjectKeyID::TYPE + std::to_string(i);
    }
    objects_str += " - " + urs_wearable::ObjectKeyID::TYPE;
  }
  else  // Add a dummy object, otherwise CPA might have problem parsing the problem definition
  {
    objects_str += " " + urs_wearable::ObjectKeyID::TYPE + "_dummy - " + urs_wearable::ObjectKeyID::TYPE;
  }

  if (object_location_id_set.size())
  {
    for (const auto i : object_location_id_set)
    {
      objects_str += " " + urs_wearable::ObjectLocationID::TYPE + std::to_string(i);
    }
    objects_str += " - " + urs_wearable::ObjectLocationID::TYPE;
  }
  else  // Add a dummy object, otherwise CPA might have problem parsing the problem definition
  {
    objects_str += " " + urs_wearable::ObjectLocationID::TYPE + "_dummy - " + urs_wearable::ObjectLocationID::TYPE;
  }

  return
    "(define (problem " + PROBLEM_NAME + ")\n"
    "  (:domain " + DOMAIN_NAME + ")\n"
    "  (:objects" + objects_str + ")\n"
    "  (:init (and" + init_str + "))\n"
    "  (:goal (and" + goal_str + "))\n"
    ")";
}

// This method needs to be modified if there is a change in
// - action
std::vector<urs_wearable::Action>KnowledgeBase::parsePlan(const std::vector<std::string>& plan)
{
  std::vector<urs_wearable::Action> actions;

  for (const auto& plan_step : plan)
  {
    std::vector<std::string> tokens;
    boost::char_separator<char> sep {",()"};
    boost::tokenizer<boost::char_separator<char>> tok{plan_step, sep};
    for (const auto& t : tok)
    {
      tokens.push_back((std::string)t);
    }

    urs_wearable::Action action;
    if (tokens[0] == urs_wearable::ActionActiveRegionUpdate::NAME)
    {
      action.type = urs_wearable::Action::TYPE_ACTIVE_REGION_UPDATE;
      action.action_active_region_update.location_id_sw_old.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
      action.action_active_region_update.location_id_ne_old.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
      action.action_active_region_update.location_id_sw_new.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
      action.action_active_region_update.location_id_ne_new.value = std::stoi(tokens[4].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionFlyAbove::NAME)
    {
      action.type = urs_wearable::Action::TYPE_FLY_ABOVE;
      action.action_fly_above.drone_id.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDroneID::TYPE.size()));
      action.action_fly_above.location_id_from.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
      action.action_fly_above.location_id_to.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionFlyTo::NAME)
    {
      action.type = urs_wearable::Action::TYPE_FLY_TO;
      action.action_fly_to.drone_id.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDroneID::TYPE.size()));
      action.action_fly_to.location_id_from.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
      action.action_fly_to.location_id_to.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionKeyAdd::NAME)
    {
      action.type = urs_wearable::Action::TYPE_KEY_ADD;
      action.action_key_add.key_id.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectKeyID::TYPE.size()));
      action.action_key_add.location_id.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionKeyPick::NAME)
    {
      action.type = urs_wearable::Action::TYPE_KEY_PICK;
      action.action_key_pick.drone_id.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDroneID::TYPE.size()));
      action.action_key_pick.key_id.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectKeyID::TYPE.size()));
      action.action_key_pick.drone_location_id.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
      action.action_key_pick.key_location_id.value = std::stoi(tokens[4].erase(0, urs_wearable::ObjectLocationID::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionLand::NAME)
    {
      action.type = urs_wearable::Action::TYPE_LAND;
      action.action_land.drone_id.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDroneID::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionTakeOff::NAME)
    {
      action.type = urs_wearable::Action::TYPE_TAKE_OFF;
      action.action_take_off.drone_id.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDroneID::TYPE.size()));
    }
    else
    {
      throw std::invalid_argument("Unrecognized action: " + tokens[0]);
    }
    actions.push_back(action);
  }

  return actions;
}

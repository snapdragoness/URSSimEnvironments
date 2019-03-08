#include <chrono>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include "urs_wearable/common.h"
#include "urs_wearable/knowledge_base.h"

void KnowledgeBase::publish()
{
  std::string pred_string;

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
    for (const auto& pred : predicate_map_lt)
    {
      state.predicates.insert(state.predicates.end(), pred.second.begin(), pred.second.end());

      if (pred_string.empty())
      {
        pred_string += KnowledgeBase::getPredicateString(pred.second);
      }
      else
      {
        pred_string += ", " + KnowledgeBase::getPredicateString(pred.second);
      }
    }
    predicate_map_lt.unlock();

    // Get a vector of locations from loc_map_
    auto loc_map_lt = loc_table_.loc_map_.lock_table();
    for (const auto& loc : loc_map_lt)
    {
      urs_wearable::Location l;
      l.loc_id = loc.first;
      l.pose = loc.second;
      state.locations.push_back(l);
    }
    loc_map_lt.unlock();

    // Get a vector of areas from area_map_
    auto area_map_lt = loc_table_.area_map_.lock_table();
    for (const auto& area : area_map_lt)
    {
      urs_wearable::Area a;
      a.area_id = area.first;
      a.loc_id_left = area.second.loc_id_left;
      a.loc_id_right = area.second.loc_id_right;
      state.areas.push_back(a);
    }
    area_map_lt.unlock();

    // Publish
    state_pub_->publish(state);
  }

  // Log the current state
  ROS_INFO_STREAM("Current state: " << pred_string);
}

std::atomic<unsigned int> KnowledgeBase::executorReplanID {0};

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

  // TODO: Compare only part of the plan of the executor that update the predicates
  // Do the re-planning for each executor
  for (const auto id : executor_id_list)
  {
    executor_map_.update_fn(id, [this](struct Executor& executor)
    {
      std::vector<urs_wearable::Predicate> unsatisfied_goals = getUnsatisfiedGoals(executor.goal);

      if (unsatisfied_goals.size() > 0)
      {
        std::ofstream ofs;
        std::string problem_file = tmp_path + "q" + std::to_string(executorReplanID++) + ".pddl";
        ofs.open(problem_file.c_str());
        ofs << getProblemDef(unsatisfied_goals);
        ofs.close();

        std::string command = planner_command + " -o " + domain_file + " -f " + problem_file;
        std::vector<std::string> plan = parseFF(exec(command.c_str()));

        if (!plan.empty())
        {
          if (executor.plan != plan)
          {
            executor.plan = plan;
            executor.plan_has_changed = true;
          }
        }
        else
        {
          ros_error("Failed in calling " + command);
        }
      }
    });
  }
}

void KnowledgeBase::getPlan(executor_id_type executor_id, const std::vector<urs_wearable::Predicate>& goal, std::vector<std::string>& plan)
{
  std::vector<urs_wearable::Predicate> unsatisfied_goals = getUnsatisfiedGoals(goal);

  if (unsatisfied_goals.size() > 0)
  {
    std::ofstream ofs;
    std::string problem_file = tmp_path + "p" + std::to_string(executor_id) + ".pddl";
    ofs.open(problem_file.c_str());
    ofs << getProblemDef(unsatisfied_goals);
    ofs.close();

    std::string command = planner_command + " -o " + domain_file + " -f " + problem_file;
    plan = parseFF(exec(command.c_str()));

    struct Executor executor = {goal, plan, false};
    if (executor_map_.insert(executor_id, executor))
    {
      plan = executor.plan;
    }
    else
    {
      ros_error("Failed in executor_map_.insert");
    }
  }
}

std::string KnowledgeBase::exec(const char* cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
  {
    throw std::runtime_error("popen() failed!");
  }
  while (!feof(pipe.get()))
  {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
    {
      result += buffer.data();
    }
  }
  return result;
}

std::vector<std::string> KnowledgeBase::parseFF(std::string s)
{
  std::vector<std::string> plan;

  std::istringstream f(s);
  std::string line;
  while (std::getline(f, line))
  {
    std::vector<std::string> tokens;
    boost::char_separator<char> sep {" "};
    boost::tokenizer<boost::char_separator<char>> tok{line, sep};
    for (const auto& t : tok)
    {
      tokens.push_back((std::string)t);
    }

    if (tokens.size() > 0 && tokens[0] == "step")
    {
      bool first_step = true;
      do
      {
        boost::algorithm::to_lower(line); // modifies string in-place

        std::vector<std::string> tokens;
        boost::tokenizer<boost::char_separator<char>> tok{line, sep};
        for (const auto& t : tok)
        {
          tokens.push_back((std::string)t);
        }

        if (tokens.empty())
        {
          break;
        }

        int start_token_index = (first_step)? 2 : 1;
        std::string plan_step(tokens[start_token_index++]);
        while (start_token_index < tokens.size())
        {
          plan_step += " " + tokens[start_token_index++];
        }

        plan.push_back(plan_step);
        first_step = false;
      } while (std::getline(f, line));

      break;
    }
  }

  return plan;
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
std::vector<urs_wearable::Predicate> KnowledgeBase::getUnsatisfiedGoals(const std::vector<urs_wearable::Predicate>& goals)
{
  std::vector<urs_wearable::Predicate> unsatisfied_goals;
  for (const urs_wearable::Predicate& goal : goals)
  {
    bool matched = false;
    switch (goal.type)
    {
      case urs_wearable::Predicate::TYPE_ABOVE:
      {
        predicate_map_.find_fn(urs_wearable::Predicate::TYPE_ABOVE, [&goal, &matched](const std::vector<urs_wearable::Predicate>& cur_preds)
        {
          for (const auto& cur_pred : cur_preds)
          {
            if (cur_pred.above.l0.value == goal.above.l0.value
                && cur_pred.above.l1.value == goal.above.l1.value
                && cur_pred.truth_value == goal.truth_value)
            {
              matched = true;
              break;
            }
          }
        });
      }
      break;

      case urs_wearable::Predicate::TYPE_ALIGNED:
      {
        predicate_map_.find_fn(urs_wearable::Predicate::TYPE_ALIGNED, [&goal, &matched](const std::vector<urs_wearable::Predicate>& cur_preds)
        {
          for (const auto& cur_pred : cur_preds)
          {
            if (cur_pred.aligned.l0.value == goal.aligned.l0.value
                && cur_pred.aligned.l1.value == goal.aligned.l1.value
                && cur_pred.truth_value == goal.truth_value)
            {
              matched = true;
              break;
            }
          }
        });
      }
      break;

      case urs_wearable::Predicate::TYPE_AT:
      {
        predicate_map_.find_fn(urs_wearable::Predicate::TYPE_AT, [&goal, &matched](const std::vector<urs_wearable::Predicate>& cur_preds)
        {
          for (const auto& cur_pred : cur_preds)
          {
            if (cur_pred.at.d.value == goal.at.d.value
                && cur_pred.at.l.value == goal.at.l.value
                && cur_pred.truth_value == goal.truth_value)
            {
              matched = true;
              break;
            }
          }
        });
      }
      break;

      case urs_wearable::Predicate::TYPE_COLLIDED:
      {
        predicate_map_.find_fn(urs_wearable::Predicate::TYPE_COLLIDED, [&goal, &matched](const std::vector<urs_wearable::Predicate>& cur_preds)
        {
          for (const auto& cur_pred : cur_preds)
          {
            if (cur_pred.collided.l0.value == goal.collided.l0.value
                && cur_pred.collided.l1.value == goal.collided.l1.value
                && cur_pred.truth_value == goal.truth_value)
            {
              matched = true;
              break;
            }
          }
        });
      }
      break;

      case urs_wearable::Predicate::TYPE_HOVERED:
      {
        predicate_map_.find_fn(urs_wearable::Predicate::TYPE_HOVERED, [&goal, &matched](const std::vector<urs_wearable::Predicate>& cur_preds)
        {
          for (const auto& cur_pred : cur_preds)
          {
            if (cur_pred.hovered.d.value == goal.hovered.d.value
                && cur_pred.truth_value == goal.truth_value)
            {
              matched = true;
              break;
            }
          }
        });
      }
      break;

      case urs_wearable::Predicate::TYPE_IN:
      {
        predicate_map_.find_fn(urs_wearable::Predicate::TYPE_IN, [&goal, &matched](const std::vector<urs_wearable::Predicate>& cur_preds)
        {
          for (const auto& cur_pred : cur_preds)
          {
            if (cur_pred.in.l.value == goal.in.l.value
                && cur_pred.in.a.value == goal.in.a.value
                && cur_pred.truth_value == goal.truth_value)
            {
              matched = true;
              break;
            }
          }
        });
      }
      break;

      case urs_wearable::Predicate::TYPE_LOW_BATTERY:
      {
        predicate_map_.find_fn(urs_wearable::Predicate::TYPE_LOW_BATTERY, [&goal, &matched](const std::vector<urs_wearable::Predicate>& cur_preds)
        {
          for (const auto& cur_pred : cur_preds)
          {
            if (cur_pred.low_battery.d.value == goal.low_battery.d.value
                && cur_pred.truth_value == goal.truth_value)
            {
              matched = true;
              break;
            }
          }
        });
      }
      break;

      case urs_wearable::Predicate::TYPE_SCANNED:
      {
        predicate_map_.find_fn(urs_wearable::Predicate::TYPE_SCANNED, [&goal, &matched](const std::vector<urs_wearable::Predicate>& cur_preds)
        {
          for (const auto& cur_pred : cur_preds)
          {
            if (cur_pred.scanned.d.value == goal.scanned.d.value
                && cur_pred.scanned.a.value == goal.scanned.a.value
                && cur_pred.truth_value == goal.truth_value)
            {
              matched = true;
              break;
            }
          }
        });
      }
      break;
    }

    if (!matched)
    {
      unsatisfied_goals.push_back(goal);
    }
  }

  return unsatisfied_goals;
}

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
      case urs_wearable::Predicate::TYPE_ABOVE:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_ABOVE, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.above.l0.value == cur_preds_it->above.l0.value
                && new_pred.above.l1.value == cur_preds_it->above.l1.value)
            {
              if (new_pred.truth_value == false)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->truth_value = new_pred.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base if its truth value is true
          if (!existed && new_pred.truth_value == true)
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (new_pred.truth_value == true)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_ABOVE, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_ALIGNED:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_ALIGNED, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.aligned.l0.value == cur_preds_it->aligned.l0.value
                && new_pred.aligned.l1.value == cur_preds_it->aligned.l1.value)
            {
              if (new_pred.truth_value == false)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->truth_value = new_pred.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base if its truth value is true
          if (!existed && new_pred.truth_value == true)
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (new_pred.truth_value == true)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_ALIGNED, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_AT:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_AT, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.at.d.value == cur_preds_it->at.d.value
                && new_pred.at.l.value == cur_preds_it->at.l.value)
            {
              if (new_pred.truth_value == false)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->truth_value = new_pred.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base if its truth value is true
          if (!existed && new_pred.truth_value == true)
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (new_pred.truth_value == true)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_AT, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_COLLIDED:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_COLLIDED, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.collided.l0.value == cur_preds_it->collided.l0.value
                && new_pred.collided.l1.value == cur_preds_it->collided.l1.value)
            {
              if (new_pred.truth_value == false)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->truth_value = new_pred.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base if its truth value is true
          if (!existed && new_pred.truth_value == true)
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (new_pred.truth_value == true)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_COLLIDED, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_HOVERED:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_HOVERED, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.hovered.d.value == cur_preds_it->hovered.d.value)
            {
              cur_preds_it->truth_value = new_pred.truth_value;
              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base if its truth value is true
          if (!existed && new_pred.truth_value == true)
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (new_pred.truth_value == true)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_HOVERED, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_IN:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_IN, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.in.l.value == cur_preds_it->in.l.value
                && new_pred.in.a.value == cur_preds_it->in.a.value)
            {
              if (new_pred.truth_value == false)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->truth_value = new_pred.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base if its truth value is true
          if (!existed && new_pred.truth_value == true)
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (new_pred.truth_value == true)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_IN, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_LOW_BATTERY:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_LOW_BATTERY, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.low_battery.d.value == cur_preds_it->low_battery.d.value)
            {
              cur_preds_it->truth_value = new_pred.truth_value;
              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base if its truth value is true
          if (!existed && new_pred.truth_value == true)
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (new_pred.truth_value == true)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_LOW_BATTERY, std::vector<urs_wearable::Predicate>{new_pred});
          }
        }
      }
      break;

      case urs_wearable::Predicate::TYPE_SCANNED:
      {
        if (!predicate_map_.update_fn(urs_wearable::Predicate::TYPE_SCANNED, [&new_pred](std::vector<urs_wearable::Predicate>& cur_preds)
        {
          bool existed = false;
          std::vector<urs_wearable::Predicate>::iterator cur_preds_it = cur_preds.begin();

          while (cur_preds_it != cur_preds.end())
          {
            // If the new predicate already existed, update the current predicate
            if (new_pred.scanned.d.value == cur_preds_it->scanned.d.value
                && new_pred.scanned.a.value == cur_preds_it->scanned.a.value)
            {
              if (new_pred.truth_value == false)
              {
                cur_preds.erase(cur_preds_it);
              }
              else  // Otherwise, just update its truth value
              {
                cur_preds_it->truth_value = new_pred.truth_value;
              }

              existed = true;
              break;
            }
            cur_preds_it++;
          }

          // If the predicate does not exist, then add it to the knowledge base if its truth value is true
          if (!existed && new_pred.truth_value == true)
          {
            cur_preds.push_back(new_pred);
          }
        }))
        {
          if (new_pred.truth_value == true)
          {
            predicate_map_.insert(urs_wearable::Predicate::TYPE_SCANNED, std::vector<urs_wearable::Predicate>{new_pred});
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

      // XXX: May only be needed for CPA
      // Add unknown goal predicates as false in :init, otherwise some plans can't be made
//      for (const auto& goal_pred : goal)
//      {
//        bool found = false;
//
//        // Verdict: It is faster when no exception is thrown
//        // Tested with 10,000 iterations
//        // 19 us    - No exception thrown, just using if statement
//        // 17342 us - Throw std::exception and catch by std::exception
//        // 18828 us - Throw std::out_of_range and catch by std::exception
//        // 18904 us - Throw std::out_of_range and catch by std::out_of_range
//        try
//        {
//          // This could throw std::out_of_range if key goal_pred.type is not found in the map
//          const std::vector<urs_wearable::Predicate>& preds = predicate_map_lt.at(goal_pred.type);
//          switch (goal_pred.type)
//          {
//            case urs_wearable::Predicate::TYPE_AT:
//              for (const auto& pred : preds)
//              {
//                if (goal_pred.at.d.value == pred.at.d.value
//                    && goal_pred.at.l.value == pred.at.l.value)
//                {
//                  found = true;
//                  break;
//                }
//              }
//              break;
//
//            case urs_wearable::Predicate::TYPE_HOVERED:
//              for (const auto& pred : preds)
//              {
//                if (goal_pred.hovered.d.value == pred.hovered.d.value)
//                {
//                  found = true;
//                  break;
//                }
//              }
//              break;
//          }
//        }
//        catch (const std::exception& e) {}
//
//        // If the goal is not found in predicate_map_, add the unknown goal to current_predicates as false
//        if (!found)
//        {
//          current_predicates.push_back(goal_pred);
//          urs_wearable::Predicate& p = current_predicates.back();
//          p.truth_value = false;
//        }
//      }

      predicate_map_lt.unlock();
      break;
    }

    // Wait until updating_instances_ == 0
    predicate_map_lt.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  /**** Create PDDL problem definition ****/
  // Create an object set for every object type we have
  std::set<std::uint8_t> object_area_id_set;
  std::set<std::uint8_t> object_drone_id_set;
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
        case urs_wearable::Predicate::TYPE_ABOVE:
          s = " (" + urs_wearable::PredicateAbove::NAME
            + " " + urs_wearable::ObjectLocation::TYPE + std::to_string(pred.above.l0.value)
            + " " + urs_wearable::ObjectLocation::TYPE + std::to_string(pred.above.l1.value)
            + ")";
          *preds_string += (pred.truth_value) ? s : " (not" + s + ")";

          object_location_id_set.insert(pred.above.l0.value);
          object_location_id_set.insert(pred.above.l1.value);
          break;

        case urs_wearable::Predicate::TYPE_ALIGNED:
          s = " (" + urs_wearable::PredicateAligned::NAME
            + " " + urs_wearable::ObjectLocation::TYPE + std::to_string(pred.aligned.l0.value)
            + " " + urs_wearable::ObjectLocation::TYPE + std::to_string(pred.aligned.l1.value)
            + ")";
          *preds_string += (pred.truth_value) ? s : " (not" + s + ")";

          object_location_id_set.insert(pred.aligned.l0.value);
          object_location_id_set.insert(pred.aligned.l1.value);
          break;

        case urs_wearable::Predicate::TYPE_AT:
          s = " (" + urs_wearable::PredicateAt::NAME
            + " " + urs_wearable::ObjectDrone::TYPE + std::to_string(pred.at.d.value)
            + " " + urs_wearable::ObjectLocation::TYPE + std::to_string(pred.at.l.value)
            + ")";
          *preds_string += (pred.truth_value) ? s : " (not" + s + ")";

          object_drone_id_set.insert(pred.at.d.value);
          object_location_id_set.insert(pred.at.l.value);
          break;

        case urs_wearable::Predicate::TYPE_COLLIDED:
          s = " (" + urs_wearable::PredicateCollided::NAME
            + " " + urs_wearable::ObjectLocation::TYPE + std::to_string(pred.collided.l0.value)
            + " " + urs_wearable::ObjectLocation::TYPE + std::to_string(pred.collided.l1.value)
            + ")";
          *preds_string += (pred.truth_value) ? s : " (not" + s + ")";

          object_location_id_set.insert(pred.collided.l0.value);
          object_location_id_set.insert(pred.collided.l1.value);
          break;

        case urs_wearable::Predicate::TYPE_HOVERED:
          s = " (" + urs_wearable::PredicateHovered::NAME
            + " " + urs_wearable::ObjectDrone::TYPE + std::to_string(pred.hovered.d.value)
            + ")";
          *preds_string += (pred.truth_value) ? s : " (not" + s + ")";

          object_drone_id_set.insert(pred.hovered.d.value);
          break;

        case urs_wearable::Predicate::TYPE_IN:
          s = " (" + urs_wearable::PredicateIn::NAME
            + " " + urs_wearable::ObjectLocation::TYPE + std::to_string(pred.in.l.value)
            + " " + urs_wearable::ObjectArea::TYPE + std::to_string(pred.in.a.value)
            + ")";
          *preds_string += (pred.truth_value) ? s : " (not" + s + ")";

          object_location_id_set.insert(pred.in.l.value);
          object_area_id_set.insert(pred.in.a.value);
          break;

        case urs_wearable::Predicate::TYPE_LOW_BATTERY:
          s = " (" + urs_wearable::PredicateLowBattery::NAME
            + " " + urs_wearable::ObjectDrone::TYPE + std::to_string(pred.low_battery.d.value)
            + ")";
          *preds_string += (pred.truth_value) ? s : " (not" + s + ")";

          object_drone_id_set.insert(pred.low_battery.d.value);
          break;

        case urs_wearable::Predicate::TYPE_SCANNED:
          s = " (" + urs_wearable::PredicateScanned::NAME
            + " " + urs_wearable::ObjectDrone::TYPE + std::to_string(pred.scanned.d.value)
            + " " + urs_wearable::ObjectArea::TYPE + std::to_string(pred.scanned.a.value)
            + ")";
          *preds_string += (pred.truth_value) ? s : " (not" + s + ")";

          object_drone_id_set.insert(pred.scanned.d.value);
          object_area_id_set.insert(pred.scanned.a.value);
          break;
      }
    }
  }

  // Create :objects string from all object sets we have
  std::string objects_str;
  if (object_area_id_set.size())
  {
    for (const auto i : object_area_id_set)
    {
      objects_str += " " + urs_wearable::ObjectArea::TYPE + std::to_string(i);
    }
    objects_str += " - " + urs_wearable::ObjectArea::TYPE;
  }
  else  // Add a dummy object, otherwise CPA might have problem parsing the problem definition
  {
    objects_str += " " + urs_wearable::ObjectArea::TYPE + "_dummy - " + urs_wearable::ObjectArea::TYPE;
  }

  if (object_drone_id_set.size())
  {
    for (const auto i : object_drone_id_set)
    {
      objects_str += " " + urs_wearable::ObjectDrone::TYPE + std::to_string(i);
    }
    objects_str += " - " + urs_wearable::ObjectDrone::TYPE;
  }
  else  // Add a dummy object, otherwise CPA might have problem parsing the problem definition
  {
    objects_str += " " + urs_wearable::ObjectDrone::TYPE + "_dummy - " + urs_wearable::ObjectDrone::TYPE;
  }

  if (object_location_id_set.size())
  {
    for (const auto i : object_location_id_set)
    {
      objects_str += " " + urs_wearable::ObjectLocation::TYPE + std::to_string(i);
    }
    objects_str += " - " + urs_wearable::ObjectLocation::TYPE;
  }
  else  // Add a dummy object, otherwise CPA might have problem parsing the problem definition
  {
    objects_str += " " + urs_wearable::ObjectLocation::TYPE + "_dummy - " + urs_wearable::ObjectLocation::TYPE;
  }

  return
    "(define (problem " + PROBLEM_NAME + ")\n"
    "  (:domain " + DOMAIN_NAME + ")\n"
    "  (:objects" + objects_str + ")\n"
//    "  (:init (and" + init_str + "))\n"
    "  (:init" + init_str + ")\n"
    "  (:goal (and" + goal_str + "))\n"
    ")\n";
}

// This method needs to be modified if there is a change in
// - action
std::vector<urs_wearable::Action>KnowledgeBase::parsePlan(const std::vector<std::string>& plan)
{
  std::vector<urs_wearable::Action> actions;

  for (const auto& plan_step : plan)
  {
    std::vector<std::string> tokens;
    boost::char_separator<char> sep {" ,()"};
    boost::tokenizer<boost::char_separator<char>> tok{plan_step, sep};
    for (const auto& t : tok)
    {
      tokens.push_back((std::string)t);
    }

    urs_wearable::Action action;
    if (tokens[0] == urs_wearable::ActionAscend::NAME)
    {
      action.type = urs_wearable::Action::TYPE_ASCEND;
      action.ascend.d.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDrone::TYPE.size()));
      action.ascend.l0.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
      action.ascend.l1.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionDescend::NAME)
    {
      action.type = urs_wearable::Action::TYPE_DESCEND;
      action.descend.d.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDrone::TYPE.size()));
      action.descend.l0.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
      action.descend.l1.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionGather::NAME)
    {
      action.type = urs_wearable::Action::TYPE_GATHER;
      action.gather.d.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDrone::TYPE.size()));
      action.gather.l0.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
      action.gather.l1.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionLand::NAME)
    {
      action.type = urs_wearable::Action::TYPE_LAND;
      action.land.d.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDrone::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionMove::NAME)
    {
      action.type = urs_wearable::Action::TYPE_MOVE;
      action.move.d.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDrone::TYPE.size()));
      action.move.l0.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
      action.move.l1.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionScan::NAME)
    {
      action.type = urs_wearable::Action::TYPE_SCAN;
      action.scan.d.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDrone::TYPE.size()));
      action.scan.l.value = std::stoi(tokens[2].erase(0, urs_wearable::ObjectLocation::TYPE.size()));
      action.scan.a.value = std::stoi(tokens[3].erase(0, urs_wearable::ObjectArea::TYPE.size()));
    }
    else if (tokens[0] == urs_wearable::ActionTakeoff::NAME)
    {
      action.type = urs_wearable::Action::TYPE_TAKEOFF;
      action.takeoff.d.value = std::stoi(tokens[1].erase(0, urs_wearable::ObjectDrone::TYPE.size()));
    }
    else
    {
      throw std::invalid_argument("Unrecognized action: " + tokens[0]);
    }
    actions.push_back(action);
  }

  return actions;
}

// This method needs to be modified if there is a change in
// - predicate
std::string KnowledgeBase::getPredicateString(const std::vector<urs_wearable::Predicate>& preds)
{
  if (preds.empty())
  {
    return "";
  }

  std::string pred_string;
  for (const auto& pred : preds)
  {
    switch (pred.type)
    {
      case urs_wearable::Predicate::TYPE_ABOVE:
        pred_string += urs_wearable::PredicateAbove::NAME
                    + "(" + std::to_string(pred.above.l0.value) + "," + std::to_string(pred.above.l1.value) + "), ";
        break;

      case urs_wearable::Predicate::TYPE_ALIGNED:
        pred_string += urs_wearable::PredicateAligned::NAME
                    + "(" + std::to_string(pred.aligned.l0.value) + "," + std::to_string(pred.aligned.l1.value) + "), ";
        break;

      case urs_wearable::Predicate::TYPE_AT:
        pred_string += urs_wearable::PredicateAt::NAME
                    + "(" + std::to_string(pred.at.d.value) + "," + std::to_string(pred.at.l.value) + "), ";
        break;

      case urs_wearable::Predicate::TYPE_COLLIDED:
        pred_string += urs_wearable::PredicateCollided::NAME
                    + "(" + std::to_string(pred.collided.l0.value) + "," + std::to_string(pred.collided.l1.value) + "), ";
        break;

      case urs_wearable::Predicate::TYPE_HOVERED:
        pred_string += urs_wearable::PredicateHovered::NAME
                    + "(" + std::to_string(pred.hovered.d.value) + "), ";
        break;

      case urs_wearable::Predicate::TYPE_IN:
        pred_string += urs_wearable::PredicateIn::NAME
                    + "(" + std::to_string(pred.in.l.value) + "," + std::to_string(pred.in.a.value) + "), ";
        break;

      case urs_wearable::Predicate::TYPE_LOW_BATTERY:
        pred_string += urs_wearable::PredicateLowBattery::NAME
                    + "(" + std::to_string(pred.low_battery.d.value) + "), ";
        break;

      case urs_wearable::Predicate::TYPE_SCANNED:
        pred_string += urs_wearable::PredicateScanned::NAME
                    + "(" + std::to_string(pred.scanned.d.value) + "," + std::to_string(pred.scanned.a.value) + "), ";
        break;
    }
  }

  pred_string.pop_back();   // Remove a trailing space
  pred_string.pop_back();   // Remove a comma
  return pred_string;
}

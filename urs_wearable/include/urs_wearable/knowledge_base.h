#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_KNOWLEDGE_BASE_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_KNOWLEDGE_BASE_H_

#include <atomic>
#include <cstdint>
#include <vector>

#include "libcuckoo/cuckoohash_map.hh"
#include <ros/ros.h>

#include "urs_wearable/location_table.h"
#include "urs_wearable/Action.h"
#include "urs_wearable/Predicate.h"
#include "urs_wearable/State.h"

class KnowledgeBase
{
public:
  // We use uint8_t here to match with the type of 'type' in Predicate.msg
  typedef std::uint8_t predicate_type;

  // We use uint8_t here to match with the type of 'executor_id' in Feedback.msg
  typedef std::uint8_t executor_id_type;

  LocationTable location_table_;

  KnowledgeBase(const std::string& problem_name, const std::string& domain_name, const std::string& planner_service_name)
  : PROBLEM_NAME(problem_name), DOMAIN_NAME(domain_name), PLANNER_SERVICE_NAME(planner_service_name)
  {
    state_pub_ = nullptr;
  }

  ~KnowledgeBase()
  {
    if (state_pub_)
    {
      state_pub_->shutdown();
    }
  }

  executor_id_type registerExecutor()
  {
    executor_id_type id_mutex = executor_id_++;
    while (executor_map_.contains(id_mutex))
    {
      id_mutex = executor_id_++;
    }
    return id_mutex;
  }

  bool unregisterExecutor(executor_id_type id)
  {
    return executor_map_.erase(id);
  }

  // Insert or update predicates, may also erase depending on the ERASE_WHEN_FALSE constant defined in Predicate*.msg
  void upsertPredicates(const std::vector<urs_wearable::Predicate>&);

  std::vector<urs_wearable::Predicate> getPredicateList(predicate_type type)
  {
    std::vector<urs_wearable::Predicate> v;
    predicate_map_.find(type, v);
    return v;
  }

  // getPlan can only be called once from the same executor, the subsequent calls will return an empty plan
  void getPlan(executor_id_type, const std::vector<urs_wearable::Predicate>&, std::vector<std::string>&);
  void getPlanIfPlanHasChanged(executor_id_type, std::vector<std::string>&);

  // Throw std::invalid_argument if there is an unrecognized action
  std::vector<urs_wearable::Action> parsePlan(const std::vector<std::string>&);

  void setStatePub(ros::Publisher* state_pub)
  {
    state_pub_ = state_pub;
  }

  void publish();

private:
  struct Executor
  {
    std::vector<urs_wearable::Predicate> goal;
    std::vector<std::string> plan;
    bool plan_has_changed;
  };

  std::atomic<executor_id_type> executor_id_ {0};
  cuckoohash_map<executor_id_type, struct Executor> executor_map_;

  cuckoohash_map<predicate_type, std::vector<urs_wearable::Predicate>> predicate_map_;
  std::atomic<unsigned int> updating_instances_ {0};

  const std::string PROBLEM_NAME;
  const std::string DOMAIN_NAME;
  const std::string PLANNER_SERVICE_NAME;

  ros::Publisher* state_pub_;

  void replan();

  std::string getProblemDef(const std::vector<urs_wearable::Predicate>&);
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_KNOWLEDGE_BASE_H_ */

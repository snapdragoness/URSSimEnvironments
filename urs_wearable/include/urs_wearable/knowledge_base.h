#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_KNOWLEDGE_BASE_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_KNOWLEDGE_BASE_H_

#include <atomic>
#include <cstdint>
#include <mutex>
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

  // We use uint8_t here to match with the type of 'executor_id' in Feedback.msg, SetGoal.srv
  typedef std::uint32_t executor_id_type;

  LocationTable loc_table_;

  KnowledgeBase(const std::string& domain_name, const std::string& problem_name)
  : DOMAIN_NAME(domain_name), PROBLEM_NAME(problem_name)
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
    executor_id_type id = executor_id_++;
    while (executor_map_.contains(id))
    {
      id = executor_id_++;
    }
    return id;
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

  std::vector<urs_wearable::Predicate> getUnsatisfiedGoals(const std::vector<urs_wearable::Predicate>&);

  // getPlan can only be called once from the same executor, the subsequent calls will return an empty plan
  void getPlan(executor_id_type, const std::vector<urs_wearable::Predicate>&, std::vector<std::string>&);
  bool getPlanIfPlanHasChanged(executor_id_type, std::vector<std::string>&);

  // Throw std::invalid_argument if there is an unrecognized action
  std::vector<urs_wearable::Action> parsePlan(const std::vector<std::string>&);

  void setStatePub(ros::Publisher* state_pub)
  {
    state_pub_ = state_pub;
  }

  void publish();

  std::string domain_file;
  std::string planner_command;
  std::string problem_path;
  std::string tmp_path;

  static std::string getPredicateString(const std::vector<urs_wearable::Predicate>&);

private:
  struct Executor
  {
    std::vector<urs_wearable::Predicate> goal;
    std::vector<std::string> plan;
    bool plan_has_changed;
  };

  std::atomic<executor_id_type> executor_id_{0};
  cuckoohash_map<executor_id_type, struct Executor> executor_map_;

  cuckoohash_map<predicate_type, std::vector<urs_wearable::Predicate>> predicate_map_;
  std::atomic<unsigned int> updating_instances_ {0};

  const std::string PROBLEM_NAME;
  const std::string DOMAIN_NAME;

  ros::Publisher* state_pub_;
  std::mutex state_pub_mutex_;

  void replan();

  std::string getProblemDef(const std::vector<urs_wearable::Predicate>&);

  static std::atomic<unsigned int> executorReplanID;
  static std::string exec(const char*);
  static std::vector<std::string> parseFF(std::string);
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_KNOWLEDGE_BASE_H_ */

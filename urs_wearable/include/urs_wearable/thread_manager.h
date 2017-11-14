#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_THREAD_MANAGER_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_THREAD_MANAGER_H_

#include <vector>
#include <unordered_map>
#include <algorithm>
#include <boost/thread.hpp>

class ThreadManager {
  boost::thread_group threadGroup;
  std::unordered_multimap<int, boost::thread*> umm;
  boost::mutex mut;

public:
  void add(int id, boost::thread* thread)
  {
    mut.lock();
    std::pair<int, boost::thread*> pair(id, thread);
    umm.insert(pair);
    threadGroup.add_thread(thread);
    mut.unlock();
  }

  void remove(int id)
  {
    mut.lock();
    auto its = umm.equal_range(id);
    for (auto it = its.first; it != its.second; ++it)
    {
      it->second->interrupt();
      threadGroup.remove_thread(it->second);
      delete it->second;
    }
    umm.erase(id);
    mut.unlock();
  }
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_THREAD_MANAGER_H_ */

#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_POOL_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_POOL_H_

#include <vector>
#include <boost/thread.hpp>

template<typename T, const int POOL_SIZE>
  class Pool
  {
    std::vector<int> idList;
    boost::mutex mut;

  public:
    T data[POOL_SIZE];

    Pool()
    {
      idList.reserve(POOL_SIZE);
      for (int i = POOL_SIZE - 1; i >= 0; i--)
      {
        idList.push_back(i);
      }
    }

    int newId(std::vector<int>& allocatedIdList)
    {
      mut.lock();
      int id = idList.back();
      idList.pop_back();
      mut.unlock();
      allocatedIdList.push_back(id);
      return id;
    }

    void retrieveId(std::vector<int>& allocatedIdList)
    {
      mut.lock();
      for (std::vector<int>::reverse_iterator rit = allocatedIdList.rbegin(); rit != allocatedIdList.rend(); rit++)
      {
        idList.push_back(*rit);
      }
      mut.unlock();
    }
  };

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_POOL_H_ */

#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_POOL_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_POOL_H_

#include <vector>

template<typename T, const int POOL_SIZE>
  class Pool
  {
    std::vector<int> idList;

    void init()
    {
      idList.reserve(POOL_SIZE);
      for (int i = POOL_SIZE - 1; i >= 0; i--)
      {
        idList.push_back(i);
      }
    }

  public:
    T data[POOL_SIZE];

    Pool()
    {
      init();
    }

    int newId(std::vector<int>& allocatedIdList)
    {
      int id = idList.back();
      idList.pop_back();
      allocatedIdList.push_back(id);
      return id;
    }

    void retrieveId(std::vector<int>& allocatedIdList)
    {
      for (std::vector<int>::reverse_iterator rit = allocatedIdList.rbegin(); rit != allocatedIdList.rend(); rit++)
      {
        idList.push_back(*rit);
      }
    }
  };

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_POOL_H_ */

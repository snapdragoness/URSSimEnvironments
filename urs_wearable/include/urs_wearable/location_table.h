#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_LOCATION_TABLE_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_LOCATION_TABLE_H_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <set>
#include <stdexcept>
#include <vector>

#include "libcuckoo/cuckoohash_map.hh"

#include "urs_wearable/Location.h"
#include "urs_wearable/PoseEuler.h"

class LocationTable
{
public:
  // We use uint8_t here to match with the type of 'location_id' in
  // Location.msg, ObjectLocationID.msg, LocationAdd.srv, LocationRemove.srv
  typedef std::uint8_t location_id_type;

  cuckoohash_map<location_id_type, urs_wearable::PoseEuler> map_;

  location_id_type insert(const urs_wearable::PoseEuler& pose)
  {
    {
      std::lock_guard<std::mutex> lock(unused_id_set_mutex_);
      if (!unused_id_set_.empty())
      {
        std::set<location_id_type>::iterator it = unused_id_set_.begin();
        location_id_type id = *it;
        unused_id_set_.erase(it);
        map_.insert(id, pose);
        return id;
      }
    }

    location_id_type id_mutex = id_++;
    if (id_mutex + 1 == 0)
    {
      throw std::length_error("Location table full. The size of location_id type should be increased.");
    }
    map_.insert(id_mutex, pose);
    return id_mutex;
  }

  bool erase(location_id_type id)
  {
    {
      std::lock_guard<std::mutex> lock(unused_id_set_mutex_);
      unused_id_set_.insert(id);
    }
    return map_.erase(id);
  }

  bool update(location_id_type id, const urs_wearable::PoseEuler& pose)
  {
    return map_.update_fn(id, [&pose](urs_wearable::PoseEuler &p)
    {
      p = pose;
    });
  }

private:
  std::atomic<location_id_type> id_ {0};

  std::set<location_id_type> unused_id_set_;
  std::mutex unused_id_set_mutex_;
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_LOCATION_TABLE_H_ */
#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_LOCATION_TABLE_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_LOCATION_TABLE_H_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <set>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/Pose.h>
#include "libcuckoo/cuckoohash_map.hh"

#include "urs_wearable/Location.h"

class LocationTable
{
public:
  typedef std::uint8_t area_id_t; // This matches with its definition in ObjectArea.msg, AddArea.srv, RemoveArea.srv
  typedef std::uint8_t loc_id_t;  // This matches with its definition in ObjectLocation.msg, AddLocation.srv, RemoveLocation.srv

  cuckoohash_map<area_id_t, std::vector<loc_id_t>> area_map_;
  cuckoohash_map<loc_id_t, geometry_msgs::Pose> loc_map_;

  loc_id_t insert(const geometry_msgs::Pose& pose)
  {
    {
      std::lock_guard<std::mutex> lock(unused_id_set_mutex_);
      if (!unused_id_set_.empty())
      {
        std::set<loc_id_t>::iterator it = unused_id_set_.begin();
        loc_id_t id = *it;
        unused_id_set_.erase(it);
        loc_map_.insert(id, pose);
        return id;
      }
    }

    loc_id_t id_mutex = id_++;
    if (id_mutex + 1 == 0)
    {
      throw std::length_error("Location table full. The size of location_id type should be increased.");
    }
    loc_map_.insert(id_mutex, pose);
    return id_mutex;
  }

  bool erase(loc_id_t id)
  {
    {
      std::lock_guard<std::mutex> lock(unused_id_set_mutex_);
      unused_id_set_.insert(id);
    }
    return loc_map_.erase(id);
  }

  bool update(loc_id_t id, const geometry_msgs::Pose& pose)
  {
    return loc_map_.update_fn(id, [&pose](geometry_msgs::Pose &p)
    {
      p = pose;
    });
  }

private:
  std::atomic<loc_id_t> id_ {0};

  std::set<loc_id_t> unused_id_set_;
  std::mutex unused_id_set_mutex_;
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_LOCATION_TABLE_H_ */

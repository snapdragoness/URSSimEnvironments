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
  typedef std::uint32_t area_id_t;  // This matches with its definition in Area.msg, ObjectArea.msg, AddArea.srv, RemoveArea.srv
  typedef std::uint32_t loc_id_t;   // This matches with its definition in Location.msg, ObjectLocation.msg, AddLocation.srv, RemoveLocation.srv

  typedef struct                    // This matches with its definition in Area.msg, AddArea.srv
  {
    loc_id_t loc_id_left;
    loc_id_t loc_id_right;
  } Area;

  cuckoohash_map<area_id_t, Area> area_map_;
  cuckoohash_map<loc_id_t, geometry_msgs::Pose> loc_map_;

  area_id_t insertArea(const Area& area)
  {
    {
      std::lock_guard<std::mutex> lock(unused_area_id_set_mutex_);
      if (!unused_area_id_set_.empty())
      {
        std::set<area_id_t>::iterator it = unused_area_id_set_.begin();
        area_id_t id = *it;
        unused_area_id_set_.erase(it);
        area_map_.insert(id, area);
        return id;
      }
    }

    area_id_t id_mutex = area_id_++;
    if (id_mutex + 1 == 0)
    {
      throw std::length_error("location_table.area_map_ is full");
    }
    area_map_.insert(id_mutex, area);
    return id_mutex;
  }

  bool eraseArea(area_id_t id)
  {
    {
      std::lock_guard<std::mutex> lock(unused_area_id_set_mutex_);
      unused_area_id_set_.insert(id);
    }
    return area_map_.erase(id);
  }

  bool updateArea(area_id_t id, const Area& area)
  {
    return area_map_.update_fn(id, [&area](Area &a)
    {
      a = area;
    });
  }

  loc_id_t insertLocation(const geometry_msgs::Pose& pose)
  {
    {
      std::lock_guard<std::mutex> lock(unused_loc_id_set_mutex_);
      if (!unused_loc_id_set_.empty())
      {
        std::set<loc_id_t>::iterator it = unused_loc_id_set_.begin();
        loc_id_t id = *it;
        unused_loc_id_set_.erase(it);
        loc_map_.insert(id, pose);
        return id;
      }
    }

    loc_id_t id_mutex = loc_id_++;
    if (id_mutex + 1 == 0)
    {
      throw std::length_error("location_table.loc_map_ is full");
    }
    loc_map_.insert(id_mutex, pose);
    return id_mutex;
  }

  bool eraseLocation(loc_id_t id)
  {
    {
      std::lock_guard<std::mutex> lock(unused_loc_id_set_mutex_);
      unused_loc_id_set_.insert(id);
    }
    return loc_map_.erase(id);
  }

  bool updateLocation(loc_id_t id, const geometry_msgs::Pose& pose)
  {
    return loc_map_.update_fn(id, [&pose](geometry_msgs::Pose &p)
    {
      p = pose;
    });
  }

private:
  std::atomic<loc_id_t> area_id_ {0};
  std::set<loc_id_t> unused_area_id_set_;
  std::mutex unused_area_id_set_mutex_;

  std::atomic<loc_id_t> loc_id_ {0};
  std::set<loc_id_t> unused_loc_id_set_;
  std::mutex unused_loc_id_set_mutex_;
};

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_LOCATION_TABLE_H_ */

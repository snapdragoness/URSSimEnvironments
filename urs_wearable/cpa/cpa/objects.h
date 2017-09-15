#ifndef URS_WEARABLE_CPA_CPA_OBJECTS_H_
#define URS_WEARABLE_CPA_CPA_OBJECTS_H_

#include <iostream>
#include <algorithm>
#include <set>

class Objects {
public:
  std::string type;
  std::set<std::string> objs;

  static std::string objToString(const std::string& type, std::string obj)
  {
    obj.erase(0, type.length());
    return obj;
  }

  static int objToInt(const std::string& type, std::string obj)
  {
    obj.erase(0, type.length());
    return stoi(obj);
  }

  static int objToDouble(const std::string& type, std::string obj)
  {
    obj.erase(0, type.length());
    std::replace(obj.begin(), obj.end(), '_', '.');
    return stod(obj);
  }

  static std::string stringToObj(const std::string& type, const std::string& value)
  {
    return type + value;
  }

  static std::string intToObj(const std::string& type, int value)
  {
    return type + std::to_string(value);
  }

  static std::string doubleToObj(const std::string& type, double value)
  {
    std::string s = std::to_string(value);
    std::replace(s.begin(), s.end(), '.', '_');
    return type + s;
  }

  Objects(const std::string& type)
  {
    this->type = type;
  }

  void insert(const std::string& value)
  {
    objs.insert(Objects::stringToObj(type, value));
  }

  void insert(int value)
  {
    objs.insert(Objects::intToObj(type, value));
  }

  void insert(double value)
  {
    objs.insert(Objects::doubleToObj(type, value));
  }

  void clear()
  {
    objs.clear();
  }
};

#endif /* URS_WEARABLE_CPA_CPA_OBJECTS_H_ */

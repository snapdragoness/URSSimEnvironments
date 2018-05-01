#ifndef URS_WEARABLE_CPA_CPA_OBJECTS_H_
#define URS_WEARABLE_CPA_CPA_OBJECTS_H_

#include <iostream>
#include <algorithm>
#include <set>
#include <string>

class Objects {
public:
  std::string type;
  std::set<std::string> objs;

  std::string objToString(std::string obj)
  {
    obj.erase(0, type.length());
    return obj;
  }

  int objToInt(std::string obj)
  {
    obj.erase(0, type.length());
    return std::stoi(obj);
  }

  double objToDouble(std::string obj)
  {
    obj.erase(0, type.length());
    std::replace(obj.begin(), obj.end(), '_', '.');
    return std::stod(obj);
  }

  std::string stringToObj(const std::string& value)
  {
    return type + value;
  }

  std::string intToObj(int value)
  {
    return type + std::to_string(value);
  }

  std::string doubleToObj(double value)
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
    objs.insert(stringToObj(value));
  }

  void insert(int value)
  {
    objs.insert(intToObj(value));
  }

  void insert(double value)
  {
    objs.insert(doubleToObj(value));
  }

  void clear()
  {
    objs.clear();
  }
};

#endif /* URS_WEARABLE_CPA_CPA_OBJECTS_H_ */

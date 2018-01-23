// From https://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal

#ifndef URS_WEARABLE_INCLUDE_URS_WEARABLE_COLORMOD_H_
#define URS_WEARABLE_INCLUDE_URS_WEARABLE_COLORMOD_H_

#include <ostream>

namespace Color
{
enum Code
{
  FG_RED = 31, FG_GREEN = 32, FG_BLUE = 34, FG_DEFAULT = 39, BG_RED = 41, BG_GREEN = 42, BG_BLUE = 44, BG_DEFAULT = 49
};
class Modifier
{
  Code code;

public:
  Modifier(Code pCode) : code(pCode){}

  friend std::ostream& operator<<(std::ostream& os, const Modifier& mod)
  {
    return os << "\033[" << mod.code << "m";
  }
};

Color::Modifier fg_red(Color::FG_RED);
Color::Modifier fg_green(Color::FG_GREEN);
Color::Modifier fg_blue(Color::FG_BLUE);
Color::Modifier fg_default(Color::FG_DEFAULT);
Color::Modifier bg_red(Color::BG_RED);
Color::Modifier bg_green(Color::BG_GREEN);
Color::Modifier bg_blue(Color::BG_BLUE);
Color::Modifier bg_default(Color::BG_DEFAULT);
}

#endif /* URS_WEARABLE_INCLUDE_URS_WEARABLE_COLORMOD_H_ */

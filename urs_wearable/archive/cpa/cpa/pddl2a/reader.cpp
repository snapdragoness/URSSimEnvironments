#include <set>
#include <map>
#include <algorithm>    // for set_intersection, set_union
#include <functional>   // for less
#include "reader.h"


// prototype of bison-generated parser function
int yyparse();

StringList Reader::name(Literals* x)
{
  StringList y;
  Literals::iterator it;

  for (it = x->begin(); it != x->end(); it++) {
    y.insert(name(*it));
  }

  return y;
}

string Reader::name(Literal x)
{
  unsigned short i = x / 2;
  StringList::iterator it;

  if (i >= m_fluents.size())
    return NULL;

  it = m_fluents.begin();

  for (i = 0; i < x/2; i++) {
    it++;
  }

  if (x % 2 == 0)
    return *it;
  
  return (NEGATION_SYMBOL + (*it));
}

int Reader::read()
{
  return yyparse();
}

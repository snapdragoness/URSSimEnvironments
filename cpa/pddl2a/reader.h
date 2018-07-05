#ifndef READER_H
#define READER_H

#include "define.h"

using namespace std;

class Reader
{
 public:
  /* domain name */
  string m_domain_name;

  /* reading from the input file */
  StringList m_fluents;
  StringList m_actions;
  PropositionList m_propositions;
  StringList2 m_init;
  StringList m_goal;
  
  //domain 
  pDomain m_domain;
  pProblem m_problem;
  
public:
  string name(Literal x);
  StringList name(Literals* x);
  int read();
};

#endif

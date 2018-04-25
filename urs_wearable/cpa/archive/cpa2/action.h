#ifndef ACTION_H
#define ACTION_H

#include "define.h"
#include "state.h"

class Planner;

/***********************************************************************
 class: Action
************************************************************************/
class Action {
 protected:
  const Planner* m_planner;
  string m_name;          // action name
  ExecList m_execs;       // executability conditions
  EffectList m_effects;   // conditional effects
  ExecList m_imposs;      // impossibility conditions

  //add mar 31 2008 for hgraph
  int m_index;

 public:
  /* constructor & destructor */
  Action(const Planner* planner, const string& name);
  ~Action();

  /* get/set functions */
  string get_name() const;
  void set_name(const string& name);

  const EffectList* get_effects() const;
  const ExecList* get_execs() const;
  const ExecList* get_imposs() const;

  void add_exec(const Literals ex);
  void add_imposs(const Literals imp);
  void add_effect(const Literals hd, const Literals bd);
   
  //add mar 31 2008 for hgraph
  void add_exec(const ExecList execs);
  void add_effect(const Effect eff);
  void add_effect(const Effect eff, const Literals execs);


  void set_index(int index);
  int get_index() const;

  /* main functions */
  bool is_executable(const State* s) const;
  bool is_executable(const CState* cs) const;
  void print() const;
};

typedef vector<Action> ActionList;

#endif

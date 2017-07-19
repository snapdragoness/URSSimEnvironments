#ifndef PLAN_H
#define PLAN_H

#include "define.h"
#include "reader.h"
#include "timer.h"
#include "state.h"
#include "action.h"

/***********************************************************************
 Other classes used in Planner
************************************************************************/
typedef list<string> RawPlan;


typedef list<string> RawPlan;
typedef list<Action*> ListPtrActions;
typedef pair<Action*, int> actionpos;

struct a_possible 
{
   unsigned short index;
   ListPtrActions list_action;
};


/***********************************************************************
 Main class
************************************************************************/
class Planner 
{
  friend class State;
  friend class Action;
 public:
  /* input parameters */
  Semantics m_semantics;
  Task m_task;
  Algorithm m_algorithm;
  RawPlan m_plan;
  bool m_detect_essential;
  bool m_output_decisive;

protected:
  map<string,int> my_action_map;
  list<int> my_plan;
  const Reader* m_reader;       // reader
  Timer* m_timer;               // timer
  bool rebuild_domain;
  string m_domain_name;         // domain name
  map<string,int> m_action_map; // action mapping
  Literals m_literals;          // fluent literals
  ActionList m_actions;         // action description
  StaticLaws m_statics;         // static laws
  CState m_init_cstate;         // init c-state
  Literals m_goal;              // set of literals in the goal
  set<Literals> m_gd;           // goal description, in CNF form:
                                // {(f ^ h ) v g} is represented as
                                // {{f,g},{h,g}}
  map<string,Literal> m_map;    // literal mapping
  int n_cstates;

  //add Mar 31 2008
  ActionList m_iactions;
  map<string,int> m_iaction_map; // action mapping

  int *m_iactions_bd; 
  int *m_fsatisfied;
  int *m_fexecuted;
  int *n_actions;

  ListPtrActions *fa_links;
  ListPtrActions true_links;
  

protected:
  /* for searching */
  StateTable m_states;   // state table (reached states)
  CStateTable m_cstates; // visited cstates
  PriorityCStateQueue m_queue; // current queue

 public:
  /* constructor and destructor */
  Planner(const Reader* reader, Timer* timer);
  virtual ~Planner();

  /* main function */
  bool main();

  /* generic functions */
  bool intersect(const Literals* x, const Literals* y) const;
  void rebuild();
  void printplan(CState* myfinalstate);
  /* grounding functions */
  Literal ground(const string& x) const;
  Literals ground(const StringList* x) const;
  string convert(const Literal& x) const;
  StringList convert(const Literals& x) const;

  /* build the domain & check errors */
  bool build_domain();
  bool build_goal();
  bool build_init();

  Action* add_action(const string str);

  /* printing functions */
  void print_statistics() const;
  void print(const Literal& l) const;
  void print(const Literals& x) const;
  void print_interal_domain() const;
  void print_summary() const;
  void pos_print(const Literal& l) const;
  void pos_print(const Literals& x) const;

  //add Mar 31 2008
  int add_iaction(const string str);
  int build_hgraph_relaxed(Literals m_literals);
  bool goal_satisfies(const Literals* goal, const Literals* x);
  bool goal_satisfied(const set<Literals> gd, const Literals* x);

  /* functions used during the search */  
protected:  
  State* add_state(State* s);
  Literals negate(const Literals& x) const;  
public:
  bool search();
  bool execute();   // execute a plan and print out steps
protected:
  State* next_state_pc(State* s, const Action* act);
  State* next_state_ph(State* s, const Action* act);
  CState* next_cstate(CState* csin, const Action* act);
  bool explore_once();
};

#endif

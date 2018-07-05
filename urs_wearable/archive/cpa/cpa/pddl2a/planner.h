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
  ofstream plcout;
  bool completedinfo;

protected:
  Reader* m_reader;       // reader
  Timer* m_timer;               // timer
  
  string m_domain_name;         // domain name
  map<string,int> m_action_map; // action mapping
  Literals m_literals;          // fluent literals
  ActionList m_actions;         // action description
  StaticLaws m_statics;         // static laws
  CState m_init_cstate;         // init c-state
  Literals m_goal;              // goal
  Literals m_cgoal;             // closure of the goal, for speedup
  map<string,Literal> m_map;    // literal mapping

  bool m_typing;			
;

protected:
  /* for searching */
  StateTable m_states;   // state table (reached states)
  CStateTable m_cstates; // visited cstates
  PriorityCStateQueue m_queue; // current queue

 public:
  /* constructor and destructor */
  Planner(Reader* reader, Timer* timer);
  virtual ~Planner();

  /* main function */
  bool main();

  /* generic functions */
  bool intersect(const Literals* x, const Literals* y) const;
  
  /* grounding functions */
  Literal ground(const string& x) const;
  Literals ground(const StringList* x) const;
  string convert(const Literal& x) const;
  StringList convert(const Literals& x) const;

  void copy_formula(Formula *f, Formula *cp);
  void negation_movein(Formula* f);
  void formula_flattening(Formula* f);
  bool formula_grounding(Formula* f, const map<string,string> v_map);
  bool term_grounding(Formula* f, const map<string,string> v_map);
  void qterm_grounding(Formula* f, const Term qterm, const Term qobj);
  void flattening(Formula* f, Formula* qformula, const Term qterm, const Term qobj,  const
  OperatorType opr);
  void parameter_grounding(Structure& inf, unsigned int index);
  void dequantified_condition(Formula* f);
  void add_conditional(Formula *f, Proposition *act);
  /* Grounding the domain */
  bool grounding(); 
  StringList predicate_grounding(StringList *pinit);
  void action_grounding();
  void derived_grounding();

  void parameter_grounding(Structure& inf);

  StringList2 convertf_to_dnf(Formula* f);
  StringList2 dnf_conjunction(StringList2 str1, StringList2 str2);
  StringList2 dnf_disjunction(StringList2 str1, StringList2 str2);
  void print (StringList2 str2);

  /* build the domain & check errors */
  bool build();

  
  void add_proposition(const Structure& p);
  StringList build_fluent_list(const LiteralList ll);
  bool quantified_grounding(LiteralList2* inf);

  /* convert PDDL to PL */
  void convert_pddl_to_pl();
  void print_types(const TypeList *_types);
  void print_objects(const TermList *_objs);
  void print_consts(const TermList *_consts);
  void print_types_rules(const TypeList *_types);
  void print_predicates(const AtomList *_atoms);
  string strupcase(const string& str);
  string strlowercase(const string& str);
  string strvar(const string& str);
  void print_actions_def(const StructureList _actions);
  void print_actions_pre(const StructureList _actions);
  void print_actions_eff(const StructureList _actions);
  void print_actions_conds(Formula* f, Structure& _act, StructureList& _actions_conds);
  void print_init(Formula* _init);
  void print_goal(Formula* _goal);

  Action* add_action(const string str);
  
  //need to pass args to 
  StringList build_goal();
  StringList2 build_init();

  /* printing functions */
  void print_statistics() const;
  void print(const Literal& l) const;
  void print(const Literals& x) const;
  void print_domain() const;
  void print_summary() const;

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

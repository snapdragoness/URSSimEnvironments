#ifndef DEFINE_H
#define DEFINE_H

#include <string>
#include <iostream>
#include <set>
#include <list>
#include <queue>
#include <map>
#include <vector>
#include <algorithm>
#include <sys/time.h>
#include <sys/resource.h>

using namespace std;


/***********************************************************************
 Do not change the following
************************************************************************/

class Action;
class Domain;

#define NEGATE(l,x) (x = ((l % 2 == 0) ? l+1 : l-1))

/***********************************************************************
 Constants
************************************************************************/
#define NEGATION_SYMBOL "-"
#define EMPTY " "

// maximum fluent/action name length
#define MAX_LEN 200

#define ERROR_001 "Invalid fluent/literal. Check the fluent declaration"
#define ERROR_002 ""
#define ERROR_003 ""
#define ERROR_004 ""

enum Algorithm {
  GREEDY
};

enum Semantics {
  PC,
  PH
};

enum Task {
  SEARCH,
  DOPLAN
};

// timers
#define READ_TIMER 0
#define BUILD_TIMER 1
#define SEARCH_TIMER 2
#define GOAL_TIMER 3
#define HEURISTIC_TIMER 4
#define RESULT_TIMER 5
#define TRANSITION_LOOKUP_TIMER 6
#define EXECUTABILITY_TIMER 7
#define ACTUAL_RESULT_TIMER 8
#define DIRECT_EFFECT_TIMER 9
#define INDIRECT_EFFECT_TIMER 10
#define IDLE_TIMER 11
#define NEXTSTATE_TIMER 12
#define COMMUNICATION_TIMER 13
/***********************************************************************
 Types -- For scanner & parser
************************************************************************/
// for scanner & parser
typedef set<string> StringList;
typedef set<StringList> StringList2;

enum PropositionType {
  STATIC,
  DYNAMIC,
  EXECUTABILITY,
  IMPOSSIBILITY
};

enum OperatorType {
  AND_,
  OR_,
  IMPLY_,
  NOT_,
  QFORALL_,
  QEXISTS_,
  WHEN_,
  UNIOP_,
  UNKNOWN_,
  ONEOF_
};


typedef int Fluent;
typedef set<Fluent> Fluents;

// literals
typedef unsigned short Literal;

// set of literals and states
typedef set<Literal> Literals;

class Proposition {
 public:
  PropositionType n_type;
  string act_name;
  StringList precond;
  StringList effect;
  
  const StringList* get_precondition() const{
    return &precond;
  }
  const StringList* get_effect() const {
    return &effect;
  }
};

typedef list<Proposition> PropositionList;


//Type class
class Type {
 public:
 string _name;
 string _type;
 
 bool _typed;
 void print() {
   cout << _name << "-"; 
   if (_typed)
     cout <<_type;
 }
};
typedef list<Type> TypeList;

//Term class
class Term {
 public:
 string _name;
 TypeList *_type;
 string _ground; 

 bool constant;
 bool variable;

 bool compare_type (const TypeList *type) const 
 {
  TypeList::const_iterator ifrom;
  TypeList::const_iterator ito;
  
  for (ifrom = _type->begin(); ifrom != _type->end(); ifrom++)
   for (ito = type->begin(); ito != type->end(); ito++)
    if (ifrom->_name == ito->_name)
	return true;	
  return false;

 }
 
 void print() {
  cout << _ground;
 }
};
//List of terms
typedef list<Term> TermList;

class Predicate {
 public:
 string _name; 
 TypeList *_parameter;

 void print() { cout << _name; }
 void print_pl() { cout << strlowercase(_name); }
};


//Atomic formula class
class Atom {
 public:
 Predicate *_predicate;
 TermList _term;

 void print() {
 TermList::iterator it;
 _predicate->print();
  // print out list of terms
  for (it = _term.begin(); it != _term.end(); it++) {
    cout << "_";
    it->print();
  }
 }
 void print_pl() {
 TermList::iterator it;
 _predicate->print();
  // print out list of terms
  for (it = _term.begin(); it != _term.end(); it++) {
    cout << "_";
    it->print();
  }
 }
};
typedef list<Atom> AtomList;

//Literal class
class LiteralTerm {
 public:
 Atom _atom;
 bool _neg;

 void print()
 {
   if (_neg) cout<<"-"; 
   _atom.print();
 }
};
typedef list<LiteralTerm> LiteralList;
typedef list<LiteralList> LiteralList2;


//Formula class
class Formula {
 public: 
 OperatorType _operator;
 TermList _quantified_parameters;
 LiteralTerm _term;
 Formula* _t1;
 Formula* _t2; 
 
 public:
 Formula();
 Formula(OperatorType opr, TermList qparm, Formula *t1, Formula *t2);
 void print();
 void print_pl();
 void negation();
};


class Requirement {
 public:
 StringList _require;
 bool strips;
 bool adl;
 bool equality;
 bool typing;
 bool negative_preconditions;
 bool conditional_effects;
 bool derived_predicates;
 bool constraints;
};

class Structure {
  public:
  PropositionType n_type;
  string _name;
  TermList *_parameters;
  Formula *_preconditions;
  Formula *_effects;
};

typedef list<Structure> StructureList;

class pDomain {
  public:
  AtomList _predicates;
  TermList _constants;
  Requirement _require_def;
  TypeList  _types_def;
  StructureList _actions;
  StructureList _derived; 
};

class pProblem {
  public:
  string _pname;
  string _dname; 
  TermList _objects;
  Formula* _init;
  Formula* _goal;
};


#endif

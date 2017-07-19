/* Parser for PDDL language */
%{
#include "reader.h"

int yyerror(char *s);
int yylex(void);
int negation(Formula inf, Formula outf, Formula& negform);
int totaltreeleaves(Formula *node);
void negation_movein(Formula* f);
void addtree(Formula *f, Formula *node, int totalrank, int i);
void copy_formula(Formula *f, Formula *cp);
int ranktree(Formula *node, int rank, int pos);
int derivedname=0; 
string strupcase(const string& str );


extern Reader reader;
typedef map<string,string>::value_type value_type;
map<string, string> t_map;

%}

%union{
  string*	str_val;
  StringList*  str_list; 
  StringList2* str_list2;
  
  Term* term_;
  TermList* termlist_;
  Predicate* predicate_;
  Atom* atom_;
  AtomList* atomlist_;
  LiteralTerm* literal_;
  LiteralList* literallist_;
  Formula* formula_; 
  Type* type_;
  TypeList* typelist_;
  Requirement* requirement_;
  Structure* structure_;
  StructureList* structurelist_;
  pDomain* domain_;
  pProblem* problem_;
}

%start pdomain


%token dDOMAIN
%token DEFINE
%token REQUIREMENTS
%token <str_val> STRIPS
%token <str_val> ADL
%token <str_val> TYPING
%token <str_val> NEGATIVE_PRECONDITIONS
%token <str_val> DISJUNCTIVE_PRECONDITIONS
%token <str_val> EQUALITY
%token <str_val> EXISTENTIAL_PRECONDITIONS
%token <str_val> UNIVERSAL_PRECONDITIONS
%token <str_val> QUANTIFIED_PRECONDITIONS
%token <str_val> CONDITIONAL_EFFECTS
%token <str_val> DERIVED_PREDICATES

%token TYPES
%token CONSTANTS
%token PREFERENCE
%token PREDICATES
%token <str_val> CONSTRAINTS
%token STRUCTURE

//Problem
%token PROBLEM
%token INIT
%token GOAL
%token OBJECT
%token pDOMAIN

//action
%token ACTION
%token DERIVED
%token PRECONDITION
%token EFFECT
%token PARAMETERS

// constraints 
%token <str_val> WHEN
%token <str_val> AND
%token <str_val> OR
%token <str_val> NOT
%token <str_val> IMPLY
%token <str_val> EXISTS
%token <str_val> FORALL
%token <str_val> ATEND
%token <str_val> ALWAYS
%token <str_val> SOMETIME
%token <str_val> WITHIN
%token <str_val> AT_MOST_ONCE
%token <str_val> SOMETIME_AFTER
%token <str_val> SOMETIME_BEFORE
%token <str_val> ALWAYS_WITHIN
%token <str_val> HOLD_DURING
%token <str_val> HOLD_AFTER
%token EITHER

//NPDDL
%token <str_val> UNKNOWN
%token <str_val> ONEOF

%token <str_val> NUMBER
%token <str_val> NAME
%token <str_val> VARIABLE
%token MINUS

%type <requirement_> 	require_def
%type <requirement_> 	require_key_plus
%type <requirement_> 	require_key


%type <atomlist_> 	predicates_def
%type <atomlist_> 	atomic_formula_skeleton_plus
%type <atom_> 		atomic_formula_skeleton
%type <termlist_> 	typed_list_variable
%type <typelist_> 	type
%type <typelist_> 	primitive_type
%type <typelist_> 	primitive_type_plus

%type <termlist_> 	constants_def
%type <typelist_> 	types_def
%type <typelist_> 	typed_list_name

%type <structurelist_> 	structure_def_star
%type <structure_> 	structure_def
%type <structure_> 	action_def
%type <structure_> 	derived_def

%type <structure_> 	action_def_body
%type <formula_> 	precondition_def
%type <formula_> 	effect_def
%type <formula_> 	effectlist
%type <formula_> 	c_effect
%type <formula_> 	c_effect_star_and
%type <formula_>	p_effect
%type <formula_>	p_effect_star_and
%type <formula_>	cond_effect

%type <formula_> 	pre_GD
%type <formula_> 	pre_GD_star_and
%type <formula_> 	pre_GD_star_or
%type <formula_>	pref_GD

%type <domain_> 	pdomain
%type <domain_> 	domain_option
//goal
%type <formula_> 	GD
%type <formula_> 	GD_star_or
%type <formula_> 	GD_star_and
%type <formula_>	GD_star_oneof
%type <literal_> 	literal
%type <atom_> 		atomic_formula_term
%type <predicate_> 	predicate
%type <term_> 		term
%type <termlist_> 	term_star

%type <str_val> 	name 
%type <str_val> 	number
%type <str_val> 	variable
%type <str_list> 	variable_star
%type <str_list> 	variable_plus

//problem
%type <problem_> 	pproblem
%type <problem_> 	problem_option 
%type <termlist_> 	object_dec
%type <literal_> 	init_el
%type <literallist_> 	init_el_star
%type <formula_> 	init
%type <formula_>	init_list
%type <termlist_> 	typed_list_object
%type <formula_> 	goal

%token EQUAL
%token LEFT_PAREN
%token RIGHT_PAREN
%token SEMICOLON

%%

pdomain:
LEFT_PAREN DEFINE LEFT_PAREN dDOMAIN name RIGHT_PAREN domain_option RIGHT_PAREN pproblem
{
 reader.m_domain_name = *$5;
 reader.m_domain = *$7;
 reader.m_problem = *$9;
}
;


domain_option:
{
  $$ = new pDomain;
}
|
domain_option require_def
{
  $$ = $1;
  $$->_require_def = *$2;

}
|
domain_option types_def
{
  $$ = $1;
  $$->_types_def = *$2;
}
|
domain_option constants_def
{
  //needed to check 
  $$ = $1;
  $$->_constants = *$2;
}
|
domain_option predicates_def 
{
  $$ = $1;
  $$->_predicates = *$2;
}
|
domain_option derived_def
{
  $$ = $1;
  $$->_derived.push_back(*$2);
}
|
domain_option action_def
{
  $$ = $1;
  $$->_actions.push_back(*$2);
}
;

pproblem:
LEFT_PAREN DEFINE LEFT_PAREN PROBLEM name RIGHT_PAREN LEFT_PAREN pDOMAIN name RIGHT_PAREN problem_option init goal RIGHT_PAREN
{
  $$ = $11;
  $$->_pname = *$5;
  $$->_dname = *$9;
  $$->_init = $12;
  $$->_goal = $13;
  
}
|
LEFT_PAREN DEFINE LEFT_PAREN PROBLEM name RIGHT_PAREN LEFT_PAREN pDOMAIN name RIGHT_PAREN init goal RIGHT_PAREN
{
  $$ = new pProblem;
  $$->_pname = *$5;
  $$->_dname = *$9;
  $$->_init = $11;
  $$->_goal = $12;
  
}
;

problem_option:
require_def
{
  $$ = new pProblem;
}
|
object_dec 
{
  $$ = new pProblem;
  $$->_objects = *$1;
}
|
require_def object_dec 
{
  $$ = new pProblem;
  $$->_objects = *$2;
}
;

object_dec:
LEFT_PAREN OBJECT typed_list_object RIGHT_PAREN
{
  $$ = $3;
}
;

init:
LEFT_PAREN INIT init_list RIGHT_PAREN
{
  $$ = $3;
}
;

goal:
LEFT_PAREN GOAL pre_GD RIGHT_PAREN
{
  $$ = $3;
}
;

init_list:
LEFT_PAREN AND pre_GD_star_and RIGHT_PAREN
{
  $$ = $3;
}
|
LEFT_PAREN OR pre_GD_star_or RIGHT_PAREN
{
  $$ = $3;
}
;

init_el_star:
//to delete
init_el
{
  $$ = new LiteralList;
  $$->push_back(*$1);
}
|
init_el_star init_el
{
  $$ = $1;
  $$->push_back(*$2);
}
;

init_el:
//to delete
literal
{
  $$ = $1;
}
;

structure_def:
action_def
{
  $$ = $1;
}
|
derived_def
{
  $$ = $1;
}
;

//not in use
structure_def_star:
{
  $$ = new StructureList;
}
|
structure_def_star structure_def
{
  $$ = $1;
  $$->push_back(*$2);
}
;

action_def:
LEFT_PAREN ACTION name PARAMETERS LEFT_PAREN typed_list_variable RIGHT_PAREN action_def_body RIGHT_PAREN
{
  $$ = $8;
  $$->n_type = DYNAMIC;
  $$->_name = *$3;
  $$->_parameters = $6;	

}
|
LEFT_PAREN ACTION name action_def_body RIGHT_PAREN
{
  $$ = $4;
  $$->n_type = DYNAMIC;
  $$->_name = *$3;
  $$->_parameters = new TermList;

}
;

action_def_body:
{
  $$ = new Structure;
}
|
precondition_def
{
  $$ = new Structure;
  $$->_preconditions = $1;
  $$->_effects = 0;
}
|
effect_def
{
  $$ = new Structure;
  $$->_preconditions = 0;
  $$->_effects = $1;
}
|
precondition_def effect_def
{
  $$ = new Structure;

  $$->_preconditions = $1;
  $$->_effects = $2;

}
;

precondition_def:
PRECONDITION LEFT_PAREN RIGHT_PAREN
{
  //empty precondition body
  $$ = new Formula;
}
|
PRECONDITION pre_GD 
{
  $$ = $2; 
}
;


effect_def:
EFFECT LEFT_PAREN RIGHT_PAREN
{
  //empty effect body
  $$ = new Formula;
}
|
EFFECT effectlist 
{
  $$ = $2;
  //$2->print($2);
}
;

effectlist:
c_effect
{
  $$ = $1;
}
|
LEFT_PAREN AND c_effect_star_and RIGHT_PAREN
{
  $$ = $3;
}
;

c_effect_star_and:
{
  $$ = 0;
}
|
c_effect_star_and c_effect
{
  $$ = new Formula;
  $$->_operator = AND_;
  $$->_t1 = $1;
  $$->_t2 = $2;
}
;

c_effect:
LEFT_PAREN FORALL LEFT_PAREN typed_list_variable RIGHT_PAREN effectlist RIGHT_PAREN
{
  $$ = new Formula;
  $$->_operator = QFORALL_;
  $$->_quantified_parameters = *$4;
  $$->_t1 = $6;
  $$->_t2 = 0;
}
|
LEFT_PAREN WHEN GD cond_effect RIGHT_PAREN
{
 $$ = new Formula; 
 $$->_operator = WHEN_;
 $$->_t1 = $3;
 $$->_t2 = $4;
}
|
p_effect
{
  $$ = $1;
}
;

p_effect:
atomic_formula_term
{
  LiteralTerm* lt;
  lt = new LiteralTerm;
  lt->_atom = *$1;
  lt->_neg = false;

  $$ = new Formula;
  $$->_operator = UNIOP_;
  $$->_term = *lt;
}
| 
LEFT_PAREN NOT atomic_formula_term RIGHT_PAREN
{
 LiteralTerm* lt;
  lt = new LiteralTerm;
  lt->_atom = *$3;
  lt->_neg = true;

  $$ = new Formula;
  $$->_operator = UNIOP_;
  $$->_term = *lt;

}
;

p_effect_star_and:
{
  $$ = 0;
}
|
p_effect_star_and p_effect 
{
  $$ = new Formula;
  $$->_operator = AND_;
  $$->_t1 = $1;
  $$->_t2 = $2;
}
;

cond_effect:
p_effect
{
 $$ = $1;
}
|
LEFT_PAREN AND p_effect p_effect_star_and RIGHT_PAREN
{
  $$ = new Formula;
  $$->_operator = AND_;
  $$->_t1 = $3;
  $$->_t2 = $4;
}
;

derived_def:
LEFT_PAREN DERIVED typed_list_variable literal GD RIGHT_PAREN
{
  LiteralList ll;
  LiteralList2 ll2; 
  Formula *f;

  $$ = new Structure;
  f = new Formula;

  f->_operator = UNIOP_;
  f->_term = *$4;

  $$->n_type = STATIC;
  $$->_name = string("derived"+derivedname++);
  $$->_parameters = $3;
  $$->_preconditions = f;
  $$->_effects = $5;
}

;

pref_GD:
LEFT_PAREN PREFERENCE name GD RIGHT_PAREN
{
 //will be added
}
|
GD
{
  $$ = $1;
}
;

pre_GD:
pref_GD
/* preference goal */
{
  $$ = $1;
}
|
LEFT_PAREN AND pre_GD_star_and RIGHT_PAREN
{
  $$ = $3;
}
|
LEFT_PAREN FORALL LEFT_PAREN typed_list_variable RIGHT_PAREN pre_GD RIGHT_PAREN
{
  $$ = new Formula;
  $$->_operator = QFORALL_;
  $$->_quantified_parameters = *$4;
  $$->_t1 = $6;
  $$->_t2 = 0;
}
;

pre_GD_star_and:
{
  $$ = 0;
}
|
pre_GD_star_and pre_GD
{
  $$ = new Formula;
  $$->_operator = AND_;
  $$->_t1 = $1;
  $$->_t2 = $2;
}
;

pre_GD_star_or:
{
  $$ = 0;
}
|
pre_GD_star_or pre_GD
{
  $$ = new Formula;
  $$->_operator = OR_;
  $$->_t1 = $1;
  $$->_t2 = $2;
}
;


predicates_def:
LEFT_PAREN PREDICATES atomic_formula_skeleton_plus RIGHT_PAREN
{
  $$ = $3;
}
;

atomic_formula_skeleton_plus:
atomic_formula_skeleton
{
  $$ = new AtomList;
  $$->push_back(*$1);
}
|
atomic_formula_skeleton_plus atomic_formula_skeleton
{
  $$ = $1;
  $$->push_back(*$2);
}
;

atomic_formula_skeleton:
LEFT_PAREN predicate typed_list_variable RIGHT_PAREN
{
  
  $$ = new Atom;
  $$->_predicate = $2;
  $$->_term = *$3;
}
;

typed_list_variable:
{
  $$ = new TermList;
}
|
variable MINUS type typed_list_variable 
{
  map<string, string>::const_iterator p;

  TermList::iterator it;
  Term aterm;

  $$ = $4;
  aterm._name = *$1;
  aterm._ground = *$1;
  aterm._type = *$3;
  
  p = t_map.find($3->front()._name);
  if (p != t_map.end()) {
     //aterm._type->front()._name = p->second;
	
     //keep the original type
     aterm._type.front()._name = p->first;
  }
  else {
   cout << "\nERROR: undeclared type " <<$3->front()._name <<endl;
   return false;
  }
  
  aterm.constant = false;
  $$->push_front(aterm);
}
|
variable typed_list_variable
{
  Term aterm, pterm;

  $$ = $2;
  aterm._name = *$1;
  aterm._ground = *$1;
  aterm.constant = true;
  if (!$2->empty()) {
   pterm = $2->front();
   if (!pterm.constant)
   { 
    aterm._type = pterm._type;
    aterm.constant = false;
   }
  }
  $$->push_front(aterm);
}
;

variable_plus:
variable
{
  $$ = new StringList;
  $$->insert(*$1);
}
|
variable_plus variable
{
  $$ = $1;
  $$->insert(*$2);
}
;

variable_star:
{
  $$ = new StringList;
}
|
variable_star variable
{
  $$ = $1;
  $$->insert(*$2);
}
;

type:
// disjunction primitive type 
LEFT_PAREN EITHER primitive_type_plus RIGHT_PAREN
{
  $$ = $3;
}
|
primitive_type
{
  $$ = $1;
}
;

primitive_type_plus:
name
{
  $$ = new TypeList;
  Type nt;
  nt._name = *$1;
  $$->push_back(nt);
}
|
primitive_type_plus name
{
  Type nt;
  $$ = $1;
  nt._name = *$2;
  $$->push_back(nt);
}
;

primitive_type:
name
{
  $$ = new TypeList;
  Type nt;
  nt._name = *$1;
  $$->push_back(nt);
}
;


constants_def:
LEFT_PAREN CONSTANTS typed_list_object RIGHT_PAREN
{
  $$ = $3;
}
;

types_def:
LEFT_PAREN TYPES typed_list_name RIGHT_PAREN
{
  TypeList::iterator it;
  $$ = $3;
  
  for (it = $3->begin(); it != $3->end(); it++) {
   //handle object type
   if (strupcase(it->_type) == "OBJECT")
     t_map.insert(value_type(it->_name, it->_name));
   else
     t_map.insert(value_type(it->_name, it->_type));
  }
}
;

typed_list_name:
{
  $$ = new TypeList;
}
|
name MINUS name typed_list_name
{
  Type nt;
  
  $$ = $4;
  nt._name = *$1;
  nt._type = *$3;
  if (strupcase(*$3) == "OBJECT") //main type
    nt._type = "OBJECT";
  nt._typed = true;
  
  $$->push_front(nt); 
}
|
name typed_list_name
{
  Type nt, pt;
 
  $$ = $2;
  nt._name = *$1;
  nt._type = *$1;
  nt._typed = false;
  if (!$2->empty()) {
   pt = $2->front();
   if (pt._typed) {
     nt._type = pt._type;
     nt._typed = true;
   }
  }
  $$->push_front(nt);
}
;

typed_list_object:
{
  $$ = new TermList;
}
|
name MINUS type typed_list_object 
{
  Term aterm;

  $$ = $4;
  aterm._name = *$1;
  aterm._type = *$3;
  aterm.constant = false;
  $$->push_front(aterm);
  
}
|
name typed_list_object
{
  Term aterm, pterm;

  $$ = $2;
  aterm._name = *$1;
  aterm.constant = true;
  if (!$2->empty()) {
   pterm = $2->front();
   if (!pterm.constant)
   { 
    aterm._type = pterm._type;
    aterm.constant = false;
   }
  }
  $$->push_front(aterm);
}
;

require_def:
LEFT_PAREN REQUIREMENTS require_key_plus RIGHT_PAREN
{
  $$ = $3; 
}
;

require_key_plus:
require_key
{
  $$ = $1;
}
|
require_key_plus require_key
{
  StringList::iterator it;

  if ($1->_require.empty())
    $$ = new Requirement;
  else 
    $$ = $1;
  for (it=$2->_require.begin();it!=$2->_require.end(); it++)
    $$->_require.insert(*it);
  
  //assign flags
  $$->adl 	= ($$->adl)||($2->adl);
  $$->strips 	= ($$->strips)||($2->strips);
  $$->equality 	= ($$->equality)||($2->equality);
  $$->typing 	= ($$->typing)||($2->typing);
  $$->negative_preconditions = ($$->negative_preconditions)||($2->negative_preconditions);
  $$->conditional_effects = ($$->conditional_effects)||($2->conditional_effects);
  $$->derived_predicates = ($$->derived_predicates)||($2->derived_predicates);
  $$->constraints = ($$->constraints)||($2->constraints);
  $$->disjunctive_preconditions = ($$->disjunctive_preconditions)||($2->disjunctive_preconditions);
}
;

require_key:
STRIPS
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->strips = true;
}
|
ADL
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->adl = true;
}
|
EQUALITY
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->equality = true;
}
|
TYPING
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->typing = true;
}
|
DISJUNCTIVE_PRECONDITIONS
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->disjunctive_preconditions = true;
}
|
NEGATIVE_PRECONDITIONS
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->negative_preconditions = true;
}
|
CONDITIONAL_EFFECTS
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->conditional_effects = true;

}
|
DERIVED_PREDICATES
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->derived_predicates = true;

}
|
CONSTRAINTS
{
  $$ = new Requirement;
  $$->_require.insert(*$1);
  $$->constraints = true;
  cout << "System has not support constraints"<<endl;
}
;

GD:
atomic_formula_term
{
  LiteralTerm* lt;
  lt = new LiteralTerm;
  lt->_atom = *$1;
  lt->_neg = false;

  $$ = new Formula;
  $$->_operator = UNIOP_;
  $$->_term = *lt;
}
|
literal
{ 
  $$ = new Formula;
  $$->_operator = UNIOP_;
  $$->_term = *$1;
}
|
LEFT_PAREN AND GD_star_and RIGHT_PAREN
{
  $$ = $3;
}
|
LEFT_PAREN OR GD_star_or RIGHT_PAREN
{
  $$ = $3;
}
|
LEFT_PAREN NOT GD RIGHT_PAREN
{
  $$ = new Formula;
  $$->_operator = NOT_;
  $$->_t1 = $3;
  $$->_t2 = 0;
}
|
LEFT_PAREN IMPLY GD GD RIGHT_PAREN
{ //create node ~t1 v t2
  Formula* nott;
  nott = new Formula;
  nott->_operator = NOT_;
  nott->_t1 = $3;
  nott->_t2 = 0;

  $$ = new Formula;
  $$->_operator = OR_;
  $$->_t1 = nott;
  $$->_t2 = $4;
}
|
LEFT_PAREN FORALL LEFT_PAREN typed_list_variable RIGHT_PAREN GD RIGHT_PAREN
{
  $$ = new Formula;
  $$->_operator = QFORALL_;
  $$->_quantified_parameters = *$4;
  $$->_t1 = $6;
  $$->_t2 = 0;
}
|
LEFT_PAREN EXISTS LEFT_PAREN typed_list_variable RIGHT_PAREN GD RIGHT_PAREN
{
  $$ = new Formula;
  $$->_operator = QEXISTS_;
  $$->_quantified_parameters = *$4;
  $$->_t1 = $6;
  $$->_t2 = 0;
}
|
/*
LEFT_PAREN UNKNOWN atomic_formula_term RIGHT_PAREN
//NPDDL Unknown predicate
{
  
  LiteralTerm* lt;
  lt = new LiteralTerm;
  lt->_atom = *$3;
  lt->_neg = false;

  $$ = new Formula;
  $$->_operator = UNKNOWN_;
  $$->_term = *lt;
}
*/
LEFT_PAREN UNKNOWN GD_star_or RIGHT_PAREN
//Consider Unknown predicate as Oneof
{
  Formula *tmp, *node, *rnode;

  node = new Formula;
  rnode = new Formula;

  copy_formula($3, node);

  rnode->_operator = NOT_;
  rnode->_t1=node;
  rnode->_t2=0;
  negation_movein(rnode);

  tmp = new Formula;
  tmp->_operator = OR_;
  tmp->_t1 = $3;
  tmp->_t2 = rnode;

  $$ = new Formula;
  $$->_operator = ONEOF_;
  $$->_t1 = tmp;
  $$->_t2 = 0;
}

|
LEFT_PAREN ONEOF GD_star_or RIGHT_PAREN
//NPPDL Oneof predicate 
{ 

  $$ = new Formula;
  $$->_operator = ONEOF_;
  $$->_t1 = $3;
  $$->_t2 = 0;
/*
  //To robust parsing ONEOF laterly, NOT_ and OR node are construct 
  Formula *nNode, *f;
  int nl; 

  nNode = new Formula;
  nNode->_operator = NOT_;
  nNode->_t1 = $3;
  nNode->_t2 = 0;

  f = new Formula;
  f->_operator = ONEOF_;
  f->_t1 = new Formula;

  negation_movein(nNode);
  nl = totaltreeleaves(nNode);
  addtree(f->_t1, nNode, nl, nl);
  $$ = f->_t1;
  $$->print();
*/
}
;


GD_star_or:
{ 
  $$ = 0;
}
|
GD_star_or GD
{
  $$ = new Formula;
  $$->_operator = OR_;
  $$->_t1 = $1;
  $$->_t2 = $2;
}
;


GD_star_and:
{
  $$ = 0;
}
|
GD_star_and GD
{
  $$ = new Formula;
  $$->_operator = AND_;
  $$->_t1 = $1;
  $$->_t2 = $2;
}
;

GD_star_oneof:
//To be deleted soon
{
  $$ = 0;
}
|
GD_star_and GD
{
  $$ = new Formula;
  $$->_operator = ONEOF_;
  $$->_t1 = $1;
  $$->_t2 = $2;
}
;

literal:
atomic_formula_term
{
  $$ = new LiteralTerm;
  $$->_atom = *$1;
  $$->_neg = false;
}
|
LEFT_PAREN NOT atomic_formula_term RIGHT_PAREN
{
  $$ = new LiteralTerm;
  $$->_atom = *$3;
  $$->_neg = true; 
}
;

atomic_formula_term:
LEFT_PAREN predicate term_star RIGHT_PAREN
{
  $$ = new Atom;
  $$->_predicate = $2;
  $$->_term = *$3;

}
;

predicate:
name
{
  $$ = new Predicate;
  $$->_name = *$1;
}
|
EQUAL
{
  $$ = new Predicate;
  $$->_name = "equal";
}
;

term_star:
{
  $$ = new TermList;
}
|
term_star term
{
  $$ = $1;
  $$->push_back(*$2);
}
;

term:
name
{
  $$ = new Term;
  $$->_name = *$1;
  $$->_ground = *$1; //and 04/05/06
  $$->constant = true; 
  $$->variable = false;
}
|
variable
{
  $$ = new Term;
  $$->_name = *$1;
  $$->_ground = *$1; //and 04/05/06
  $$->variable = true; 
  $$->constant = false;
}
;

name: 
NAME {
  $$ = $1;
}
;


number:
NUMBER {
  $$ = $1;
}
;

variable:
VARIABLE {
  $$ = $1;
}
;

%%

int yyerror(string s)
{
  extern int yylineno;	// defined and maintained in lex.c
  extern char *yytext;	// defined and maintained in lex.c
  
  cerr << "ERROR: " << s << " at symbol \"" << yytext;
  cerr << "\" on line " << yylineno << endl;
  exit(1);
  return 0;  
}

int yyerror(char *s)
{
  return yyerror(string(s));
}


int totaltreeleaves(Formula *node) 
{
  int nleaves = 0;

  switch (node->_operator) {
    case AND_:
      if (node->_t1 != 0) nleaves = nleaves + totaltreeleaves(node->_t1);
      if (node->_t2 != 0) nleaves = nleaves + totaltreeleaves(node->_t2);
      break;
    case UNIOP_:
      nleaves++;
      break;
    default:
      printf("Error in oneof %d syntax! \n",node->_operator);
      break;
  }
  return (nleaves);
}

int ranktree(Formula *node, int totalrank, int pos) 
{
  static int rank = 0;
  switch (node->_operator) {
    case ONEOF_:
    case AND_:
      if (node->_t1 != 0) ranktree(node->_t1,totalrank,pos);
      if (node->_t2 != 0) ranktree(node->_t2,totalrank,pos);
      break;
    case UNIOP_:
      rank++;
      if (rank == pos)
         node->_term._neg = false; 
      if (rank == totalrank) rank = 0;
      break;
    default:
      printf("Error in oneof %d syntax! \n",node->_operator);
      break;
  }
  return rank;
}

void negation_movein(Formula* f)
{ 
  switch (f->_operator) {
   case AND_:
   case OR_:
   case IMPLY_:
   case QFORALL_:
   case QEXISTS_: 
   case UNIOP_:
   case WHEN_:
   case ONEOF_:
	if (f->_t1 != 0) negation_movein(f->_t1);
	if (f->_t2 != 0) negation_movein(f->_t2);
        break;
   case NOT_: 
	//shift tree up one level as negating the NOT
        if (f->_t1 != 0 ) {
	f->_operator = f->_t1->_operator;
	f->_term = f->_t1->_term;
	f->_quantified_parameters = f->_t1->_quantified_parameters;
	f->_t2 = f->_t1->_t2;
	f->_t1 = f->_t1->_t1;	
	f->negation();
        }
	break;
   default: 
        break;
  }
}

void addtree(Formula *f, Formula *node, int totalrank, int i)
{
   Formula *cpnode;

     cpnode = new Formula;
     copy_formula(node, cpnode);
     ranktree(cpnode,totalrank,i);

     f->_operator = OR_;
     f->_t1 = cpnode;
     f->_t2 = new Formula;
     if (--i>0) 
       addtree(f->_t2,node, totalrank, i);
}

void copy_formula(Formula *f, Formula *cp)
//cp is a copied formula of the pointer formula f
{
    cp->_operator = f->_operator;
    cp->_quantified_parameters = f->_quantified_parameters;
    cp->_term = f->_term;
    if (f->_t1 != 0) { 
      cp->_t1 = new Formula;
      copy_formula (f->_t1, cp->_t1);
    }
    if (f->_t2 != 0) { 
      cp->_t2 = new Formula;
      copy_formula (f->_t2, cp->_t2);
    }
}

string strupcase(const string& str )
{
   string tmp=str;

   for(int i = 0; i < str.length(); i++)
   {
      tmp[i] = toupper(str[i]);
   }
   return tmp;
}  


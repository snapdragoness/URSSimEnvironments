// planner.cpp: implementation of the planner class. 
//////////////////////////////////////////////////////////////////////
#include "planner.h" 
#include "define.h"


#define SOLUTION_TAG       0
#define REQUEST_TAG        1
#define ACCEPT_TAG         2
#define BEGINDATA_TAG      3
#define DATA_TAG           4
#define ENDOFDATA_TAG      5

/*************************************************************************
 * Planner Implementation
 *************************************************************************/
// Constructor & destructor 
Planner::Planner(Reader* reader, Timer* timer)
{
  m_timer = timer;
  m_reader = reader;

  m_semantics = PC;
  m_task = SEARCH;
  m_algorithm = GREEDY;

  // add the empty state to the state table
  State* s = new State(this);
  add_state(s);  
}

Planner::~Planner()
{
 
}

bool Planner::main()
{
  switch (m_semantics) {
  case PH:
    cout << "The approximation used is Possibly-Holds." << endl;
    break;
  case PC:
    cout << "The approximation used is Possibly-Changes." << endl;
    break;
  default:
    return false;
  }
  cout << "Converting pddl to pl\n";
  convert_pddl_to_pl();

  //grounding the domain
  cout << "Grounding & building domain...";
  m_timer->start(BUILD_TIMER);


  //grounding();
  // build the domain
  //build();
  m_timer->end(BUILD_TIMER);
  cout << "done." << endl;

  //print_domain();
  // print domain
#ifdef PRINT_DOMAIN
  print_domain();
  cout << endl;
#endif

#ifdef PRINT_SUMMARY
  cout << endl;
  print_summary();
#endif
  // search for a plan or execute a sequence of actions
  switch (m_task) {
  case SEARCH:
    m_timer->start(SEARCH_TIMER);
    search();
    m_timer->end(SEARCH_TIMER);
    break;
  case DOPLAN:
    execute();
    break;
  default:
    return false;
  }
  cout << endl;
  //print_statistics(); 

  return true;
};

void Planner::negation_movein(Formula* f)
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

void Planner::formula_flattening(Formula* f)
{
  TermList::iterator iterm;
  TermList::const_iterator iobj;
  Formula* qformula;
  OperatorType opr;

  switch (f->_operator) {
   case QFORALL_:
   case QEXISTS_: 
	qformula = new Formula; 
	if (f->_operator == QFORALL_) 
	  opr = AND_;
	else opr = OR_;
  	copy_formula(f->_t1, qformula);
	for (iterm = f->_quantified_parameters.begin();iterm != f->_quantified_parameters.end(); iterm++) {
	  for (iobj = m_reader->m_problem._objects.begin(); iobj != m_reader->m_problem._objects.end(); iobj++)
	    //match term with object
	    if ((!m_typing)||((m_typing)&&(iterm->compare_type(&iobj->_type))))
		flattening(f, qformula, *iterm, *iobj, opr);
	  if (f->_t1 != 0) formula_flattening(f->_t1);
	  if (f->_t2 != 0) formula_flattening(f->_t2);
	}
	break;
   default: 
	if (f->_t1 != 0) formula_flattening(f->_t1);	
	if (f->_t2 != 0) formula_flattening(f->_t2);
        break;
  }

}

void Planner::dequantified_condition(Formula* f)
//flattening conditional effect
{
 Formula *pwhen, *qt1, *qt2;
 switch (f->_operator) {
  case QFORALL_:
  case QEXISTS_:
	if (f->_t1->_operator == WHEN_) {
  
	 pwhen = f->_t1;
	 qt1 = new Formula(f->_operator, f->_quantified_parameters, pwhen->_t1, 0);
	 qt2 = new Formula(f->_operator, f->_quantified_parameters, pwhen->_t2, 0);
	 //shift when up one level 
	 f->_operator = WHEN_;
	 f->_t1 = qt1;
	 f->_t2 = qt2;	 
	 break;
	}	
  default :
	if (f->_t1 != 0) dequantified_condition(f->_t1);
	if (f->_t2 != 0) dequantified_condition(f->_t2);
	break; 
 }
}

void Planner::copy_formula(Formula *f, Formula *cp)
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

void Planner::flattening(Formula* f, Formula* qformula, const Term qterm, const Term qobj, const OperatorType opr)
//flattening all quantified terms in a formula
{
  Formula* cpform;
  cpform = new Formula;

  qterm_grounding(qformula, qterm, qobj);
  copy_formula(qformula, cpform);
  
  switch (f->_operator) {
    case QFORALL_:
    case QEXISTS_:
	 f->_operator = opr;	 
	 f->_t1 = cpform;
	 f->_t2 = 0;
	 break;
    case OR_:
    case AND_:
	 if (f->_t2 != 0)
	    flattening(f->_t2, qformula, qterm, qobj, opr);
	 else {
         f->_t2 = new Formula;
         f->_t2->_operator = opr;
	 f->_t2->_t1 = cpform;
	 f->_t2->_t2 = 0;
	 }
    	 break;
    default :
	 break;
  }
}

void Planner::qterm_grounding(Formula *f, const Term qterm, const Term qobj)
//grounding quantified terms
{
  TermList:: iterator iterm;

  switch(f->_operator) {
  case  UNIOP_:
	for (iterm = f->_term._atom._term.begin(); iterm != f->_term._atom._term.end(); iterm++)
	 if ((iterm->_name == qterm._name)) {
	 iterm->_ground = qobj._name;
	 iterm->constant = true; 
	}	
  default: 
 	if (f->_t1 != 0) qterm_grounding(f->_t1, qterm, qobj);
 	if (f->_t2 != 0) qterm_grounding(f->_t2, qterm, qobj);
        break;

  }
}

bool Planner::grounding()
{
  StringList predicates; 
  StructureList actions;
  StringList *pinit, *p;
  TermList::const_iterator ait;

  pinit = new StringList;
  p = new StringList;

  if (( m_reader->m_domain._require_def.typing)||(m_reader->m_domain._require_def.adl))
    m_typing = true;
  else
    m_typing = false;

  //attatch constants list to objects list for grounding
  for(ait = m_reader->m_domain._constants.begin(); ait != m_reader->m_domain._constants.end(); ait++) 
  {
      m_reader->m_problem._objects.push_front(*ait);
  }

  //grounding predicate and derived preditaces and actions
  m_reader->m_fluents = predicate_grounding(pinit);
  derived_grounding();
  action_grounding();
  //build init and goal
  m_reader->m_goal = build_goal();
  m_reader->m_init = build_init();

  return true;
}

StringList Planner::predicate_grounding(StringList *pinit)
{ 
  AtomList::const_iterator ait;
  TermList::const_iterator it;
  TermList::const_iterator oit;
 
  StringList::const_iterator sit;
  StringList pred,temp,fluents;
  StringList negpred, negtemp, negfluents;

  //grounding predicates 
  fluents.clear();
  for (ait = m_reader->m_domain._predicates.begin(); ait != m_reader->m_domain._predicates.end(); ait++) {
    pred.clear();
    pred.insert(ait->_predicate->_name);
    negpred.clear();
    negpred.insert(string("-"+ait->_predicate->_name));
    //for each term type 
    for (it = ait->_term.begin(); it != ait->_term.end(); it++) {
       //for each object type that match
       temp.clear();
	negtemp.clear();
       for (oit = m_reader->m_problem._objects.begin(); oit != m_reader->m_problem._objects.end(); oit++) 
       if ((!m_typing)||((m_typing)&&(it->compare_type(&oit->_type))))
       {
          for (sit = pred.begin(); sit != pred.end(); sit++) 
             temp.insert(string(*sit + "_" + oit->_name));
	  for (sit = negpred.begin(); sit != negpred.end(); sit++) 
             negtemp.insert(string(*sit + "_" + oit->_name));
       }
       pred = temp;
	negpred = negtemp;
    }
    for (sit = pred.begin(); sit != pred.end(); sit++)
         //insert grounding predicate to fluent list
         fluents.insert(*sit); 
    for (sit = negpred.begin(); sit != negpred.end(); sit++) 
         //insert grounding predicate to fluent list
         negfluents.insert(*sit); 
  }
  *pinit = negfluents;
  return fluents;
}


void Planner::action_grounding() 
{
  StructureList::iterator strl;
  
  for (strl = m_reader->m_domain._actions.begin();strl != m_reader->m_domain._actions.end();strl++)
  {
    //preconditions
    //cout << strl->_name << endl;
    if (strl->_preconditions != 0) {
      negation_movein(strl->_preconditions);
      formula_flattening(strl->_preconditions);
    }
    //effects
    if (strl->_effects != 0) {
      negation_movein(strl->_effects);
      formula_flattening(strl->_effects);
    }
    //strl->_effects->print(); cout << endl;
    //grounding
    parameter_grounding(*strl, 1);
  }
}

void Planner::derived_grounding()
{
  StructureList::iterator strl;
  
  for (strl = m_reader->m_domain._derived.begin();strl != m_reader->m_domain._derived.end();strl++)
  {
    //precondition
    negation_movein(strl->_preconditions);
    formula_flattening(strl->_preconditions);
    //effects
    negation_movein(strl->_effects);
    formula_flattening(strl->_effects);
    parameter_grounding(*strl, 1);
  }
}

void Planner::parameter_grounding(Structure &inf, unsigned int index)
{
  TermList::iterator pterm, ip;
  TermList::const_iterator iobj;
  typedef map<string,string>::value_type value_type;
  map<string,string> v_map;
  unsigned int i;
 
  i = 0;
  for (pterm = inf._parameters->begin(); pterm != inf._parameters->end(); pterm++) {
  i++;
  if (i==index)
    for (iobj = m_reader->m_problem._objects.begin(); iobj != m_reader->m_problem._objects.end(); iobj++) 
    //match term with object
    if ((!m_typing)||((m_typing)&&(pterm->compare_type(&iobj->_type)))) {
     //assign object to variable 
     pterm->_ground = iobj->_name;
     if (i < inf._parameters->size())
        parameter_grounding(inf,index+1);
     else {
     v_map.clear();
     for (ip = inf._parameters->begin(); ip != inf._parameters->end(); ip++) 
       v_map.insert(value_type(ip->_name, ip->_ground));

     //grounding preconditions and effects
     if (inf._preconditions != 0) 
     {
     if (formula_grounding(inf._preconditions, v_map))
      if (formula_grounding(inf._effects, v_map))
        add_proposition(inf);
     }
     else {
      if (formula_grounding(inf._effects, v_map))
         add_proposition(inf);
     }
    }
    }
}
}

bool Planner::formula_grounding(Formula *f, const map<string,string> v_map) 
{
  switch (f->_operator) {
  case  UNIOP_:
	if (!term_grounding(f,v_map))
	 return false;
  default:
	if (f->_t1 != 0) formula_grounding(f->_t1, v_map);
	if (f->_t2 != 0) formula_grounding(f->_t2, v_map);
	break;
 }
 return true;
}

bool Planner::term_grounding(Formula *f, const map<string,string> v_map)
{
  LiteralTerm lterm;
  Term t1, t2;
  TermList:: iterator iterm;
  map<string, string>::const_iterator p;
  switch(f->_operator) {
  case  UNIOP_:
 	for (iterm = f->_term._atom._term.begin(); iterm != f->_term._atom._term.end(); iterm++)
 	 if ((!iterm->constant)) {
	 p = v_map.find(iterm->_name);
	 if (p != v_map.end())
	    iterm->_ground = p->second;
	 else {
	   cout << "\nERROR: undeclared variable " <<iterm->_name <<endl;
	   return false;
	 }
	}
	if (f->_term._atom._predicate->_name == "=") {
	 lterm = f->_term;
	 t1 = lterm._atom._term.front();
	 lterm._atom._term.pop_front();
	 t2 = lterm._atom._term.front();
	 if (!f->_term._neg) {
	    return (t1._ground == t2._ground);
	  }
	  else if (f->_term._neg) {
 	    return (t1._ground != t2._ground);
	  }
	}
  default: 
        break;
  }
  return true;
}

/************************************************************************
Convert PDDL to PL syntax

Date: Jan 7 2007

*************************************************************************/


string Planner::strupcase(const string& str )
{
   string strp = prefix+str;
   string tmp=strp;

   for(int i = 0; i < tmp.length(); i++)
   {
      tmp[i] = toupper(strp[i]);
   }
   return tmp;
}  


string Planner::strlowercase(const string& str )
{
   string tmp=str;

   for(int i = 0; i < str.length(); i++)
   {
      tmp[i] = tolower(str[i]);
      if (tmp[i] =='-') tmp[i] ='_';
   }
   if ((tmp[0]=='n')&&(tmp[1]=='e')&&(tmp[2]=='g'))
      return tmp;
   return string(prefix+tmp);

}

string Planner::strvar(const string& str )
{
   string tmp=str;

   for(int i = 0; i < str.length(); i++)
   {
      tmp[i] = toupper(str[i]);
      if (tmp[i] =='-') tmp[i] ='_';
      if (tmp[i] =='?') tmp[i] =' ';
   }
   return tmp;
}  

void Planner::print_goal(Formula* _goal)
{
  string str;

  _goal->print_goal(str);
  str = "plan_goal("+str+").\n";
  plcout << str;

}

void Planner::print_init(Formula* _init)
{
  string str;

  _init->print_init(str);
  str = string(prefix)+"initially("+str+").\n";
  plcout << str;

  //if (completedinfo) 


  //if (completedinfo) 
plcout << "unknown(X):- fluent(X),\n";
plcout << "            findall(L, (cpa_initially(cpa_oneof(Y)), member(L,Y)), LUnk),\n";
plcout << "            member(X, LUnk).\n";
plcout << "unknown(X):- fluent(X), cpa_unknown(X).\n";
plcout << "unknown(X) :- fluent(X), (cpa_initially(cpa_or(Y)),in_or(Y,X);\n";
plcout << "          cpa_initially(cpa_or(Y,Z)), (in_or(Y,X);in_or(Z,X))), !.\n";
plcout << "in_or(X,X) :- !.\n";
plcout << "in_or(neg(X),X) :- !.\n";
plcout << "in_or(cpa_or(Y),X) :- in_or(Y,X).\n";
plcout << "in_or(cpa_or(Y,Z),X) :- (in_or(Y,X);in_or(Z,X)).\n";
plcout << "cpa_unknown(nop).\n";

}

void Planner::print_actions_eff(const StructureList _actions)
{
  StructureList::const_iterator isl;
  TermList::const_iterator itl;
  TypeList::const_iterator ipl;
  Structure  _caction;
  StructureList  _cactionlist; 
  StructureList::const_iterator il;  
  string str;

  for (isl = _actions.begin(); isl != _actions.end(); ++isl)
  {
  
 
    //_caction._name = isl->_name;
    //_caction._parameters = isl->_parameters;
    //_cactionlist.clear();
    //print_actions_conds(isl->_effects, _caction, _cactionlist);
    
    //if (_cactionlist.empty()) 
    {
    plcout << "causes("<<strlowercase(isl->_name);
    if (!isl->_parameters->empty())
    {
    plcout <<"(";
    //print parameters list
    for(itl = isl->_parameters->begin();itl != isl->_parameters->end(); ++itl) 
     {
      if (itl != isl->_parameters->begin())
	plcout <<",";
      plcout <<" "<<strvar(itl->_name);
     }
     plcout<<")"; 
    } //endif
    plcout <<", ["<<endl;
    
    //print preconditions
    str.clear();
    isl->_effects->print_pl(str);
    plcout << str;
    plcout <<" ], \n[";

    plcout <<"])";
    
    //print domains of variables
    if (!isl->_parameters->empty()) {
    plcout <<":-"<<endl<<"\t";
    for(itl = isl->_parameters->begin();itl != isl->_parameters->end(); ++itl) 
    { 
      if (!itl->_type.empty())  {

       if (itl != isl->_parameters->begin())
	     plcout <<", ";

       for(ipl = itl->_type.begin(); ipl != itl->_type.end(); ++ipl) 
         plcout << strlowercase(ipl->_name) <<"("<<strvar(itl->_name)<<")"; 
      }
      else {
        //plcout <<"Error: variable"<<strlowercase(itl->_name)<<" has no type"<<endl;  
       if (itl != isl->_parameters->begin())
	     plcout <<", ";

	plcout<<strlowercase("object")<<"("<<strvar(itl->_name)<<")"; 

      }
     }//endfor
     } //endif
     plcout <<"."<<endl<<endl;
   }//endif
   }//endfor
}

void Planner::print_actions_pre(const StructureList _actions)
{
  StructureList::const_iterator isl;
  TermList::const_iterator itl;
  TypeList::const_iterator ipl;
  string str;

  for (isl = _actions.begin(); isl != _actions.end(); ++isl)
  {

    plcout << "executable("<<strlowercase(isl->_name);
    if (!isl->_parameters->empty())
    {
     plcout <<"(";
      //print parameters list
    for(itl = isl->_parameters->begin();itl != isl->_parameters->end(); ++itl) 
     {
      if (itl != isl->_parameters->begin())
	plcout <<",";
      plcout <<" "<<strvar(itl->_name);
     }
     plcout<<")"; 
    } //endif
    plcout <<", ["<<endl;
    str.clear();
    
    //print preconditions
    isl->_preconditions->print_pl(str);
    plcout << str;
    plcout <<" ])";
    //print domains of variables
    if (!isl->_parameters->empty()) {
    plcout <<":-"<<endl<<"\t";
    for(itl = isl->_parameters->begin();itl != isl->_parameters->end(); ++itl) 
    { 
      if (!itl->_type.empty())  {

       if (itl != isl->_parameters->begin())
	     plcout <<", ";

       for(ipl = itl->_type.begin(); ipl != itl->_type.end(); ++ipl) 
         plcout << strlowercase(ipl->_name) <<"("<<strvar(itl->_name)<<")"; 
      }
      else {
        //plcout <<"Error: variable"<<strlowercase(itl->_name)<<" has no type"<<endl;  
       if (itl != isl->_parameters->begin())
	     plcout <<", ";

	plcout<<strlowercase("object")<<"("<<strvar(itl->_name)<<")"; 

      }
     }//endfor
     } //endif
     plcout <<"."<<endl<<endl;
   }//endfor
}

void Planner::print_actions_def(const StructureList _actions)
{
  StructureList::const_iterator isl;
  TermList::const_iterator itl;
  TypeList::const_iterator ipl;
  string str;

  for (isl = _actions.begin();isl != _actions.end(); ++isl)
  {
    plcout << "action("<<strlowercase(isl->_name);
    if (!isl->_parameters->empty())
    {
     plcout << "(";
      //print parameters list
    for(itl = isl->_parameters->begin();itl != isl->_parameters->end(); ++itl) 
     {
      if (itl != isl->_parameters->begin())
	plcout <<",";
      plcout <<" "<<strvar(itl->_name);
     }
     plcout<<")"; 
    }
    plcout <<")";
    
    
    //print domains of variables
    if (!isl->_parameters->empty()) {
    plcout <<":-"<<endl<<"\t";
    for(itl = isl->_parameters->begin();itl != isl->_parameters->end(); ++itl) 
    { 
      if (!itl->_type.empty())  {

       if (itl != isl->_parameters->begin())
	     plcout <<", ";

       for(ipl = itl->_type.begin(); ipl != itl->_type.end(); ++ipl) 
         plcout << strlowercase(ipl->_name) <<"("<<strvar(itl->_name)<<")"; 
      }
      else {
       if (itl != isl->_parameters->begin())
	     plcout <<", ";

        //plcout <<"Error: variable"<<strlowercase(itl->_name)<<" has no type"<<endl;  
	plcout<<strlowercase("object")<<"("<<strvar(itl->_name)<<")"; 

      }
     }//endfor
    }//endif
    plcout <<"."<<endl<<endl;
  }
}


void Planner::print_predicates(const AtomList *_atoms)
{//prolog syntax
  AtomList::const_iterator iat;
  TermList::const_iterator itl;
  TypeList::const_iterator ipl;

  for(iat = _atoms->begin();iat != _atoms->end(); ++iat)
  {

    //print predicate name
    plcout <<"fluent("<<strlowercase(iat->_predicate->_name);
    if (!iat->_term.empty()) 
    {
     plcout<<"(";

     //print list of variables
     for(itl = iat->_term.begin();itl != iat->_term.end(); ++itl) 
     {
      if (itl != iat->_term.begin())
	plcout <<",";
      plcout <<" "<<strvar(itl->_name);
     }
     plcout<<")"; 
    }
    plcout <<")";
    
    
    //print domains of variables
    if (!iat->_term.empty()) {
    plcout <<":-"<<endl<<"\t";
    for(itl = iat->_term.begin();itl != iat->_term.end(); ++itl) 
    { 
      if (!itl->_type.empty())  {

       if (itl != iat->_term.begin())
	     plcout <<", ";

       for(ipl = itl->_type.begin(); ipl != itl->_type.end(); ++ipl) 
         plcout << strlowercase(ipl->_name) <<"("<<strvar(itl->_name)<<")"; 
      }
      else {
       if (itl != iat->_term.begin())
	     plcout <<", ";

        //plcout <<"Error: variable "<<strlowercase(itl->_name)<<" has no type"<<endl;  
	plcout<<strlowercase("object")<<"("<<strvar(itl->_name)<<")"; 

      }
     }
     plcout <<"."<<endl<<endl;
    }
   else
    plcout <<"."<<endl<<endl;
  }
  
}

void Planner::print_types_rules(const TypeList *_types)
{
  TypeList::const_iterator itl;

  for(itl = _types->begin();itl != _types->end(); ++itl)
   if ((itl->_type != "OBJECT")&&(itl->_type != itl->_name))
   {
     plcout << strlowercase(itl->_type) <<"(X):- "<<strlowercase(itl->_name)<<"(X)."<<endl;
   }
}

void Planner::print_consts(const TermList *_consts)
{
 TermList::const_iterator itl;
  TypeList::const_iterator ipl;
  
  for(itl = _consts->begin();itl != _consts->end(); ++itl)
  {
    if (!itl->_type.empty())  
     for(ipl = itl->_type.begin(); ipl != itl->_type.end();  ++ ipl) {
      plcout << strlowercase(ipl->_name) <<"("<<strlowercase(itl->_name)<<")."<<endl;
      plcout <<strlowercase("object")<<"("<<strlowercase(itl->_name)<<")."<<endl;  
     }
    else
      plcout <<strlowercase(itl->_name)<<"."<<endl;  
  }
}

void Planner::print_objects(const TermList *_objs)
{ //consider cases objects has more than 1 types or 0 type ?
  
  TermList::const_iterator itl;
  TypeList::const_iterator ipl;
  
  for(itl = _objs->begin();itl != _objs->end(); ++itl)
  {
    if (!itl->_type.empty())  
     for(ipl = itl->_type.begin(); ipl != itl->_type.end();  ++ ipl)  {
      plcout << strlowercase(ipl->_name) <<"("<<strlowercase(itl->_name)<<")."<<endl;
      plcout <<strlowercase("object")<<"("<<strlowercase(itl->_name)<<")."<<endl;  
     }
    else
      plcout <<strlowercase("object")<<"("<<strlowercase(itl->_name)<<")."<<endl;  
  }
}


void Planner::print_types(const TypeList *_types)
{
  TypeList::const_iterator itl;

  for(itl = _types->begin();itl != _types->end(); ++itl)
  {
     plcout << strupcase(itl->_name) <<endl;
  }
}

void Planner::convert_pddl_to_pl()
{
  fstream file_in;
  char s[256];

 // char* pl = new char [m_reader->m_problem._pname.size()+4];
 // strcpy(pl,string(strlowercase(m_reader->m_problem._pname+".pl")).c_str());
  
  plcout.open("pddl2pl.pl");
  
  cout<<"Output file: pddl2pl.pl";
  cout <<endl<<endl;
  
  plcout<<"\n:- use_module(library(lists)).";
  plcout<<"\n:- dynamic executable/2.";
  plcout<<"\n:- dynamic cpa_executable/2.";
  plcout<<"\n:- dynamic causes/3.";
  plcout<<"\n:- dynamic cpa_causes/3.";
  plcout<<endl;

  //  SP_pred_ref SP_predicate("member",2,"member")

  //cout<<endl<<"%%%%  Parsing domain file %%%%"<<endl;
  //cout<<"Domain "<<m_reader->m_domain_name<<endl;

  //cout<<endl<<"%%%%  Parsing problem file %%%%"<<endl;
  //cout<<"Problem "<<m_reader->m_problem._pname<<endl;

  //cout<<endl<<"%%%%  Types %%%%"<<endl;
  //if (m_reader->m_domain._require_def.typing)  
  //  print_types(&(m_reader->m_domain._types_def));


  plcout<<endl<<"%%%% Objects %%%%"<<endl;
  print_objects(&(m_reader->m_problem._objects));

  plcout<<endl<<"%%%% Constants %%%%"<<endl;
  print_consts(&(m_reader->m_domain._constants));

  plcout<<endl<<"%%%%  Types rules %%%%"<<endl;
  if (m_reader->m_domain._require_def.typing)  
    print_types_rules(&(m_reader->m_domain._types_def));

  plcout<<endl<<"%%%% Predicates %%%%"<<endl;
  print_predicates(&(m_reader->m_domain._predicates));

  plcout<<endl<<"%%%% Actions %%%%"<<endl;
  print_actions_def(m_reader->m_domain._actions);

  plcout<<endl<<"%%%% Preconditions %%%%"<<endl;
  print_actions_pre(m_reader->m_domain._actions);

  plcout<<endl<<"%%%% Effects %%%%"<<endl;
  print_actions_eff(m_reader->m_domain._actions);
  
  plcout<<endl<<"%%%% Inits %%%%"<<endl;
  print_init(m_reader->m_problem._init);

  plcout<<endl<<"%%%% Goals %%%%"<<endl;
  print_goal(m_reader->m_problem._goal);
  
  plcout.close();

}


bool Planner::build()
{
  StringList2::const_iterator it;
  State* s;
  StringList x;
  int i;
  Action* p_act;
  PropositionList::const_iterator it_prop;
  StaticLaw stat;
  Literals hd, bd;
  Literals::iterator il;
  Literals::iterator il2;
  string f; // fluent
  bool inconsist;
  StringList::const_iterator itf;
  typedef map<string,Literal>::value_type value_type;
 
  // build fluent literals;
  i = 0;
  for (itf = m_reader->m_fluents.begin(); 
       itf != m_reader->m_fluents.end(); itf++) { 
    m_map.insert(value_type(*itf,i));
    m_literals.insert(i++);
    m_map.insert(value_type(NEGATION_SYMBOL+*itf,i));
    m_literals.insert(i++);
  }

  // build action list
  for (it_prop = m_reader->m_propositions.begin(); 
       it_prop != m_reader->m_propositions.end(); it_prop++) {    
    hd = ground((*it_prop).get_effect());
    //check for consistency
    inconsist = false;
    for (il = hd.begin(); il != hd.end(); il++) 
     for (il2 = hd.begin(); il2 != hd.end(); il2++)  
       if (convert(*il) == NEGATION_SYMBOL +convert(*il2)) 
          inconsist = true;

    bd = ground((*it_prop).get_precondition());
    if (!inconsist) 
    switch ((*it_prop).n_type) {
    case STATIC:
      stat.set_head(hd);
      stat.set_body(bd);
      m_statics.push_back(stat);
      break;
    case DYNAMIC:
      p_act = add_action((*it_prop).act_name);
      p_act->add_effect(hd,bd);
      break;
    case EXECUTABILITY:
      p_act = add_action((*it_prop).act_name);
      p_act->add_exec(bd);
      break;
    case IMPOSSIBILITY:
      p_act = add_action((*it_prop).act_name);
      p_act->add_imposs(bd);
      break;
    default:
      break;
    }
  }
  
  // build initial c-state
  for (it = m_reader->m_init.begin(); it != m_reader->m_init.end(); it++) {
    s = new State(this);
    s->m_literals = ground(&(*it));
    s->closure();
    s = add_state(s);
    m_init_cstate.m_states.insert(s);
  }

  if (m_init_cstate.m_states.empty())
    m_init_cstate.m_states.insert(add_state(new State(this)));

  // build the goal
  m_goal = ground(&m_reader->m_goal);

  // stupid code
  State t(this);
  t.m_literals = m_goal;
  t.closure();
  m_cgoal = t.m_literals;

  return true;
}

// grounding functions
Literals Planner::ground(const StringList* x) const
{
  StringList::iterator it;
  Literals y;

  for (it = x->begin(); it != x->end(); it++) {
    y.insert(ground(*it));
  }

  return y;
}

Literal Planner::ground(const string& x) const
{
  map<string,Literal>::const_iterator p = m_map.find(x);

  if (p != m_map.end()) {
    return (p->second);
  }

  cout << "ERROR: Literal " << x << " is undeclared." << endl;
  cout << "Check the fluent declaration." << endl << endl;
  
  exit(1);
}

StringList Planner::convert(const Literals& x) const
{
  StringList y;
  Literals::iterator it;

  for (it = x.begin(); it != x.end(); it++) {
    y.insert(convert(*it));
  }

  return y;
}

string Planner::convert(const Literal& x) const
{
  unsigned short int i = x / 2;
  StringList::iterator it;

  if (i >= m_reader->m_fluents.size())
    return NULL;

  it = m_reader->m_fluents.begin();

  for (i = 0; i < x/2; i++) {
    it++;
  }

  if (x % 2 == 0)
    return *it;
  
  return (NEGATION_SYMBOL + (*it));
}


/* printing functions */
void Planner::print(const Literal& l) const
{
  cout << convert(l);
}

void Planner::print(const Literals& x) const
{
  Literals::iterator it;
  bool comma = false;
  for (it = x.begin(); it != x.end(); it++) {
    if (comma) 
      cout << ",";
    print(*it);
    comma = true;
  }
}

void Planner::print_domain() const
{
  ActionList::const_iterator it;
  
  cout << "DOMAIN DESCRIPTION" << endl;
  cout << "----------------------------" << endl;
  for (it = m_actions.begin();it != m_actions.end(); it++) {
    it->print();
    cout << endl;    
  }

  StaticLaws::const_iterator its;
  
  cout << "Static Laws: " << endl;

  for (its = m_statics.begin(); its != m_statics.end(); ++its) {
    cout << "    ";
    print(*its->get_head());
    cout << " <- ";
    print(*its->get_body());
    cout << endl;	  
  }

  // print init cstate
  cout <<  "INIT" << endl;
  cout <<  "----------------------------" << endl;
  m_init_cstate.print();
  cout << endl;

  // print goal state
  cout <<  "GOAL " << endl;
  cout <<  "----------------------------" << endl;
  print(m_goal);
  cout << endl;  
}

void Planner::print_summary() const
{
  cout <<  "SUMMARY" << endl;
  cout <<  "---------------------" << endl;
  cout <<  "Number of fluents: " << m_literals.size()/2 << endl;
  cout <<  "Number of actions: " << m_actions.size() << endl;
  cout <<  "Number of static laws: " << m_statics.size() << endl;
}

void Planner::print_statistics() const
{
  double total = m_timer->time(READ_TIMER) + 
    m_timer->time(BUILD_TIMER) + m_timer->time(SEARCH_TIMER);

  cout << "STATISTICS" << endl;
  cout << "---------------------" << endl;
  printf("Total time: %.3f (sec) \n",total);
#ifdef PRINT_TIME_DETAILS
  printf("  Reading: %.3f (sec) [%.2f %%]\n", 
		 m_timer->time(READ_TIMER), 
		 100.0 * m_timer->time(READ_TIMER) / total);
  
  printf("  Preprocessing: %.3f (sec) [%.2f %%]\n", 
		 m_timer->time(BUILD_TIMER),
		 100.0 * m_timer->time(BUILD_TIMER)/total);
  printf("  Goal checking: %.3f (sec) [%.2f %%]\n", 
		 m_timer->time(GOAL_TIMER),
		 100.0 * m_timer->time(GOAL_TIMER)/total);
  printf("  Heuristic computation: %.3f (sec) [%.2f %%]\n", 
		 m_timer->time(HEURISTIC_TIMER),
		 100.0 * m_timer->time(HEURISTIC_TIMER) / total);
  printf("  Next-state computation (+ Executability Checking):\
%.3f (sec)[%.2f %%]\n", 
		 m_timer->time(RESULT_TIMER),
		 100.0 * m_timer->time(RESULT_TIMER)/total);

  printf("    Transition Lookup: %.3f (sec) [%.2f %%]\n",
		 m_timer->time(TRANSITION_LOOKUP_TIMER),
		 100.0 * m_timer->time(TRANSITION_LOOKUP_TIMER)/total);

  printf("    Executability Checking: %.3f (sec) [%.2f %%]\n",
		 m_timer->time(EXECUTABILITY_TIMER),
		 100.0 * m_timer->time(EXECUTABILITY_TIMER)/total);

  printf("    Actual Next-state Computation: %.3f (sec) [%.2f %%]\n",
		 m_timer->time(ACTUAL_RESULT_TIMER),
		 100.0 * m_timer->time(ACTUAL_RESULT_TIMER)/total);

  printf("      Direct Effect Computation: %.3f (sec) [%.2f %%]\n",
		 m_timer->time(DIRECT_EFFECT_TIMER),
		 100.0 * m_timer->time(DIRECT_EFFECT_TIMER)/total);

  printf("      Indirect Effect Computation: %.3f (sec) [%.2f %%]\n",
		 m_timer->time(INDIRECT_EFFECT_TIMER),
		 100.0 * m_timer->time(INDIRECT_EFFECT_TIMER)/total);


  printf("  Other: %.3f (sec) [%.2f %%]\n", 
		 m_timer->time(SEARCH_TIMER) - 
		 m_timer->time(RESULT_TIMER) - 
		 m_timer->time(HEURISTIC_TIMER) - 
		 m_timer->time(GOAL_TIMER),
		 100.0 * (m_timer->time(SEARCH_TIMER) - 
				  m_timer->time(RESULT_TIMER) - 
				  m_timer->time(HEURISTIC_TIMER) - 
				  m_timer->time(GOAL_TIMER))/total);
#endif

  printf("Total states allocated: %d\n", m_states.size());
  printf("Total cstate(s): %d\n", m_cstates.size());
  printf("Total cstate(s) remaining in the queue: %d\n", m_queue.size());  
}

bool Planner::execute()
{
  RawPlan::const_iterator it;
  int n_count = 0;
  CState *cs;
  const Action *act;
  map<string,int>::iterator it1;

  cs = &m_init_cstate;

  cout << endl << "STEP " << n_count++ << endl;
  cout << "CState: " << endl;  
  cs->print();
  if (cs->goal_satisfied())
    cout << " (GOAL)";
  
  cout << endl;

  for (it = m_plan.begin(); it != m_plan.end(); ++it) {
    it1 = m_action_map.find(*it);
    if (it1 == m_action_map.end()) {
      cout << endl << "ERROR: Could not find action " << *it 
		   << " in the source file." << endl;
      return false;
    }

    // execute the action   
    cout << "-> Action: " << *it << endl;
    act = &m_actions[it1->second];
    
    if (!act->is_executable(cs)) {
      cout << endl << "STOP: Action " << *it 
		   << " cannot be executed." << endl;
      return false;
    }

    cs = next_cstate(cs,act);
    
    // print the next state
    cout << endl << "STEP " << n_count++ << endl;
    cout << "CState:" << endl;
    cs->print();
    if (cs->goal_satisfied())
      cout << " (GOAL)";
    cout << endl;
  }
  
  return true;
}

Action* Planner::add_action(const string str)
{ 
  map<string,int>::iterator it;
  Action a(this,str);

  it = m_action_map.find(str);

  if (it == m_action_map.end()) {
    // create a new entry
    m_actions.push_back(a);
    m_action_map.insert(map<string,int>::value_type(str,m_actions.size()-1));
    return &(m_actions[m_actions.size()-1]);
  }

  return &(m_actions[it->second]);
}

State* Planner::add_state(State* s)
{
  pair<StateTable::iterator,bool> result;

  result = m_states.insert(s);
  
  if (!result.second) {
    // state exists
    delete s;
    return *result.first;
  }

  return s;
}

State* Planner::next_state_pc(State* s, const Action* act)
{
  Literals pc0,pc1;		// set of literals that possibly changes
  EffectList::const_iterator it;
  Literals::const_iterator it2;
  StaticLaws::const_iterator it3;
  unsigned int n1,n2;
  const Literals* x;
  const Literals *bd;
  const Literals *hd;
  State* ns = new State(this);
  Literals y;

  // compute direct effects & pc0
  for (it = act->get_effects()->begin(); 
       it != act->get_effects()->end(); ++it) {
    // if the body is a subset of
    // of the state then the head will be added to e
    bd = it->get_body();
    hd = it->get_head();
    if (s->includes(bd)) {
      ns->m_literals.insert(hd->begin(),hd->end());
    }
    // compute pc0: if the -body(r) \cap s = \emptyset then
    // add the head(r) to pc_0
    x = it->get_neg_body();
    if (!s->intersect(x)) {
      // add the head to pc_0
      pc0.insert(hd->begin(),hd->end());
    }
  }

  ns->closure();	

  do {
    n1 = ns->m_literals.size();

    pc1 = pc0;

    // compute set of literals that possibly changes, pc
    do {
      n2 = pc1.size();

      // for each static law
      for (it3 = m_statics.begin(); it3 != m_statics.end(); ++it3) {
		bd = it3->get_body();
		x = it3->get_neg_body();
		hd = it3->get_head();
		// if body \cap (pc \setminus \delta) != \emptyset and
		// \neg body \cap s1 = \emptyset then add the head to pc
		if (!ns->intersect(x)) {
		  for (it2 = bd->begin(); it2 != bd->end(); it2++) {
			if (pc1.find(*it2) != pc1.end() && 
				s->m_literals.find(*it2) == s->m_literals.end()) {
			  // add the head to pc
			  pc1.insert(hd->begin(),hd->end());
			  break;
			}
		  }
		}
      }
    } while (pc1.size() != n2);

    // compute the next state and store it in s1
    // s1 = closure(e(a,s) \cup (s \ -pc1))
    y = negate(pc1);
    set_difference(s->m_literals.begin(),s->m_literals.end(),
				   y.begin(),y.end(),
				   insert_iterator<Literals>(ns->m_literals,ns->m_literals.end()));
    ns->closure();

  } while (ns->m_literals.size() != n1);

  ns->closure();	

  return add_state(ns);  
}

State* Planner::next_state_ph(State* s, const Action* act)
{
  State mdc(this);			// things that possibly hold
  Literal l;
  EffectList::const_iterator it;
  const EffectList* p_effs = act->get_effects();
  const Literals *hd;
  const Literals *bd;
  Literals::iterator it1;
  const Literals *temp;
  State* ns;

  ns = new State(this);

  // compute direct effects & possible direct effects
  // for each conditional effect
  for (it = p_effs->begin(); it != p_effs->end(); ++it) {
    // compute direct effects: if the body is a subset of
    // of the current state then add the head to dc
    bd = it->get_body();
    hd = it->get_head();
    if (s->includes(bd)) {
      ns->m_literals.insert(hd->begin(),hd->end());
    }

    // compute mdc: if the -body(r) \cap s = \emptyset then
    // add the head(r) to mdc
    temp = it->get_neg_body();//negate(*bd);

    if (!s->intersect(temp)) {
      // add the head to mdc	
      mdc.m_literals.insert(hd->begin(),hd->end());
    }
  }

  for (it1 = m_literals.begin(); it1 != m_literals.end(); it1++) {
    NEGATE(*it1,l);
    if (!s->m_literals.count(l) && !ns->m_literals.count(l)) {
      mdc.m_literals.insert(*it1);
    }
  }

  mdc.closure();

  // next state = Cl(dc \cup {l | -l is not in mdc})
  for (it1 = m_literals.begin(); it1 != m_literals.end(); it1++) {
    NEGATE(*it1,l);
    if (!mdc.m_literals.count(l)) {
      ns->m_literals.insert(*it1);
    }
  }
  
  ns->closure();

  return add_state(ns);
}

CState* Planner::next_cstate(CState* csin, const Action* act)
{
  set<State*>::iterator it;
  CState* csout;

  csout = new CState;
  for (it = csin->m_states.begin(); it != csin->m_states.end(); it++) {
    if (m_semantics == PC)
      csout->m_states.insert(next_state_pc(*it,act));
    else
      csout->m_states.insert(next_state_ph(*it,act));
  }
  csout->set_action(act);
  csout->set_previous_cstate(csin);
  csout->set_plan_length(csin->get_plan_length()+1);
  csout->hvalue();

  return csout;
}

Literals Planner::negate(const Literals& x) const
{
  Literals::const_iterator it;
  Literals y; // output literals
  Literal l;
  
  for (it = x.begin(); it != x.end(); ++it) {
    NEGATE(*it,l);
    y.insert(l);
  }
  return y;
}

bool Planner::search()
{
  bool solution = false;

  m_queue.push(&m_init_cstate);
  m_cstates.insert(&m_init_cstate);

  while (!solution && !m_queue.empty()) {
    solution = explore_once();
  }

  if (!solution)
    cout << "No plan was found." << endl;

  return solution;
}

bool Planner::explore_once()
{
  CState *cs, *cs1;
  const Action *act;
  ActionList::const_iterator it;
  CStateTable::iterator it2;
  //static int n = 0;

  cs = m_queue.top();
  m_queue.pop();
  //  cout << n++ << ": pickup node with hvalue = " << cs->hvalue() << endl;
  //  cs->print();
  //  cout << endl;
  for (it = m_actions.begin(); it != m_actions.end(); it++) {
    act = &(*it);
    if (act->is_executable(cs)) {
      cs1 = next_cstate(cs,act);
      if (cs1->goal_satisfied()) {
		cout << "Found a plan of length " <<
		  cs1->get_plan_length() << ":" << endl;
		cs1->print_plan();
		return true;
      }
      else {
		it2 = m_cstates.find(cs1);
		if (it2 == m_cstates.end()) {
		  m_queue.push(cs1);
		  m_cstates.insert(cs1);
		}
		else {
		  /*		  if (cs1->get_plan_length() < (*it2)->get_plan_length()) {
			cout << "Updating shorter plan" << endl;
			(*it2)->set_previous_cstate(cs);
			(*it2)->set_action(cs1->get_action());
			(*it2)->set_plan_length(cs1->get_plan_length());
			}*/

		  delete cs1;
		}
      }
    }
  }
  return false;
}

void Planner::add_proposition(const Structure& p)
{

  Proposition *act;
  StringList empty;
  StringList2 prop;
  StringList2::iterator isl;
  TermList::iterator ip; 
  string act_param;
  
  for (ip = p._parameters->begin(); ip != p._parameters->end(); ip++) {
      act_param=act_param+string("_"+ip->_ground);
  }
 
  if (p.n_type == STATIC) {
  //add derived actions
  act = new Proposition;
  act->n_type = STATIC;
  act->act_name = p._name+act_param;  
  prop = convertf_to_dnf(p._preconditions);
  for (isl = prop.begin(); isl != prop.end(); isl++)
     act->precond = *isl;
  prop = convertf_to_dnf(p._effects);
  for (isl = prop.begin(); isl != prop.end(); isl++)
     act->effect = *isl;
  m_reader->m_propositions.push_back(*act);
  }

  else { 
  //add executability condition
  if (p._preconditions != 0) {
     prop = convertf_to_dnf(p._preconditions);
     for (isl = prop.begin(); isl != prop.end(); isl++) {
   	act =  new Proposition;
   	act->n_type = EXECUTABILITY;
   	act->act_name = p._name+act_param;
   	act->precond = *isl;
   	act->effect = empty;
   	m_reader->m_propositions.push_back(*act);
    }
  }
  } 
   //add dynamic action
   act = new Proposition;
   act->n_type = DYNAMIC;
   act->act_name = p._name+act_param;
   prop = convertf_to_dnf(p._effects);
   for (isl = prop.begin(); isl != prop.end(); isl++)
     act->effect = *isl;
   act->precond = empty;
   m_reader->m_propositions.push_back(*act); 

   //add conditional effect
   act = new Proposition;
   act->n_type = DYNAMIC;
   act->act_name = p._name+act_param;
   add_conditional(p._effects, act);
}

void Planner::add_conditional(Formula *f, Proposition *act)
{
 StringList2 ceffs, cpreds;
 StringList2::iterator ieffs;
 StringList2::iterator ipreds;
 StringList ::iterator is;
 switch (f->_operator) {
  case WHEN_:
	cpreds = convertf_to_dnf(f->_t1);
 	ceffs = convertf_to_dnf(f->_t2);
	for (ipreds = cpreds.begin(); ipreds != cpreds.end(); ipreds++) {
  	  act->precond = *ipreds;
  	  for (ieffs = ceffs.begin(); ieffs != ceffs.end(); ieffs++) {
     	     act->effect = *ieffs;
	     m_reader->m_propositions.push_back(*act); 
          }
	}
	break;
  default: 
	if (f->_t1 != 0) add_conditional(f->_t1, act);
	if (f->_t2 != 0) add_conditional(f->_t2, act);
	break; 
 }
}

void Planner::print_actions_conds(Formula* f, Structure& _act, StructureList& _actions_conds)
{
  Formula *cpreds, *ceffs;

  switch (f->_operator) {
  case WHEN_:
	cpreds = new Formula;
	ceffs = new Formula;
        copy_formula(f->_t1, cpreds);
        copy_formula(f->_t2, ceffs);

	_act._preconditions = cpreds;
	_act._effects = ceffs;
	_actions_conds.push_back(_act);
	break;
  default: 
	if (f->_t1 != 0) print_actions_conds(f->_t1, _act, _actions_conds);
	if (f->_t2 != 0) print_actions_conds(f->_t2, _act, _actions_conds);
	break; 
 }
}

StringList2 Planner::convertf_to_dnf(Formula* f)
{
//this function convert a formula into disjunctive normal form 
string fluent;
StringList lstr;
StringList2 lstr2, f1,f2;
TermList::iterator it;

 switch(f->_operator) {
  case AND_:
	if (f->_t1 != 0) f1 = convertf_to_dnf(f->_t1);
	if (f->_t2 != 0) f2 = convertf_to_dnf(f->_t2);
	lstr2 = dnf_conjunction (f2,f1);
	return lstr2;
	break;
  case OR_:
 	if (f->_t1 !=0) f1 = convertf_to_dnf(f->_t1);
        if (f->_t2 !=0) f2 = convertf_to_dnf(f->_t2);
	lstr2 = dnf_disjunction (f1,f2);
	return lstr2;
	break;
  case UNIOP_:
	if (f->_term._atom._predicate->_name != "=")
	{
	fluent = f->_term._atom._predicate->_name;
        if (!f->_term._atom._term.empty()) fluent = fluent + string("(");
	for (it = f->_term._atom._term.begin(); it != f->_term._atom._term.end(); it++) {
	  if (it !=f->_term._atom._term.begin())
	    fluent = fluent + string(", "+strlowercase(it->_ground));
 	  else
	    fluent = fluent + strlowercase(it->_ground);
        }
        if (!f->_term._atom._term.empty()) fluent = fluent + string(")");
        if (f->_term._neg) 
            fluent = string("neg(")+strlowercase(fluent)+")";
	lstr.insert(fluent);
	lstr2.insert(lstr);
	}
	return lstr2;

	break;
  case WHEN_:
	return lstr2;
	break;
  default:
	return lstr2;
	break;
 }
 return lstr2;	//add Dec 3, 07 debug
}

void Planner::print(StringList2 str2)
{
 StringList2::iterator isl;
 StringList::iterator il;
  for (isl = str2.begin(); isl != str2.end(); isl++) {
     for (il = isl->begin(); il != isl->end(); il++)
	 cout << *il <<",";
     cout << endl;
  }
}

StringList2 Planner::dnf_conjunction(StringList2 str1, StringList2 str2)
{
 StringList lstr1, lstr2;
 StringList2 llstr, temp2;
 StringList::iterator il1, il2;
 StringList2::iterator ill1, ill2;
 
 //make sure that str1 is not empty
 if (str1.empty()) {
   temp2 = str1;
   str1 = str2;
   str2 = temp2;
 }
     
 for (ill1 = str1.begin(); ill1 != str1.end(); ill1++) {
  lstr1.clear();
  for (il1 = ill1->begin(); il1 != ill1->end(); il1++) {
   lstr1.insert(*il1);
  }
  //str2
  if (!str2.empty()) {
  for (ill2 = str2.begin(); ill2 != str2.end(); ill2++) {
  lstr2 = lstr1;
   for (il2 = ill2->begin(); il2 != ill2->end(); il2++) {
    lstr2.insert(*il2);
   }
  //do the conjunction
  llstr.insert(lstr2);
  }
  }
  else 
    llstr.insert(lstr1);
 }
 return llstr;
}

StringList2 Planner::dnf_disjunction(StringList2 str1, StringList2 str2)
{
 StringList lstr1, lstr2;
 StringList2 llstr;
 StringList::iterator il1, il2;
 StringList2::iterator ill1, ill2;
  
  llstr = str1;
  //str2
  for (ill2 = str2.begin(); ill2 != str2.end(); ill2++)
    llstr.insert(*ill2);
  return llstr;
}

StringList Planner::build_goal()
{
 StringList goal;
 StringList2 prop;
 StringList2::iterator isl;

 //cout << "Support goals in form of (AND <GD>*)"<<endl;
 //assume no OR in the GOAL, pick out the first node
 prop = convertf_to_dnf(m_reader->m_problem._goal);
 for (isl = prop.begin(); isl != prop.end(); isl++)
     goal = *isl;
 return goal;

}

StringList2 Planner::build_init()
{
StringList2 init2;

 LiteralList::iterator lit;
 TermList::iterator tit;
 string subinit;
 StringList2 init;

 negation_movein(m_reader->m_problem._init);
 init = convertf_to_dnf(m_reader->m_problem._init);
 return init;

}

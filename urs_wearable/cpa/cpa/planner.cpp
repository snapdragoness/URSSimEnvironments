// planner.cpp: implementation of the planner class.
/////////////////////////////////////////////////////////////////////
#include "planner.h"
#include <string.h>

/*************************************************************************
 * Planner Implementation
 *************************************************************************/
// Constructor & destructor 
Planner::Planner(const Reader* reader, Timer* timer)
{
  m_timer = timer;
  m_reader = reader;

  m_semantics = PC;
  m_task = SEARCH;
  m_algorithm = GREEDY;
  m_detect_essential = true;
  m_output_decisive = false;
  rebuild_domain = false;
  // add the empty state to the state table
  State* s = new State(this);
  add_state(s);  
  cout << "0\n%%\n";
}

Planner::~Planner()
{
 
}

bool Planner::main()
{

#ifdef PRINT_DOMAIN
  m_reader->print();
  exit(0);
#endif

  // build the domain
  //cout << "Building domain...";
  m_timer->start(BUILD_TIMER);
  if(rebuild_domain == false)
	{
		build_domain();
  		build_goal();
		build_init();
		rebuild_domain = true;
	}
  else rebuild();
  m_timer->end(BUILD_TIMER);

  //cout << "done." << endl << endl;

  // print domain
#ifdef PRINT_INTERNAL_DOMAIN
  print_interal_domain();
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

  //cout << endl;
  //print_statistics(); 

  return true;
};

/*

  StringList x;
  unsigned int i;
  string f; // fluent
  StringList::const_iterator itf;
  typedef map<string,Literal>::value_type value_type;
*/

bool Planner::build_domain()
{
  int i;
  StringList::const_iterator itf;
  typedef map<string,Literal>::value_type value_type;
  PropositionList::const_iterator it_prop;
  Action* p_act;
  StaticLaw stat;
  Literals hd, bd;

  //add mar 31 2008 for hgraph
  Literals::const_iterator l_it;
  ActionList::iterator a_it;
  EffectList::const_iterator e_it;
  const EffectList* effs;
  const ExecList* execs;
  ExecList::const_iterator ex_it;
  ListPtrActions::const_iterator pa_it;
  Literals owliterals;

  
  // build fluent literals;
  i = 0;
  if(rebuild_domain == false)
  {
	for (itf = m_reader->m_fluents.begin(); 
	itf != m_reader->m_fluents.end(); itf++) { 
	m_map.insert(value_type(*itf,i));
	m_literals.insert(i++);
	m_map.insert(value_type(NEGATION_SYMBOL+*itf,i));
	m_literals.insert(i++);    
	}
  }
  // build action list
  for (it_prop = m_reader->m_propositions.begin(); 
       it_prop != m_reader->m_propositions.end(); it_prop++) {    
    hd = ground((*it_prop).get_effect());
    bd = ground((*it_prop).get_precondition());
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

  fa_links = new ListPtrActions[m_literals.size()];
   
  //build m_iaction list on m_action list
  for (a_it = m_actions.begin(); a_it != m_actions.end(); a_it++) {
    effs = a_it->get_effects();
    for (e_it = effs->begin();e_it != effs->end(); e_it++)
    {
	execs = a_it->get_execs();
	if (!execs->empty())
          for (ex_it = execs->begin(); ex_it != execs->end(); ex_it++) {
	    p_act = &(m_iactions[add_iaction(a_it->get_name())]);
	    p_act->add_effect(*e_it, *ex_it);	
         }
	else {
	  p_act = &(m_iactions[add_iaction(a_it->get_name())]);
	  p_act->add_effect(*e_it);
	}
    }
  }
 

  //create fluent-action links: true_links and fa_links
  for (a_it = m_iactions.begin(); a_it != m_iactions.end(); a_it++) {
    effs = a_it->get_effects();
    for (e_it = effs->begin();e_it != effs->end(); e_it++)
    {
          bd = *(e_it->get_body());
      if (bd.empty()) {
	true_links.push_back (&(*a_it));
      }
      else {
        for (l_it = bd.begin(); l_it != bd.end(); l_it++) {
         fa_links[*l_it].push_back(&(*a_it));
        }
      }
    }
  }
  //  cout << "Cardinality of actions' precond : "<< endl;
  int index = 0;
  m_iactions_bd = new int[m_iactions.size()];
  for (a_it = m_iactions.begin(); a_it != m_iactions.end(); a_it++, index++) {
    m_iactions_bd[index] = (((a_it->get_effects())->begin())->get_body())->size();
  }
 
  //initialize for hgraph
  m_fsatisfied = new int[m_iactions.size()];
  m_fexecuted = new int[m_literals.size()];
  n_actions = new int[m_literals.size()];

  return true;
}

bool Planner::build_goal()
{
  StringList2::const_iterator it;
  Literals st;

  // build the goal
  for (it = (m_reader->m_gd).begin(); it != (m_reader->m_gd).end(); it++) {
    st = ground(&(*it));   
    m_goal.insert(st.begin(),st.end());
    m_gd.insert(st);

  }
  return true;
}

bool Planner::build_init()
{
  bool *dep; // dependencies between literals
  bool *adep; // dependencies between actions and literals
  bool *gdep;
  bool first;
  unsigned int n,i,j,k;
  StringList2::const_iterator it;
  ActionList::const_iterator it_act;
  EffectList::const_iterator it_ef;
  const Literals *hd1, *bd1;
  Literals::const_iterator it1, it2;  
  Literals st, st_temp;
  Literals essentials;
  set<Literals> cs_tmp;
  set<Literals> cs, cs_new;
  set<Literals>::const_iterator its;
  set<Literals>::const_iterator itg;
  ExecList::const_iterator it_exec;
  Literal l1, l2;
  State *s;

  // build the set of initial partial states
  n = m_literals.size();

  if (m_detect_essential) {
    // compute dependencies
    //    m_timer->start(ESSENTIAL_FLUENTS_TIMER); 
    dep = new bool[n*n];

    for (i = 0; i < n; i++) {
      for (j = 0; j < n; j++) {
	if (i ==j)
	  dep[n*i+j] = true;
	else
	  dep[n*i+j] = false;
      }
    }

    for (it_act = m_actions.begin(); it_act != m_actions.end(); it_act++) {
      for (it_ef = (*it_act).get_effects()->begin(); 
	   it_ef != (*it_act).get_effects()->end(); ++it_ef) {
	hd1 = it_ef->get_head();
	bd1 = it_ef->get_body();
	for (it1 = hd1->begin(); it1 != hd1->end(); it1++) {
	  for (it2 = bd1->begin(); it2 != bd1->end(); it2++) {
	    dep[n*(*it1)+(*it2)] = true;
	    NEGATE(*it1,l1);
	    NEGATE(*it2,l2);	  
	    dep[n*l1+l2] = true;
	  }
	}
      }
    } 

    for (k = 0; k < n; k++) {
      for (i = 0; i < n; i++) {
	for (j = 0; j < n; j++) {
	  if (!dep[n*i+j])
	    dep[n*i+j] = dep[n*i+k] && dep[n*k+j];
	}
      }
    }

    // compute dependencies for actions
    adep = new bool[m_actions.size()*n];

    // initialize array
    for (i = 0; i < m_actions.size(); i++) {
      for (k = 0; k < n; k++) {
	adep[i*n+k] = false;
      }
    }

    for (i = 0; i < m_actions.size(); i++) {
      for (it_exec = m_actions[i].get_execs()->begin();
	   it_exec != m_actions[i].get_execs()->end(); it_exec++) {
	for (it1 = it_exec->begin(); it1 != it_exec->end(); it1++) {
	  // action depends on it1
	  for (k = 0; k < n; k++) {
	    if (dep[n*(*it1)+k]) {
	      // literal it1 depends on k --> action depends on k 
	      adep[i*n+k] = true;
	    }
	  }
	}
      }
    }

    // goal dependencies
    gdep = new bool[m_gd.size()*n];
    for (i = 0; i < m_gd.size(); i++) {
      for (k = 0; k < n; k++) {
	gdep[i*n+k] = false;
      }
    }
    i = 0;
    for (itg = m_gd.begin(); itg != m_gd.end(); itg++) {
      for (it1 = itg->begin(); it1 != itg->end(); it1++) {
	for (k = 0; k < n; k++) {
	  // ith subgoal depends on k if one of literals, it1, depends on k
	  gdep[i*n+k] |= dep[(*it1)*n+k];
	}
      }
      i++;
    }
    
  //m_timer->end(ESSENTIAL_FLUENTS_TIMER);
  }

  //build initial c-state
  cout << endl;  

  for (it = m_reader->m_init.begin(); it != m_reader->m_init.end(); it++) {
    st = ground(&(*it));
	
    s = new State(this);
    s->m_literals = st;
    if (!s->is_consistent()) {
      delete s;
      continue;
    }

    if (m_detect_essential) {
      essentials.clear();
      for (i = 0; i < m_gd.size(); i++) {
	// test i-th subgoal
	for (k = 0; k < n/2; k++) {
	  if (gdep[i*n+2*k] &&
	      gdep[i*n+2*k+1] &&
	      st.find(2*k) == st.end() && st.find(2*k+1) == st.end()) {
	    essentials.insert(k);
	  }
	}
      }	
    

      for (i = 0; i < m_actions.size(); i++) {
	for (k = 0; k < n/2; k++) {
	  if (adep[i*n+2*k] &&
	      adep[i*n+2*k+1] &&
	      st.find(2*k) == st.end() && st.find(2*k+1) == st.end()) {
	    essentials.insert(k);
	  }
	}
      }
      if (m_output_decisive) {
	cout << "\tState:{";
	print(st);
	cout << "} \tDecisive Set: {";
	first = true;
	for (it1 = essentials.begin(); it1 != essentials.end(); it1++) {
	  if (first)
	    first = false;
	  else {
	    cout << ", ";
	  }
	  print(2*(*it1));
	}
	
	cout << "}" << endl;
      }
    }
    else {
      essentials.clear();
      for (i = 0; i < n/2; i++) {
	if (st.find(2*i) == st.end() && st.find(2*i+1) == st.end()) {
	  essentials.insert(i);
	}
      }
    }

    
    cs.clear();
    cs.insert(st);
    cs_new.clear();

    for (it1 = essentials.begin(); it1 != essentials.end(); it1++) {
      for (its = cs.begin(); its != cs.end(); its++) {	
	st_temp = *its;
	st_temp.insert(2*(*it1));
	cs_new.insert(st_temp);
	st_temp = *its;
	st_temp.insert(2*(*it1)+1);
	cs_new.insert(st_temp);
      }
      cs = cs_new;
      cs_new.clear();
    }

    for (its = cs.begin(); its != cs.end(); its++) {
      s = new State(this);
      s->m_literals = *its;
      s->closure();
      s = add_state(s);
      m_init_cstate.m_states.insert(s);
    }
  }
  //set hlevel to 0
  m_init_cstate.hlevel = 0;

  if (m_detect_essential) {
    delete [] dep;
    delete [] adep;
    delete [] gdep;
  }

  if (m_init_cstate.m_states.empty()) {
    cout << "ERROR: There is no possible initial state. Check your domain description." << endl;
    exit(1);
    return false;
  }

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

/* printing functions */
void Planner::pos_print(const Literal& l) const
{
  string s = convert(l);
  if (s[0] != '-') 
    cout << s;
}

void Planner::pos_print(const Literals& x) const
{
  Literals::iterator it;
  bool comma = false;
  for (it = x.begin(); it != x.end(); it++) {
    if (comma) 
      cout << ",";
    pos_print(*it);
    comma = true;
  }
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

void Planner::print_interal_domain() const
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

  // print statistics
  StringList2::const_iterator itst;
  unsigned int i = 0;

  cout <<  "STATISTICS " << endl;
  cout <<  "----------------------------" << endl;
  cout << "Total actions: " << m_actions.size() << endl;
  cout << "Total fluents: " << m_literals.size() / 2 << endl;
  cout << "Unknown fluents: " << endl;
  for (itst = m_reader->m_init.begin(); 
       itst != m_reader->m_init.end(); itst++) {
    cout << "\tState " << i++ << ": ";
    cout << m_literals.size()/2 - (*itst).size();
    cout << endl;
  }
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
  int i;
  map<string,int>::const_iterator it;
  cout << my_action_map.size() << " ";
  for(i=0;i<my_action_map.size();i++)
  	for(it = my_action_map.begin() ;it != my_action_map.end() ;it++)
  	{
  		if(it->second == i) cout << it->first << " ";
  	}
  list<int>::const_iterator it1;
  cout << "\n%%\nlinear " << my_plan.size() << " ";
  for(it1 = my_plan.begin() ;it1 != my_plan.end() ;it1++)
  {
  	cout << *it1 << " ";
  }
  cout << "\n";
  cout << "STATISTICS" << endl;
  cout << "---------------------" << endl;
  printf("Total search time: %.3f (sec) \n",total);
#ifdef PRINT_TIME_DETAILS
  printf("  Reading: %.3f (sec) [%.2f %%]\n", 
	 m_timer->time(READ_TIMER), 
	 100.0 * m_timer->time(READ_TIMER) / total);
  printf("  Preprocessing: %.3f (sec) [%.2f %%]\n", 
	 m_timer->time(BUILD_TIMER),
	 100.0 * m_timer->time(BUILD_TIMER)/total);
  printf("  Search: %.3f (sec) [%.2f %%]\n", 
	 m_timer->time(SEARCH_TIMER),
	 100.0 * m_timer->time(SEARCH_TIMER)/total);
#endif

  //printf("Total states allocated: %d\n", m_states.size());
  //printf("Total cstate(s): %d\n", m_cstates.size());
  //printf("Total cstate(s) remaining in the queue: %d\n", m_queue.size());  
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
  cout << "CState m_states.size()" << cs->m_states.size()<<endl;  
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
    cout << "CState m_states.size()" << cs->m_states.size()<<endl;  
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

int Planner::add_iaction(const string str)
{
  map<string, int>::iterator it;
  Action a(this, str);

   //create new iaction
   a.set_index(m_iactions.size());
   m_iactions.push_back(a);
   m_iaction_map.insert(map<string,int>::value_type(str,m_iactions.size()-1));
   return (m_iactions.size()-1);


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
		   insert_iterator<Literals>(ns->m_literals,
					     ns->m_literals.end()));
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

  //add
  csout->hlevel = csin->hlevel + 1;

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
  if (m_init_cstate.goal_satisfied()) {
    //cout << "Found a plan of length " <<
      //m_init_cstate.get_plan_length() << ":" << endl;
      //m_init_cstate.print_plan();
    return true;
  }

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

  cs = m_queue.top();
  m_queue.pop();
 

  for (it = m_actions.begin(); it != m_actions.end(); it++) {
    
    act = &(*it);
    if (act->is_executable(cs)) {
      cs1 = next_cstate(cs,act);
      if (cs1->goal_satisfied()) {
	//cout << "Found a plan of length " <<
        //cs1->get_plan_length() << ":" << endl;
	//cs1->print_plan();
        printplan(cs1);
	set<State*>::iterator cit;  
  
	
	m_init_cstate = *cs1;
	m_init_cstate.set_previous_cstate(NULL);
	m_init_cstate.set_plan_length(0);
	m_init_cstate.set_action(NULL);
	for (cit = m_init_cstate.m_states.begin(); cit != m_init_cstate.m_states.end(); cit++){
     		(*cit)->gclear();
   	}    
	return true;
      }
      else {
	it2 = m_cstates.find(cs1);
	if (it2 == m_cstates.end()) {
	  m_queue.push(cs1);
	  m_cstates.insert(cs1);
      
	}

	else {
	  delete cs1;
	}
      }
    }
  }
  
  return false;
}

int Planner::build_hgraph_relaxed(Literals m_literals)
{
ListPtrActions::const_iterator a_it;
Literals facts, next_facts, new_facts;
unsigned short level= 0;
Literals::const_iterator l_it;
Literals::const_iterator hd_it;

const Literals *hd;
int pos;


set<Literals>::const_iterator it;

memset(n_actions, 0, sizeof(int)*Planner::m_literals.size());
memset(m_fexecuted,0, sizeof(int)*Planner::m_literals.size());
memset(m_fsatisfied,0,sizeof(int)*Planner::m_iactions.size());

facts = m_literals;
new_facts = facts;

do {
next_facts.clear();

for (l_it = new_facts.begin(); l_it != new_facts.end(); l_it++) {
 if (!m_fexecuted[*l_it])
 for (a_it = fa_links[*l_it].begin(); a_it != fa_links[*l_it].end(); a_it++) 
 { 
  pos = (*a_it)->get_index();
  if(++m_fsatisfied[pos] == m_iactions_bd[pos] ) {
    hd = (((*a_it)->get_effects())->front()).get_head();
    for (hd_it = hd->begin(); hd_it != hd->end(); hd_it++) 
    if (!m_fexecuted[*hd_it])
    {
       	next_facts.insert(*hd_it);
    	n_actions[*hd_it] = level+1;
    }
   }
  }
  m_fexecuted[*l_it] = 1;
 }

 new_facts = next_facts;
 facts.insert(next_facts.begin(), next_facts.end());
 level++;
}

while ((!goal_satisfied(m_gd, &facts))&&(new_facts.size() != 0));
level = 0;
for (it = m_gd.begin(); it != m_gd.end(); it++) {
  for (hd_it = it->begin(); hd_it != it->end(); hd_it++) {
    if (facts.count(*hd_it)) {
      level = level + n_actions[*hd_it];
    }
  }
}
return level;
};

bool Planner::goal_satisfies(const Literals* goal, const Literals* x)
{
  Literals::const_iterator it;
  for (it = x->begin(); it != x->end(); it++) {
    if (goal->find(*it) != goal->end())
      return true;
  }
  return false;
}

bool Planner::goal_satisfied(const set<Literals> gd, const Literals* x) 
{
  set<Literals>::const_iterator it;

  // goal satisfaction has not been checked
  for (it = (gd).begin();
       it != (gd).end(); it++) {
    if (!goal_satisfies(&(*it), x)) {
      return false;
    }
  }

  return true;
}
void Planner::rebuild()
{
	while (!m_queue.empty()) m_queue.pop();
        m_gd.clear();
        m_goal.clear();
        m_cstates.clear();
	m_states.clear();
	m_action_map.clear();
  	m_actions.clear();
  	m_statics.clear(); 
      	build_domain();
	build_goal();
	
}

void Planner::printplan(CState* myfinalstate)
{
	if (myfinalstate->get_previous_cstate() != NULL) {
		printplan((CState*)myfinalstate->get_previous_cstate());
  	}
	if (myfinalstate->get_action()!= NULL) {
		map<string,int>::iterator it;
                string name = myfinalstate->get_action()->get_name();
		it = my_action_map.find(name);
		if(it == my_action_map.end())
		{			
			my_action_map.insert(map<string,int>::value_type(name,my_action_map.size()));
                	my_plan.push_back(my_action_map.size()-1);
		}
        	else
		{
		      	my_plan.push_back(it->second);
		}
	}
}

#include "define.h"

/****************************************************************
* Formula Class          
*****************************************************************/
Formula::Formula()
{
 _t1 = 0;
 _t2 = 0;
}

Formula::Formula(OperatorType opr, TermList qparm, Formula *t1, Formula *t2){
 _operator = opr;
 _quantified_parameters = qparm;
 _t2 = t2;
 _t1 = t1;
}

void Formula::print() {
 TermList::iterator lterm;
 if (this != 0) {
     switch (_operator) {
        case 0 : cout << "\nAND "; break;
	case 1 : cout << "\nOR "; break;
	case 2 : cout << "IMPLY "; break;
	case 3 : cout << "NOT "; break;
	case 4 : cout << "FORALL ";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 cout << lterm->_name; 
		 }
		 break;
 	case 5 : cout << "EXIST ";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 cout << lterm->_name; 
		 }
		 break;
	case 6 : cout << "\nWHEN ";break;
	case 7 : //cout << "UNIOP_";
		 _term.print();
  		 break;
	case 8: cout << "\nUNKNOWN ";break;
	case 9: cout << "\nONEOF ";break;
     }
     cout << "( ";
     if (_t1 != 0) _t1->print();
     if (_t2 != 0) _t2->print();
     cout << ") ";
 }
}

void Formula::print_pl(string& str) {
 TermList::iterator lterm;
 string tmp;

 if (this != 0) {
     switch (_operator) {
        case 0 : str = str + string(prefix)+"and"; break;
	case 1 : str = str + string(prefix)+"or"; break;
	case 2 : str = str + string(prefix)+"imply"; break;
	case 3 : str = str + "NOT "; break;
	case 4 : str = str + string(prefix)+"forall";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 str = str + lterm->_name; 
		 }
		 break;
 	case 5 : str = str + string(prefix)+"exist";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 str = str + lterm->_name; 
		 }
		 break;
	case 6 : str = str + string(prefix)+"when";break;
	case 7 : //cout << "UNIOP_";
		 _term.print_pl(str);
  		 break;
	case 8: str = str + string(prefix)+"unknown";break;
	case 9: str = str + string(prefix)+"oneof";break;
     }
     if ((_t1 != 0)||(_t2!=0)){
     str = str + "( ";
     if (_t1 != 0) _t1->print_pl(str);
     if (_t1 != 0)
     switch (_operator) {
          case 0:
          case 1:
          case 6: str = str + ", "; 
          break;    
     }
     if (_t2 != 0) _t2->print_pl(str);
     str = str + ")";
     }
 }

}

void Formula::print_oneof(string& str) {
 TermList::iterator lterm;
 string tmp;
 if (this != 0) {
     switch (_operator) {
        case 0 : 
	case 1 : 
	case 2 : 
	case 3 : 
	case 4 : 
 	case 5 : 
	case 6 : break;
	case 7 : //cout << "UNIOP_";
		 _term.print_pl(str);
  		 break;
	case 8: 
	case 9:  break;
     }
     if (((_t1 != 0)||(_t2!=0))&&(_operator != 9)){
    
     if (_t1 != 0) _t1->print_oneof(str);
     if (_t1 != 0)
     switch (_operator) {
          case 1: str = str + ", "; 
          break;    
     }
     if (_t2 != 0) _t2->print_oneof(str);
    
     }
 }

}

void Formula::print_init(string& str) {
 TermList::iterator lterm;
 string tmp;
 if (this != 0) {
     switch (_operator) {
        case 0 : break;
	case 1 : str = str + string(prefix)+"or"; break;
	case 2 : str = str + string(prefix)+"imply"; break;
	case 3 : str = str + "NOT "; break;
	case 4 : str = str + string(prefix)+"forall";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 str = str + lterm->_name; 
		 }
		 break;
 	case 5 : str = str + string(prefix)+"exist";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 str = str + lterm->_name; 
		 }
		 break;
	case 6 : str = str + string(prefix)+"when";break;
	case 7 : //cout << "UNIOP_";
		 _term.print_pl(str);
  		 break;
	case 8: str = str + string(prefix)+"unknown(";
		_term.print_pl(str);
		str = str + ")";
                break;
	case 9: str = str + string(prefix)+"oneof([";
  	        if (_t1 != 0) _t1->print_oneof(str);
		str = str + "])";
                break;
     }
     if (((_t1 != 0)||(_t2!=0))&&(_operator != 9)){
     if (_operator != 0) str = str + "( ";

     if (_t1 != 0) _t1->print_init(str);
     if (_t1 != 0)

     switch (_operator) {
          case 0: //if (tmp.size()!=str.size())
		   str = str + ").\n"+string(prefix)+"initially("; break;
          case 1:
          case 6: str = str + ", "; 
          break;    
     }
     if (_t2 != 0) _t2->print_init(str);
    if (_operator != 0) str = str + ")";
     }
 }

}

void Formula::print_goal(string& str) {
 TermList::iterator lterm;
 string tmp;
 if (this != 0) {
     switch (_operator) {
        case 0 : break;
	case 1 : str = str + string(prefix)+"or"; break;
	case 2 : str = str + string(prefix)+"imply"; break;
	case 3 : str = str + "NOT "; break;
	case 4 : str = str + string(prefix)+"forall";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 str = str + lterm->_name; 
		 }
		 break;
 	case 5 : str = str + string(prefix)+"exist";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 str = str + lterm->_name; 
		 }
		 break;
	case 6 : str = str + string(prefix)+"when";break;
	case 7 : //cout << "UNIOP_";
		 _term.print_pl(str);
  		 break;
	case 8: str = str + string(prefix)+"unknown(";
		_term.print_pl(str);
		str = str + ")";
                break;
	case 9: str = str + string(prefix)+"oneof";break;
        default: break;
     }
     if ((_t1 != 0)||(_t2!=0)){
     if (_operator != 0) str = str + "( ";
     if (_t1 != 0) _t1->print_goal(str);
     if (_t1 != 0)
     switch (_operator) {
          case 0: str = str + ").\n"+"plan_goal("; break;
          case 1:
          case 6: str = str + ", "; 
          break;    
     }
     if (_t2 != 0) _t2->print_goal(str);
    if (_operator != 0) str = str + ")";
     }
 }

}

void Formula::print_cause_pl() {
 TermList::iterator lterm;
 if (this != 0) {
     switch (_operator) {
        case 0 : cout << string(prefix)+"and"; break;
	case 1 : cout << string(prefix)+"or"; break;
	case 2 : cout << string(prefix)+"imply"; break;
	case 3 : cout << "NOT "; break;
	case 4 : cout << string(prefix)+"forall";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 cout << lterm->_name; 
		 }
		 break;
 	case 5 : cout << string(prefix)+"exist";
		 for (lterm = _quantified_parameters.begin();lterm != _quantified_parameters.end(); lterm++) {
		 cout << lterm->_name; 
		 }
		 break;
	case 6 : cout << string(prefix)+"when";break;
	case 7 : //cout << "UNIOP_";
//		 _term.print_pl();
  		 break;
	case 8: cout << string(prefix)+"unknown";break;
	case 9: cout << string(prefix)+"oneof";break;
     }
     if ((_t1 != 0)||(_t2!=0)){
     cout << "( ";
     if (_t1 != 0) _t1->print_cause_pl();
     if (_t2 != 0) _t2->print_cause_pl();
     cout << ") ";
     }
 }
}

void Formula::negation()
 {
   switch (_operator) {
    case AND_:
	_operator = OR_;
        if (_t1 != 0) _t1->negation();
        if (_t2 != 0) _t2->negation();
        break;
    case OR_:
	_operator = AND_;
        if (_t1 != 0) _t1->negation();
	if (_t2 != 0) _t2->negation();
	break;
    case NOT_:	
	if (_t1 != 0) {
	_operator = _t1->_operator;
	_quantified_parameters = _t1->_quantified_parameters;
	_term = _t1->_term;

        _t2 = _t1->_t2;//must create a copy _t2 before a copy _t1
	_t1 = _t1->_t1;	
 	}
	break;
    case QFORALL_: 
	_operator = QEXISTS_;
	if (_t1 != 0) _t1->negation();
	break;
    case QEXISTS_:
	_operator = QFORALL_;
	if (_t1 != 0) _t1->negation();
	break;
    case UNIOP_:
	if (_term._neg == true ) _term._neg = false;
	else _term._neg = true;
	break; 
    default: break;
   }
 }
 

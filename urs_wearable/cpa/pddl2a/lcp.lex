/* Scanner for AL language */

%{
#include "define.h"
#include "tok.h"

int yyerror(char *s);
// int yylineno = 1;

int make_string(const string s, int token);
%}

digit	[0-9]
number	{digit}+
name [a-zA-Z][a-zA-Z0-9\-_]*
comment ;.*$

%%

"domain" {return dDOMAIN;}
"define" {return DEFINE;}
":requirements" {return REQUIREMENTS;}
":strips"	return make_string(yytext,STRIPS);
":adl"		return make_string(yytext,ADL);
":typing"	return make_string(yytext,TYPING);

":negative-preconditions"	return make_string(yytext,NEGATIVE_PRECONDITIONS);
":disjunctive-preconditions"	return make_string(yytext,DISJUNCTIVE_PRECONDITIONS);
":equality"			return make_string(yytext,EQUALITY);
":existential-preconditions"	return make_string(yytext,EXISTENTIAL_PRECONDITIONS);
":universal-preconditions"	return make_string(yytext,UNIVERSAL_PRECONDITIONS);
":quantified-preconditions"	return make_string(yytext,QUANTIFIED_PRECONDITIONS);
":conditional-effects"		return make_string(yytext,CONDITIONAL_EFFECTS);
":derived-predicates"		return make_string(yytext,DERIVED_PREDICATES);
":types" 	{return TYPES;}
":constants" 	return make_string(yytext,CONSTANTS);
":predicates" 	{return PREDICATES;}
":constraints" 	{return CONSTRAINTS;}
"preference "	{return PREFERENCE;}
":action" 	{return ACTION;}
":derived" 	{return DERIVED;}
":precondition" {return PRECONDITION;}
":effect" 	{return EFFECT;}
":parameters" 	{return PARAMETERS;}

"either"	{return EITHER;}
"when" 		return make_string(yytext, WHEN);
"or" 		return make_string(yytext, OR);
"and" 		return make_string(yytext, AND);
"not" 		return make_string(yytext, NOT);
"imply" 	return make_string(yytext, IMPLY);
"exists" 	return make_string(yytext, EXISTS);
"forall" 	return make_string(yytext, FORALL);
"at end" 		return make_string(yytext, ATEND);
"always" 	return make_string(yytext, ALWAYS);
"sometime" 	return make_string(yytext, SOMETIME);
"within" 	return make_string(yytext, WITHIN);
"at-most-once" 	return make_string(yytext,AT_MOST_ONCE);
"sometime-after" 	return make_string(yytext, SOMETIME_AFTER);
"sometime-before" 	return make_string(yytext, SOMETIME_BEFORE);
"always-within" 	return make_string(yytext, ALWAYS_WITHIN);
"hold-during" 		return make_string(yytext, HOLD_DURING);
"hold-after" 		return make_string(yytext, HOLD_AFTER);

"problem" 	{return PROBLEM;}
":domain"	{return pDOMAIN;}
":objects" 	{return OBJECT;}
":init"		{return INIT;}
":goal"		{return GOAL;}

"unknown"	{return UNKNOWN;}
"oneof"		{return ONEOF;}

"="		{return EQUAL;}
"-" 		{return MINUS;}
";"		{return SEMICOLON;} 
"(" 		{return LEFT_PAREN;}
")" 		{return RIGHT_PAREN;}


{name} 		{return make_string(yytext, NAME);}
\?{name}		{return make_string(yytext, VARIABLE);}
{number}	{return make_string(yytext, NUMBER);}

[ \t]+	{} /*whitespace*/
[\n]		{ yylineno++;	}
{comment} ;

.		{ std::cerr << "SCANNER "; yyerror(""); exit(1);	}
%%

int make_string(const string s, int token) {
  yylval.str_val = new std::string(s);
  return token;
}

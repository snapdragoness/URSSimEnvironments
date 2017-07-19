
:- use_module(library(lists)).
:- dynamic executable/2.
:- dynamic cpa_executable/2.
:- dynamic causes/3.
:- dynamic cpa_causes/3.

%%%% Objects %%%%
cpa_node(cpa_n1).
cpa_object(cpa_n1).
cpa_node(cpa_n2).
cpa_object(cpa_n2).

%%%% Constants %%%%

%%%%  Types rules %%%%

%%%% Predicates %%%%
fluent(cpa_visited(  X)):-
	cpa_node( X).

fluent(cpa_edge(  X,  Y)):-
	cpa_node( X), cpa_node( Y).

fluent(cpa_at(  X)):-
	cpa_node( X).

fluent(cpa_started).


%%%% Actions %%%%
action(cpa_start(  X)):-
	cpa_node( X).

action(cpa_travel(  X,  Y)):-
	cpa_node( X), cpa_node( Y).


%%%% Preconditions %%%%
executable(cpa_start(  X), [
 ]):-
	cpa_node( X).

executable(cpa_travel(  X,  Y), [
cpa_started ]):-
	cpa_node( X), cpa_node( Y).


%%%% Effects %%%%
causes(cpa_start(  X), [
cpa_when( cpa_at( X), cpa_and( cpa_started, cpa_and( cpa_visited( X)))) ], 
[]):-
	cpa_node( X).

causes(cpa_travel(  X,  Y), [
cpa_when( cpa_and( cpa_and( cpa_at( X)), cpa_edge( X,  Y)), cpa_and( cpa_visited( Y), cpa_and( cpa_and( cpa_at( Y)), neg(cpa_at( X))))) ], 
[]):-
	cpa_node( X), cpa_node( Y).


%%%% Inits %%%%
cpa_initially(cpa_edge(cpa_n1, cpa_n2)).
cpa_initially(cpa_edge(cpa_n2, cpa_n1)).
cpa_initially(cpa_oneof([cpa_at(cpa_n1), cpa_at(cpa_n2)])).
unknown(X):- fluent(X),
            findall(L, (cpa_initially(cpa_oneof(Y)), member(L,Y)), LUnk),
            member(X, LUnk).
unknown(X):- fluent(X), cpa_unknown(X).
unknown(X) :- fluent(X), (cpa_initially(cpa_or(Y)),in_or(Y,X);
          cpa_initially(cpa_or(Y,Z)), (in_or(Y,X);in_or(Z,X))), !.
in_or(X,X) :- !.
in_or(neg(X),X) :- !.
in_or(cpa_or(Y),X) :- in_or(Y,X).
in_or(cpa_or(Y,Z),X) :- (in_or(Y,X);in_or(Z,X)).
cpa_unknown(nop).

%%%% Goals %%%%
plan_goal(cpa_visited(cpa_n1)).
plan_goal(cpa_visited(cpa_n2)).

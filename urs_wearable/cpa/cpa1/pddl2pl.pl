
:- use_module(library(lists)).
:- dynamic executable/2.
:- dynamic cpa_executable/2.
:- dynamic causes/3.
:- dynamic cpa_causes/3.

%%%% Objects %%%%
cpa_uav_id(cpa_uav_id0).
cpa_object(cpa_uav_id0).
cpa_uav_id(cpa_uav_id1).
cpa_object(cpa_uav_id1).
cpa_uav_id(cpa_uav_id2).
cpa_object(cpa_uav_id2).
cpa_uav_id(cpa_uav_id3).
cpa_object(cpa_uav_id3).
cpa_wp_id(cpa_wp_id0).
cpa_object(cpa_wp_id0).
cpa_wp_id(cpa_wp_id1).
cpa_object(cpa_wp_id1).
cpa_wp_id(cpa_wp_id2).
cpa_object(cpa_wp_id2).
cpa_wp_id(cpa_wp_id3).
cpa_object(cpa_wp_id3).
cpa_wp_id(cpa_wp_id4).
cpa_object(cpa_wp_id4).
cpa_wp_id(cpa_wp_id5).
cpa_object(cpa_wp_id5).

%%%% Constants %%%%

%%%%  Types rules %%%%

%%%% Predicates %%%%
fluent(cpa_at(  ID,  WP)):-
	cpa_uav_id( ID), cpa_wp_id( WP).


%%%% Actions %%%%
action(cpa_goto(  ID,  WP0,  WP1)):-
	cpa_uav_id( ID), cpa_wp_id( WP0), cpa_wp_id( WP1).


%%%% Preconditions %%%%
executable(cpa_goto(  ID,  WP0,  WP1), [
cpa_at( ID,  WP0) ]):-
	cpa_uav_id( ID), cpa_wp_id( WP0), cpa_wp_id( WP1).


%%%% Effects %%%%
causes(cpa_goto(  ID,  WP0,  WP1), [
cpa_and( cpa_and( neg(cpa_at( ID,  WP0))), cpa_at( ID,  WP1)) ], 
[]):-
	cpa_uav_id( ID), cpa_wp_id( WP0), cpa_wp_id( WP1).


%%%% Inits %%%%
cpa_initially(cpa_at(cpa_uav_id0, cpa_wp_id0)).
cpa_initially(cpa_at(cpa_uav_id1, cpa_wp_id1)).
cpa_initially(cpa_at(cpa_uav_id2, cpa_wp_id2)).
cpa_initially(cpa_at(cpa_uav_id3, cpa_wp_id3)).
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
plan_goal(cpa_at(cpa_uav_id1, cpa_wp_id4)).
plan_goal(cpa_at(cpa_uav_id3, cpa_wp_id5)).

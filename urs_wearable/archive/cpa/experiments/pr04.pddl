(define (domain coins)
  (:requirements :conditional-effects :equality :typing)
  (:types elevator floor pos coin)
  (:predicates (dec_f ?f ?g - floor)
               (dec_p ?p ?q - pos)
               (in ?e - elevator ?f - floor)
               (at ?f - floor ?p - pos)
               (shaft ?e - elevator ?p - pos)
               (inside ?e - elevator)
               (coin-at ?c - coin ?f - floor ?p - pos)
               (have ?c - coin)
  )
  (:action go-up
    :parameters (?e - elevator ?f ?nf - floor)
    :precondition (dec_f ?nf ?f)
    :effect (when (in ?e ?f) (and (in ?e ?nf) (not (in ?e ?f))))
  )
  (:action go-down
    :parameters (?e - elevator ?f ?nf - floor)
    :precondition (dec_f ?f ?nf)
    :effect (when (in ?e ?f) (and (in ?e ?nf) (not (in ?e ?f))))
  )
  (:action step-in
    :parameters (?e - elevator ?f - floor ?p - pos)
    :precondition (and (at ?f ?p) (shaft ?e ?p))
    :effect (when (in ?e ?f) (and (inside ?e) (not (at ?f ?p))))
  )
  (:action step-out
    :parameters (?e - elevator ?f - floor ?p - pos)
    :precondition (and (inside ?e) (shaft ?e ?p))
    :effect (when (in ?e ?f) (and (at ?f ?p) (not (inside ?e))))
  )
  (:action move-left
    :parameters (?f - floor ?p ?np - pos)
    :precondition (and (at ?f ?p) (dec_p ?p ?np))
    :effect (and (not (at ?f ?p)) (at ?f ?np))
  )
  (:action move-right
    :parameters (?f - floor ?p ?np - pos)
    :precondition (and (at ?f ?p) (dec_p ?np ?p))
    :effect (and (not (at ?f ?p)) (at ?f ?np))
  )
  (:action collect
    :parameters (?c - coin ?f - floor ?p - pos)
    :precondition (at ?f ?p)
    :effect (when (coin-at ?c ?f ?p) (and (have ?c) (not (coin-at ?c ?f ?p))))
  )
)
(define (problem coins_2_2_2_17260)
  (:domain coins)
  (:objects e0 e1 - elevator f0 f1 - floor p0 p1 - pos c0 c1 - coin)
  (:init (and (dec_f f1 f0) (dec_p p1 p0) (shaft e0 p0) (oneof (in e0 f0) (in e0 f1)) (shaft e1 p1) (oneof (in e1 f0) (in e1 f1)) (oneof (coin-at c0 f1 p0) (coin-at c0 f1 p1)) (oneof (coin-at c1 f1 p0) (coin-at c1 f1 p1)) (at f0 p0)))
  (:goal (and (have c0) (have c1)))
)
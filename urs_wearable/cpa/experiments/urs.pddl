(define (domain urs)
  (:requirements :conditional-effects :equality :typing)
  (:types id cx cy cz)
  (:predicates (at ?id - id ?x - cx ?y - cy ?z - cz)
  )
  (:action goto
    :parameters (?id - id ?x0 ?x1 - cx ?y0 ?y1 - cy ?z0 ?z1 - cz)
    :precondition (at ?id ?x0 ?y0 ?z0)
    :effect (and (not (at ?id ?x0 ?y0 ?z0)) (at ?id ?x1 ?y1 ?z1))
  )
)
(define (problem urs_prob)
  (:domain urs)
  (:objects id0 - id x0 x2 - cx y0 y3 - cy z1 z4 - cz)
  (:init (and (at id0 x0 y0 z1) (not (at id0 x2 y3 z4))))
  (:goal (at id0 x2 y3 z4))
)
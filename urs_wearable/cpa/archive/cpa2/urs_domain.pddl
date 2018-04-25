(define (domain urs)
  (:requirements :conditional-effects :typing :equality)
  (:types uav_id wp_id)
  (:predicates
    (at ?id - uav_id ?wp - wp_id)
  )
  (:action goto
    :parameters (?id - uav_id ?wp0 - wp_id ?wp1 - wp_id)
    :precondition (at ?id ?wp0)
    :effect (and (not (at ?id ?wp0)) (at ?id ?wp1))
  )
)
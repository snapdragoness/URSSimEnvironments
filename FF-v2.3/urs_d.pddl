(define (domain urs)
  (:requirements :strips :typing :equality :adl)
  (:types area drone loc)
  (:predicates
    (above ?l0 ?l1 - loc)
    (aligned ?l0 ?l1 - loc)
    (at ?d - drone ?l - loc)
    (collided ?l0 ?l1 - loc)
    (hovered ?d - drone)
    (in ?l - loc ?a - area)
    (low_battery ?d - drone)
    (scanned ?d - drone ?a - area)
  )
  (:action ascend
    :parameters (?d - drone ?l0 ?l1 - loc)
    :precondition (and
      (above ?l1 ?l0)
      (aligned ?l0 ?l1)
      (at ?d ?l0)
      (not (exists (?x - drone ?y - loc)
        (and
          (not (= ?x ?d))
          (at ?x ?y)
          (or
            (= ?y ?l1)
            (collided ?y ?l1)
          )
        )
      ))
      (hovered ?d)
      (not (low_battery ?d))
    )
    :effect (and
      (not (at ?d ?l0))
      (at ?d ?l1)
    )
  )
  (:action descend
    :parameters (?d - drone ?l0 ?l1 - loc)
    :precondition (and
      (above ?l0 ?l1)
      (aligned ?l0 ?l1)
      (at ?d ?l0)
      (not (exists (?x - drone ?y - loc)
        (and
          (not (= ?x ?d))
          (at ?x ?y)
          (or
            (= ?y ?l1)
            (collided ?y ?l1)
          )
        )
      ))
      (hovered ?d)
      (not (low_battery ?d))
    )
    :effect (and
      (not (at ?d ?l0))
      (at ?d ?l1)
    )
  )
  (:action gather
    :parameters (?d - drone ?l0 ?l1 - loc)
    :precondition (and
      (at ?d ?l0)
      (exists (?x - drone ?y - loc)
        (and
          (not (= ?x ?d))
          (at ?x ?y)
          (or
            (= ?y ?l1)
            (collided ?y ?l1)
          )
        )
      )
      (hovered ?d)
      (not (low_battery ?d))
    )
    :effect (and
      (not (at ?d ?l0))
      (at ?d ?l1)
    )
  )
  (:action land
    :parameters (?d - drone)
    :precondition (hovered ?d)
    :effect (and
      (not (hovered ?d))
    )
  )
  (:action move
    :parameters (?d - drone ?l0 ?l1 - loc)
    :precondition (and
      (at ?d ?l0)
      (not (exists (?x - drone ?y - loc)
        (and
          (not (= ?x ?d))
          (at ?x ?y)
          (or
            (= ?y ?l1)
            (collided ?y ?l1)
          )
        )
      ))
      (hovered ?d)
      (not (low_battery ?d))
    )
    :effect (and
      (not (at ?d ?l0))
      (at ?d ?l1)
    )
  )
  (:action scan
    :parameters (?d  - drone ?l - loc ?a - area)
    :precondition (and
      (at ?d ?l)
      (not (exists (?x - drone ?y - loc)
        (and
          (not (= ?x ?d))
          (at ?x ?y)
          (in ?y ?a)
        )
      ))
      (hovered ?d)
      (not (scanned ?d ?a))
    )
    :effect (and
      (not (at ?d ?l))
      (forall (?y - loc)
        (when (in ?y ?a)
          (at ?d ?y)
        )
      )
      (scanned ?d ?a)
    )
  )
  (:action takeoff
    :parameters (?d - drone)
    :precondition (and
      (not (hovered ?d))
      (not (low_battery ?d))
    )
    :effect (and
      (hovered ?d)
    )
  )
)

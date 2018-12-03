(define (domain urs)
  (:requirements :strips :typing :equality :adl)
  (:types drone_id loc_id)
  (:predicates
    (drone_above ?d - drone_id ?l - loc_id)
    (drone_at ?d - drone_id ?l - loc_id)
    (took_off ?d - drone_id)
  )
  (:action fly_above
    :parameters (?d - drone_id ?from ?to - loc_id)
    :precondition (and
      (or (drone_above ?d ?from) (drone_at ?d ?from))
      (took_off ?d)
      (not (= ?from ?to))
      (not (exists (?x - drone_id) (drone_above ?x ?to)))
      (not (exists (?x - drone_id) (drone_at ?x ?to)))
    )
    :effect (and
      (not (drone_above ?d ?from))
      (not (drone_at ?d ?from))
      (drone_above ?d ?to)
    )
  )
  (:action fly_to
    :parameters (?d - drone_id ?from ?to - loc_id)
    :precondition (and
      (or (drone_above ?d ?from) (drone_at ?d ?from))
      (took_off ?d)
      (not (= ?from ?to))
      (not (exists (?x - drone_id) (drone_above ?x ?to)))
      (not (exists (?x - drone_id) (drone_at ?x ?to)))
    )
    :effect (and
      (not (drone_above ?d ?from))
      (not (drone_at ?d ?from))
      (drone_at ?d ?to)
    )
  )
  (:action land
    :parameters (?d - drone_id)
    :precondition (took_off ?d)
    :effect (and (not (took_off ?d)))
  )
  (:action take_off
    :parameters (?d - drone_id)
    :precondition (not (took_off ?d))
    :effect (and (took_off ?d))
  )
)

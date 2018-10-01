; Action's :effect must have 'and'
; Problem definition's :init must have 'and'
; All :types must be in problem defintion's :objects
(define (domain urs)
  (:requirements :strips :typing :equality :adl)
  (:types drone_id key_id loc_id)
  (:predicates
    (active_region ?loc_id_sw - loc_id ?loc_id_ne - loc_id)
    (drone_above ?drone_id - drone_id ?loc_id - loc_id)
    (drone_at ?drone_id - drone_id ?loc_id - loc_id)
    ; (key_at ?key_id - key_id ?loc_id - loc_id)
    ; (key_with ?key_id - key_id ?drone_id - drone_id)
    (took_off ?drone_id - drone_id)
  )
  (:action active_region_update
    :parameters (?loc_id_sw_old ?loc_id_ne_old ?loc_id_sw_new ?loc_id_ne_new - loc_id)
    :precondition (and
      (active_region ?loc_id_sw_old ?loc_id_ne_old)
      (not (and (= ?loc_id_sw_old ?loc_id_sw_new) (= ?loc_id_ne_old ?loc_id_ne_new)))
      (not (= ?loc_id_sw_new ??loc_id_ne_new))
      (not (exists (?x - drone_id) (drone_at ?x ?loc_id_sw_new)))
      (not (exists (?x - drone_id) (drone_at ?x ?loc_id_ne_new)))
    )
    :effect (and (not (active_region ?loc_id_sw_old ?loc_id_ne_old)) (active_region ?loc_id_sw_new ?loc_id_ne_new))
  )
  (:action fly_above
    :parameters (?drone_id - drone_id ?loc_id_from ?loc_id_to - loc_id)
    :precondition (and
      (or (drone_above ?drone_id ?loc_id_from) (drone_at ?drone_id ?loc_id_from))
      (took_off ?drone_id)
      (not (= ?loc_id_from ?loc_id_to))
      (not (exists (?x - drone_id) (drone_above ?x ?loc_id_to)))
      (not (exists (?x - drone_id) (drone_at ?x ?loc_id_to)))
      (not (exists (?x - loc_id) (active_region ?x ?loc_id_to)))
      (not (exists (?x - loc_id) (active_region ?loc_id_to ?x)))
    )
    :effect (and
      (not (drone_above ?drone_id ?loc_id_from))
      (not (drone_at ?drone_id ?loc_id_from))
      (drone_above ?drone_id ?loc_id_to)
    )
  )
  (:action fly_to
    :parameters (?drone_id - drone_id ?loc_id_from ?loc_id_to - loc_id)
    :precondition (and
      (or (drone_above ?drone_id ?loc_id_from) (drone_at ?drone_id ?loc_id_from))
      (took_off ?drone_id)
      (not (= ?loc_id_from ?loc_id_to))
      (not (exists (?x - drone_id) (drone_above ?x ?loc_id_to)))
      (not (exists (?x - drone_id) (drone_at ?x ?loc_id_to)))
      (not (exists (?x - loc_id) (active_region ?x ?loc_id_to)))
      (not (exists (?x - loc_id) (active_region ?loc_id_to ?x)))
    )
    :effect (and
      (not (drone_above ?drone_id ?loc_id_from))
      (not (drone_at ?drone_id ?loc_id_from))
      (drone_at ?drone_id ?loc_id_to)
    )
  )
  ; (:action key_add
  ;   :parameters (?key_id - key_id ?loc_id - loc_id)
  ;   :precondition (and
  ;     (not (exists (?x - loc_id) (key_at ?key_id ?x)))
  ;     (not (exists (?x - drone_id) (key_with ?key_id ?x)))
  ;   )
  ;   :effect (and
  ;     (key_at ?key_id ?loc_id)
  ;   )
  ; )
  ; (:action key_pick
  ;   :parameters (?drone_id - drone_id ?key_id - key_id ?loc_id - loc_id)
  ;   :precondition (and
  ;     (drone_above ?drone_id ?loc_id)
  ;     (key_at ?key_id ?loc_id)
  ;   )
  ;   :effect (and
  ;     (not (key_at ?key_id ?loc_id))
  ;     (key_with ?key_id ?drone_id)
  ;   )
  ; )
  (:action land
    :parameters (?drone_id - drone_id)
    :precondition (took_off ?drone_id)
    :effect (and (not (took_off ?drone_id)))
  )
  (:action take_off
    :parameters (?drone_id - drone_id)
    :precondition (not (took_off ?drone_id))
    :effect (and (took_off ?drone_id))
  )
)

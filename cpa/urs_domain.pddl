; Action's :effect must have 'and'
; Problem definition's :init must have 'and'
; All :types must be in problem defintion's :objects
(define (domain urs)
  (:requirements :strips :typing)
  (:types drone_id key_id loc_id)
  (:predicates
    (active_region ?loc_id_sw - loc_id ?loc_id_ne - loc_id)
    (drone_above ?drone_id - drone_id ?loc_id - loc_id)
    (drone_at ?drone_id - drone_id ?loc_id - loc_id)
    (is_location ?loc_id - loc_id)
    (is_occupied ?loc_id - loc_id)
    (key_at ?key_id - key_id ?loc_id - loc_id)
    (key_picked ?key_id - key_id)
    (key_with ?key_id - key_id ?drone_id - drone_id)
    (took_off ?drone_id - drone_id)
  )
  (:action active_region_update
    :parameters (?loc_id_sw_old ?loc_id_ne_old ?loc_id_sw_new ?loc_id_ne_new - loc_id)
    :precondition (active_region ?loc_id_sw_old ?loc_id_ne_old)
    :effect (and (not (active_region ?loc_id_sw_old ?loc_id_ne_old)) (active_region ?loc_id_sw_new ?loc_id_ne_new))
  )
  (:action add_location
    :parameters (?loc_id - loc_id)
    :precondition (and (not (is_location ?loc_id)))
    :effect (and (is_location ?loc_id) (not (is_occupied ?loc_id)))
  )
  (:action fly_above
    :parameters (?drone_id - drone_id ?loc_id_from ?loc_id_to - loc_id)
    :precondition (and
      (or (drone_above ?drone_id ?loc_id_from) (drone_at ?drone_id ?loc_id_from))
      (not (is_occupied ?loc_id_to))
      (took_off ?drone_id)
    )
    :effect (and
      (not (drone_above ?drone_id ?loc_id_from))
      (not (drone_at ?drone_id ?loc_id_from))
      (drone_above ?drone_id ?loc_id_to)
      (not (is_occupied ?loc_id_from))
      (is_occupied ?loc_id_to)
    )
  )
  (:action fly_to
    :parameters (?drone_id - drone_id ?loc_id_from ?loc_id_to - loc_id)
    :precondition (and
      (or (drone_above ?drone_id ?loc_id_from) (drone_at ?drone_id ?loc_id_from))
      (not (is_occupied ?loc_id_to))
      (took_off ?drone_id)
    )
    :effect (and
      (not (drone_above ?drone_id ?loc_id_from))
      (not (drone_at ?drone_id ?loc_id_from))
      (drone_at ?drone_id ?loc_id_to)
      (not (is_occupied ?loc_id_from))
      (is_occupied ?loc_id_to)
    )
  )
  (:action key_add
    :parameters (?key_id - key_id ?loc_id - loc_id)
    :precondition (not (key_at ?key_id ?loc_id))
    :effect (and (key_at ?key_id ?loc_id) (not (key_picked ?key_id)))
  )
  (:action key_pick
    :parameters (?drone_id - drone_id ?key_id - key_id ?drone_loc_id ?key_loc_id - loc_id)
    :precondition (and (not (key_picked ?key_id)) (key_at ?key_id ?key_loc_id) (took_off ?drone_id) (or (drone_above ?drone_id ?drone_loc_id) (drone_at ?drone_id ?drone_loc_id)))
    :effect (and
      (not (drone_above ?drone_id ?drone_loc_id))
      (not (drone_at ?drone_id ?drone_loc_id))
      (drone_above ?drone_id ?key_loc_id)
      (key_picked ?key_id)
      (key_with ?key_id ?drone_id)
    )
  )
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

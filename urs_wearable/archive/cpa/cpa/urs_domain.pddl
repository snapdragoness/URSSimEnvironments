; Action's :effect must have 'and'
; Problem definition's :init must have 'and'
; All :types must be in problem defintion's :objects
(define (domain urs)
  (:requirements :typing :equality)
  (:types drone_id key_id location_id)
  (:predicates
    (active_region ?loc_id_sw - location_id ?loc_id_ne - location_id)
    (drone_above ?drone_id - drone_id ?loc_id - location_id)
    (drone_at ?drone_id - drone_id ?loc_id - location_id)
    (key_at ?key_id - key_id ?loc_id - location_id)
    (key_picked ?key_id - key_id ?drone_id - drone_id)
    (took_off ?drone_id - drone_id)
  )
  (:action active_region_insert
    :parameters (?loc_id_sw ?loc_id_ne - location_id)
    :precondition (not (= ?loc_id_sw ?loc_id_ne))
    :effect (and (active_region ?loc_id_sw ?loc_id_ne))
  )
  (:action active_region_update
    :parameters (?loc_id_sw_old ?loc_id_ne_old ?loc_id_sw_new ?loc_id_ne_new - location_id)
    :precondition (and (active_region ?loc_id_sw_old ?loc_id_ne_old) (not (= ?loc_id_sw_new ?loc_id_ne_new)))
    :effect (and (not (active_region ?loc_id_sw_old ?loc_id_ne_old)) (active_region ?loc_id_sw_new ?loc_id_ne_new))
  )
  (:action fly_above
    :parameters (?drone_id - drone_id ?loc_id_from ?loc_id_to - location_id)
    :precondition (and (took_off ?drone_id) (or (drone_above ?drone_id ?loc_id_from) (drone_at ?drone_id ?loc_id_from)))
    :effect (and (not (drone_above ?drone_id ?loc_id_from)) (not (drone_at ?drone_id ?loc_id_from)) (drone_above ?drone_id ?loc_id_to))
  ) 
  (:action fly_to
    :parameters (?drone_id - drone_id ?loc_id_from ?loc_id_to - location_id)
    :precondition (and (took_off ?drone_id) (or (drone_above ?drone_id ?loc_id_from) (drone_at ?drone_id ?loc_id_from)))
    :effect (and (not (drone_above ?drone_id ?loc_id_from)) (not (drone_at ?drone_id ?loc_id_from)) (drone_at ?drone_id ?loc_id_to))
  )
  (:action key_add
    :parameters (?key_id - key_id ?loc_id - location_id)
    :precondition (not (key_at ?key_id ?loc_id))
    :effect (and (key_at ?key_id ?loc_id))
  )
  (:action key_pick
    :parameters (?drone_id - drone_id ?drone_loc_id - location_id ?key_id - key_id ?key_loc_id - location_id)
    :precondition (and (key_at ?key_id ?key_loc_id) (or (drone_above ?drone_id ?drone_loc_id) (drone_at ?drone_id ?drone_loc_id)))
    :effect (and
      (not (drone_above ?drone_id ?drone_loc_id))
      (not (drone_at ?drone_id ?drone_loc_id))
      (not (key_at ?key_id ?key_loc_id))
      (drone_above ?drone_id ?key_loc_id)
      (key_picked ?key_id ?drone_id)
    )
  )
  (:action take_off
    :parameters (?drone_id - drone_id)
    :precondition (not (took_off ?drone_id))
    :effect (and (took_off ?drone_id))
  )
)

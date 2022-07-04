(define (domain maildelivery-domain)
 (:requirements :strips :typing :negative-preconditions)
 (:types _robot _location _package)
 (:predicates (robot_at ?r - _robot ?l - _location) (is_connected ?l_from - _location ?l_to - _location) (is_occupied ?l - _location) (robot_has_package ?p - _package ?r - _robot) (location_has_package ?p - _package ?l - _location))
 (:action move
  :parameters ( ?r - _robot ?l_from - _location ?l_to - _location)
  :precondition (and (is_connected ?l_from ?l_to) (robot_at ?r ?l_from) (not (is_occupied ?l_to)))
  :effect (and (not (robot_at ?r ?l_from)) (not (is_occupied ?l_from)) (robot_at ?r ?l_to) (is_occupied ?l_to)))
 (:action pickup
  :parameters ( ?p - _package ?r - _robot ?l - _location)
  :precondition (and (robot_at ?r ?l) (location_has_package ?p ?l))
  :effect (and (not (location_has_package ?p ?l)) (robot_has_package ?p ?r)))
 (:action drop
  :parameters ( ?p - _package ?r - _robot ?l - _location)
  :precondition (and (robot_at ?r ?l) (robot_has_package ?p ?r))
  :effect (and (not (robot_has_package ?p ?r)) (location_has_package ?p ?l)))
)


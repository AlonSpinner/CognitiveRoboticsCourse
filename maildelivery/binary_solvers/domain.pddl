(define (domain maildelivery-domain)
 (:requirements :strips :typing :numeric-fluents :durative-actions)
 (:types robot location package)
 (:predicates (robot_at ?r - robot ?l - location) (is_connected ?l_from - location ?l_to - location) (location_is_free ?l - location) (robot_has_package ?p - package ?r - robot) (location_has_package ?p - package ?l - location) (robot_not_holding_package ?r - robot) (road_is_free ?l_from - location ?l_to - location))
 (:functions (distance ?l_from - location ?l_to - location) (velocity ?r - robot))
 (:durative-action move
  :parameters ( ?r - robot ?l_from - location ?l_to - location)
  :duration (= ?duration (/ (distance ?l_from ?l_to) (velocity ?r)))
  :condition (and (at start (is_connected ?l_from ?l_to))(at start (robot_at ?r ?l_from))(over all (road_is_free ?l_to ?l_from))(over all (location_is_free ?l_to))(at end (location_is_free ?l_to)))
  :effect (and (at start (not (robot_at ?r ?l_from))) (at start (location_is_free ?l_from)) (at start (not (road_is_free ?l_from ?l_to))) (at end (robot_at ?r ?l_to)) (at end (not (location_is_free ?l_to))) (at end (road_is_free ?l_from ?l_to))))
 (:action pickup
  :parameters ( ?p - package ?r - robot ?l - location)
  :precondition (and (robot_at ?r ?l) (location_has_package ?p ?l) (robot_not_holding_package ?r))
  :effect (and (not (location_has_package ?p ?l)) (robot_has_package ?p ?r) (not (robot_not_holding_package ?r))))
 (:action drop
  :parameters ( ?p - package ?r - robot ?l - location)
  :precondition (and (robot_at ?r ?l) (robot_has_package ?p ?r))
  :effect (and (not (robot_has_package ?p ?r)) (location_has_package ?p ?l) (robot_not_holding_package ?r)))
)


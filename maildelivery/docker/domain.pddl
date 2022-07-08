(define (domain maildelivery-domain)
 (:requirements :strips :typing :negative-preconditions :disjunctive-preconditions :equality :numeric-fluents :conditional-effects :durative-actions)
 (:types robot location package)
 (:predicates (robot_at ?r - robot ?l - location) (is_connected ?l_from - location ?l_to - location) (location_is_free ?l - location) (robot_has_package ?p - package ?r - robot) (location_has_package ?p - package ?l - location) (robot_not_holding_package ?r - robot) (not_targeted ?l_to - location) (road_is_free ?l_from - location ?l_to - location) (package_goal ?p - package ?l - location))
 (:functions (charge ?r - robot) (distance ?l_from - location ?l_to - location) (location_packages_amount ?l - location))
 (:durative-action move
  :parameters ( ?r - robot ?l_from - location ?l_to - location)
  :duration (= ?duration (distance ?l_from ?l_to))
  :condition (and (at start (is_connected ?l_from ?l_to))(at start (not_targeted ?l_to))(at start (road_is_free ?l_to ?l_from))(at start (robot_at ?r ?l_from))(at start (<= (* 2 (distance ?l_from ?l_to)) (charge ?r)))(at start (or (= (location_packages_amount ?l_from) 0) (and (< 0 (location_packages_amount ?l_from)) (not (robot_not_holding_package ?r)))))(at end (location_is_free ?l_to)))
  :effect (and (at start (not (robot_at ?r ?l_from))) (at start (location_is_free ?l_from)) (at start (not (not_targeted ?l_to))) (at start (not (road_is_free ?l_from ?l_to))) (at end (robot_at ?r ?l_to)) (at end (not (location_is_free ?l_to))) (at end (not_targeted ?l_to)) (at end (road_is_free ?l_from ?l_to))))
 (:action pickup
  :parameters ( ?p - package ?r - robot ?l - location)
  :precondition (and (robot_at ?r ?l) (location_has_package ?p ?l) (robot_not_holding_package ?r))
  :effect (and (not (location_has_package ?p ?l)) (robot_has_package ?p ?r) (not (robot_not_holding_package ?r)) (decrease (location_packages_amount ?l) 1)))
 (:action drop
  :parameters ( ?p - package ?r - robot ?l - location)
  :precondition (and (robot_at ?r ?l) (robot_has_package ?p ?r))
  :effect (and (not (robot_has_package ?p ?r)) (location_has_package ?p ?l) (robot_not_holding_package ?r) (when (not (package_goal ?p ?l)) (increase (location_packages_amount ?l) 1))))
)


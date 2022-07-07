(define (domain maildelivery-domain)
 (:requirements :strips :typing :durative-actions :action-costs)
 (:types robot location package)
 (:predicates (robot_at ?r - robot ?l - location) (is_connected ?l_from - location ?l_to - location) (is_free ?l - location) (robot_has_package ?p - package ?r - robot) (location_has_package ?p - package ?l - location))
 (:functions (total-cost))
 (:durative-action move
  :parameters ( ?r - robot ?l_from - location ?l_to - location)
  :duration (= ?duration 1)
  :condition (and (at start (is_connected ?l_from ?l_to))(at start (robot_at ?r ?l_from))(at end (is_free ?l_to)))
  :effect (and (at start (not (robot_at ?r ?l_from))) (at start (is_free ?l_from)) (at end (robot_at ?r ?l_to)) (at end (not (is_free ?l_to))) (at end (increase total-cost 1))))
 (:action pickup
  :parameters ( ?p - package ?r - robot ?l - location)
  :precondition (and (robot_at ?r ?l) (location_has_package ?p ?l))
  :effect (and (not (location_has_package ?p ?l)) (robot_has_package ?p ?r) (increase total-cost 0)))
 (:action drop
  :parameters ( ?p - package ?r - robot ?l - location)
  :precondition (and (robot_at ?r ?l) (robot_has_package ?p ?r))
  :effect (and (not (robot_has_package ?p ?r)) (location_has_package ?p ?l) (increase total-cost 0)))
)


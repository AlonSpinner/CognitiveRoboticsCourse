(define (domain rovers2-domain)
 (:requirements :strips :typing :numeric-fluents :durative-actions)
 (:types robot location)
 (:predicates (robot_at ?r - robot ?l - location) (is_connected ?l_from - location ?l_to - location))
 (:functions (charge ?r - robot) (distance ?l_from - location ?l_to - location))
 (:durative-action move
  :parameters ( ?r - robot ?l_from - location ?l_to - location)
  :duration (= ?duration 1)
  :condition (and (at start (is_connected ?l_from ?l_to))(at start (robot_at ?r ?l_from))(at start (<= (* 2 (distance ?l_from ?l_to)) (charge ?r))))
  :effect (and (at start (not (robot_at ?r ?l_from))) (at end (robot_at ?r ?l_to)) (at end (decrease (charge ?r) (* 2 (distance ?l_from ?l_to))))))
)


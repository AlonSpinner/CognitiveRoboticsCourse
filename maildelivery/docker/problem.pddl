(define (problem maildelivery-problem)
 (:domain maildelivery-domain)
 (:objects 
   r0 - robot
   l0 l1 l2 l3 l4 l5 l6 - location
   p0 p1 - package
 )
 (:init (is_connected l0 l1) (is_connected l1 l0) (is_connected l1 l2) (is_connected l2 l1) (is_connected l2 l4) (is_connected l4 l2) (is_connected l3 l4) (is_connected l4 l3) (is_connected l1 l3) (is_connected l3 l1) (is_connected l3 l5) (is_connected l5 l3) (is_connected l4 l6) (is_connected l6 l4) (robot_at r0 l0) (location_has_package p0 l5) (location_has_package p1 l6) (is_free l1) (is_free l2) (is_free l3) (is_free l4) (is_free l5) (is_free l6))
 (:goal (and (location_has_package p0 l6) (robot_at r0 l0) (location_has_package p1 l5) (robot_at r0 l0)))
)


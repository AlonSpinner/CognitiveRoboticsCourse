(define (problem rovers2-problem)
 (:domain rovers2-domain)
 (:objects
   r0 - robot
   l0 l1 l2 l3 - location
 )
 (:init (is_connected l0 l1) (is_connected l1 l2) (is_connected l2 l3) (= (distance l0 l1) 1) (= (distance l1 l2) 2) (= (distance l2 l3) 3) (robot_at r0 l0) (= (charge r0) 40) (= (distance l0 l0) 100) (= (distance l1 l0) 100) (= (distance l2 l0) 100) (= (distance l3 l0) 100) (= (distance l1 l1) 100) (= (distance l2 l1) 100) (= (distance l3 l1) 100) (= (distance l0 l2) 100) (= (distance l2 l2) 100) (= (distance l3 l2) 100) (= (distance l0 l3) 100) (= (distance l1 l3) 100) (= (distance l3 l3) 100))
 (:goal (and (robot_at r0 l2)))
(:metric maximize(charge r0))
)
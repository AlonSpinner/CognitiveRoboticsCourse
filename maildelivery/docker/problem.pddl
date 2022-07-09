(define (problem turtlebot1) 
(:domain turtlebot) 
(:objects
r1 r2 - robot
w1 w2 w3 w4 w5 w6 w7 w8 w9 w10 - waypoint
)


		
(:init 

(docked r1)
(robot_at r1 w1)
(dock_at w1)

(docked r2)
(robot_at r2 w2)
(dock_at w2)
)

(:goal (and
  (visited w3)
  (visited w4)
  (visited w5)
  (visited w7)
  (visited w8)
  (visited w9)
  (docked r1)
  (docked r2)

)))


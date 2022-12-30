(define (problem navpb1) (:domain navigation)

(:objects
  r - robot
  waypoint0 waypoint1 waypoint2 waypoint3 - waypoint
)

(:init
  (adjacent waypoint0 waypoint1)
  (adjacent waypoint0 waypoint2)
  (adjacent waypoint1 waypoint2)
  (adjacent waypoint2 waypoint3)
  (at r waypoint0)
)

(:goal
  (at r waypoint3)
)

)

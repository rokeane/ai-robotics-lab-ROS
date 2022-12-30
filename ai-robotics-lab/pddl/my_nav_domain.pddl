(define (domain navigation)

(:types robot waypoint object_of_interest)
(:predicates
  (at ?x - robot ?y - waypoint)
  (adjacent ?x ?y - waypoint)
  (location ?z - object_of_interest ?y - waypoint)
  (observed ?z - object_of_interest)
)

(:action navigate
:parameters (?x - robot ?y - waypoint ?z - waypoint)
:precondition (and (adjacent ?y ?z) (at ?x ?y))
:effect (and (not (at ?x ?y)) (at ?x ?z))
)

(:action take_picture
:parameters (?x - robot ?y - waypoint ?z - object_of_interest)
:precondition (and (at ?x ?y)(location ?z ?y))
:effect (observed ?z)
)

)

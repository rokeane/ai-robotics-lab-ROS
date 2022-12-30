(define (domain navigation)

(:types robot waypoint)

(:predicates
  (at ?x - robot ?y - waypoint)
  (adjacent ?x ?y - waypoint)
)

(:action navigate
:parameters (?x - robot ?y - waypoint ?z - waypoint)
:precondition (and (adjacent ?y ?z) (at ?x ?y))
:effect (and (not (at ?x ?y)) (at ?x ?z))
)

)

(define (problem navpb) (:domain navigation)
(:objects
	r - robot
	wp_40 - waypoint
	wp_41 - waypoint
	wp_42 - waypoint
	wp_43 - waypoint
	wp_44 - waypoint
	wp_45 - waypoint
	wp_46 - waypoint
	wp_47 - waypoint
	wp_32 - waypoint
	wp_33 - waypoint
	wp_35 - waypoint
	wp_37 - waypoint
	wp_38 - waypoint
	wp_24 - waypoint
	wp_25 - waypoint
	wp_27 - waypoint
	wp_28 - waypoint
	wp_29 - waypoint
	wp_30 - waypoint
	wp_16 - waypoint
	wp_19 - waypoint
	wp_20 - waypoint
	wp_21 - waypoint
	wp_22 - waypoint
	wp_8 - waypoint
	wp_9 - waypoint
	wp_10 - waypoint
	wp_11 - waypoint
	wp_12 - waypoint
	wp_13 - waypoint
	wp_14 - waypoint
	wp_15 - waypoint
	wp_0 - waypoint
	wp_1 - waypoint
	wp_3 - waypoint
	wp_6 - waypoint
	wp_7 - waypoint
	o2 - object_of_interest
	o1 - object_of_interest
)
(:init
	(at r wp_12)
	(location o2 wp_35)
	(location o1 wp_46)
	(adjacent wp_40 wp_32)
	(adjacent wp_40 wp_41)
	(adjacent wp_41 wp_33)
	(adjacent wp_41 wp_40)
	(adjacent wp_41 wp_42)
	(adjacent wp_42 wp_41)
	(adjacent wp_42 wp_43)
	(adjacent wp_43 wp_35)
	(adjacent wp_43 wp_42)
	(adjacent wp_43 wp_44)
	(adjacent wp_44 wp_43)
	(adjacent wp_44 wp_45)
	(adjacent wp_45 wp_37)
	(adjacent wp_45 wp_44)
	(adjacent wp_45 wp_46)
	(adjacent wp_46 wp_38)
	(adjacent wp_46 wp_45)
	(adjacent wp_46 wp_47)
	(adjacent wp_47 wp_46)
	(adjacent wp_32 wp_40)
	(adjacent wp_32 wp_24)
	(adjacent wp_32 wp_33)
	(adjacent wp_33 wp_41)
	(adjacent wp_33 wp_25)
	(adjacent wp_33 wp_32)
	(adjacent wp_35 wp_43)
	(adjacent wp_35 wp_27)
	(adjacent wp_37 wp_45)
	(adjacent wp_37 wp_29)
	(adjacent wp_37 wp_38)
	(adjacent wp_38 wp_46)
	(adjacent wp_38 wp_30)
	(adjacent wp_38 wp_37)
	(adjacent wp_24 wp_32)
	(adjacent wp_24 wp_16)
	(adjacent wp_24 wp_25)
	(adjacent wp_25 wp_33)
	(adjacent wp_25 wp_24)
	(adjacent wp_27 wp_35)
	(adjacent wp_27 wp_19)
	(adjacent wp_27 wp_28)
	(adjacent wp_28 wp_20)
	(adjacent wp_28 wp_27)
	(adjacent wp_28 wp_29)
	(adjacent wp_29 wp_37)
	(adjacent wp_29 wp_21)
	(adjacent wp_29 wp_28)
	(adjacent wp_29 wp_30)
	(adjacent wp_30 wp_38)
	(adjacent wp_30 wp_22)
	(adjacent wp_30 wp_29)
	(adjacent wp_16 wp_24)
	(adjacent wp_16 wp_8)
	(adjacent wp_19 wp_27)
	(adjacent wp_19 wp_11)
	(adjacent wp_19 wp_20)
	(adjacent wp_20 wp_28)
	(adjacent wp_20 wp_12)
	(adjacent wp_20 wp_19)
	(adjacent wp_20 wp_21)
	(adjacent wp_21 wp_29)
	(adjacent wp_21 wp_13)
	(adjacent wp_21 wp_20)
	(adjacent wp_21 wp_22)
	(adjacent wp_22 wp_30)
	(adjacent wp_22 wp_14)
	(adjacent wp_22 wp_21)
	(adjacent wp_8 wp_16)
	(adjacent wp_8 wp_0)
	(adjacent wp_8 wp_9)
	(adjacent wp_9 wp_1)
	(adjacent wp_9 wp_8)
	(adjacent wp_9 wp_10)
	(adjacent wp_10 wp_9)
	(adjacent wp_10 wp_11)
	(adjacent wp_11 wp_19)
	(adjacent wp_11 wp_3)
	(adjacent wp_11 wp_10)
	(adjacent wp_11 wp_12)
	(adjacent wp_12 wp_20)
	(adjacent wp_12 wp_11)
	(adjacent wp_12 wp_13)
	(adjacent wp_13 wp_21)
	(adjacent wp_13 wp_12)
	(adjacent wp_13 wp_14)
	(adjacent wp_14 wp_22)
	(adjacent wp_14 wp_6)
	(adjacent wp_14 wp_13)
	(adjacent wp_14 wp_15)
	(adjacent wp_15 wp_7)
	(adjacent wp_15 wp_14)
	(adjacent wp_0 wp_8)
	(adjacent wp_0 wp_1)
	(adjacent wp_1 wp_9)
	(adjacent wp_1 wp_0)
	(adjacent wp_3 wp_11)
	(adjacent wp_6 wp_14)
	(adjacent wp_6 wp_7)
	(adjacent wp_7 wp_15)
	(adjacent wp_7 wp_6)
)
(:goal
	(and
	(observed o2)
	(observed o1)
	)
)
)
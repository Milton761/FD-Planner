(define (problem roverprob--s299974--m25--r2--w6--o3--c1--g4--p0--P1--Z200--X200--C10) (:domain roverdom--s299974--m25--r2--w6--o3--c1--g4--p0--P1--Z200--X200--C10)
(:objects
	general - Lander
	colour high_res low_res - Mode
	rover0 rover1 - Rover
	rover0store rover1store - Store
	camera0 camera1 - Camera
	objective0 objective1 objective2 - Objective
	)
(:init
    (road_isunknown road0)
    (road_isunknown road1)
    (road_isunknown road2)
    (road_isunknown road3)
    (road_isunknown road4)
    (road_isunknown road5)
    (road_isunknown road6)
    (road_isunknown road7)
    (road_isunknown road8)
    (road_isunknown road9)
    (road_isunknown road10)
	(visible waypoint0 waypoint1)
	(visible waypoint1 waypoint0)
	(visible waypoint0 waypoint2)
	(visible waypoint2 waypoint0)
	(visible waypoint0 waypoint4)
	(visible waypoint4 waypoint0)
	(visible waypoint0 waypoint5)
	(visible waypoint5 waypoint0)
	(visible waypoint1 waypoint4)
	(visible waypoint4 waypoint1)
	(visible waypoint2 waypoint1)
	(visible waypoint1 waypoint2)
	(visible waypoint2 waypoint3)
	(visible waypoint3 waypoint2)
	(visible waypoint3 waypoint0)
	(visible waypoint0 waypoint3)
	(visible waypoint3 waypoint4)
	(visible waypoint4 waypoint3)
	(visible waypoint5 waypoint2)
	(visible waypoint2 waypoint5)
	(visible waypoint5 waypoint4)
	(visible waypoint4 waypoint5)
	(at_soil_sample waypoint1)
	(at_rock_sample waypoint1)
	(at_rock_sample waypoint2)
	(at_soil_sample waypoint3)
	(at_rock_sample waypoint3)
	(at_rock_sample waypoint5)
	(at_lander general waypoint0)
	(channel_free general)
	(at rover0 waypoint3)
	(available rover0)
	(store_of rover0store rover0)
	(empty rover0store)
	(equipped_for_soil_analysis rover0)
	(equipped_for_rock_analysis rover0)
	(equipped_for_imaging rover0)
	(can_traverse rover0 waypoint0 waypoint1)
	(can_traverse rover0 waypoint1 waypoint0)
	(can_traverse rover0 waypoint0 waypoint2)
	(can_traverse rover0 waypoint2 waypoint0)
	(can_traverse rover0 waypoint0 waypoint4)
	(can_traverse rover0 waypoint4 waypoint0)
	(can_traverse rover0 waypoint0 waypoint5)
	(can_traverse rover0 waypoint5 waypoint0)
	(can_traverse rover0 waypoint1 waypoint4)
	(can_traverse rover0 waypoint4 waypoint1)
	(can_traverse rover0 waypoint2 waypoint1)
	(can_traverse rover0 waypoint1 waypoint2)
	(can_traverse rover0 waypoint2 waypoint3)
	(can_traverse rover0 waypoint3 waypoint2)
	(can_traverse rover0 waypoint3 waypoint0)
	(can_traverse rover0 waypoint0 waypoint3)
	(can_traverse rover0 waypoint3 waypoint4)
	(can_traverse rover0 waypoint4 waypoint3)
	(can_traverse rover0 waypoint5 waypoint2)
	(can_traverse rover0 waypoint2 waypoint5)
	(can_traverse rover0 waypoint5 waypoint4)
	(can_traverse rover0 waypoint4 waypoint5)
	(on_board camera0 rover0)
	(calibration_target camera0 objective0 )
	(calibration_target camera0 objective1 )
	(calibration_target camera0 objective2 )
	(supports camera0 colour)
	(on_board camera1 rover1)
	(calibration_target camera1 objective0 )
	(calibration_target camera1 objective1 )
	(calibration_target camera1 objective2 )
	(supports camera1 colour)
	(visible_from objective0 waypoint4)
	(visible_from objective0 waypoint2)
	(visible_from objective0 waypoint1)
	(visible_from objective0 waypoint3)
	(visible_from objective1 waypoint5)
	(visible_from objective1 waypoint3)
	(visible_from objective2 waypoint5)
	(visible_from objective2 waypoint0)
)

(:goal (and
(communicated_soil_data waypoint3)
(communicated_soil_data waypoint1)
(communicated_rock_data waypoint2)
(communicated_rock_data waypoint5)
	)
)
(:metric minimize (total-cost))
)

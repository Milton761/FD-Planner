(define (domain navigation)
	 (:requirements :typing :strips :equality :probabilistic-effects)
	 (:types posX posY)
	 (:predicates
	 	 (robot_at_X ?X - posX)
	 	 (robot_at_Y ?Y - posY)
	 	 (path-X-Y ?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 (robot-dead)
	 )
	 (:functions (total-cost))
	 (:action move_up_0_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 0.2 (robot-dead)) )
	 )
	 (:action move_right_0_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_up_1_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 0.4 (robot-dead)) )
	 )
	 (:action move_right_1_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_left_1_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_up_2_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 0.6 (robot-dead)) )
	 )
	 (:action move_right_2_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_left_2_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_up_3_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 0.8 (robot-dead)) )
	 )
	 (:action move_left_3_0
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_up_0_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_down_0_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_right_0_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 0.4 (robot-dead)) )
	 )
	 (:action move_up_1_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_down_1_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_right_1_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 0.6 (robot-dead)) )
	 )
	 (:action move_left_1_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 0.2 (robot-dead)) )
	 )
	 (:action move_up_2_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_down_2_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_right_2_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 0.8 (robot-dead)) )
	 )
	 (:action move_left_2_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 0.4 (robot-dead)) )
	 )
	 (:action move_up_3_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_down_3_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_left_3_1
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 0.6 (robot-dead)) )
	 )
	 (:action move_down_0_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 0.2 (robot-dead)) )
	 )
	 (:action move_right_0_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_down_1_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 0.4 (robot-dead)) )
	 )
	 (:action move_right_1_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_left_1_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_down_2_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 0.6 (robot-dead)) )
	 )
	 (:action move_right_2_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_left_2_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
	 (:action move_down_3_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic 0.8 (robot-dead)) )
	 )
	 (:action move_left_3_2
	 	 :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)
	 	 :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))
	 	 :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic 1 (robot-dead)) )
	 )
)

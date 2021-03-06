(define (domain dom--tpp-m7-g4-s588620)
(:requirements :typing :probabilistic-effects)
(:types
    place - domain
    depot market - place 
    locatable - domain
    truck goods - locatable
    road - object
)

(:predicates
    (at ?t - truck ?p - place)
    (on-sale ?g - goods ?m - market)
    (available ?g - goods ?m - market)
    (loaded ?g - goods ?t - truck)
    (stored ?g - goods)
    (road_isunknown ?r - road) (road_isblocked ?r - road) (road_isfree ?r - road))

(:functions (total-cost) (price ?g - goods ?m - market))

(:constants
    depot0 - depot
    market1 market2 market3 market4 market5 market6 market7 - market
    road0 road1 road2 road3 road4 road5 road6 road7 - road
)

(:action buy
    :parameters (?t - truck ?g - goods ?m - market)
    :precondition (and (on-sale ?g ?m))
    :effect (and (increase (total-cost) (price ?g ?m)) (not (on-sale ?g ?m)) (available ?g ?m)))

(:action load
    :parameters (?g - goods ?t - truck ?m - market)
    :precondition (and (available ?g ?m) (at ?t ?m))
    :effect (and (increase (total-cost) 1) (loaded ?g ?t) (not (available ?g ?m))))

(:action unload
    :parameters (?t - truck ?g - goods ?d - depot)
    :precondition (and (loaded ?g ?t) (at ?t ?d))
    :effect (and (increase (total-cost) 1) (stored ?g) (not (loaded ?g ?t))))

(:action try-drive-m0-m4
    :parameters (?t - truck)
    :precondition (and (road_isunknown road0) (at ?t depot0))
    :effect (and (increase (total-cost) 12) (not (road_isunknown road0)) (probabilistic
    200/1000 (and (road_isblocked road0))
    800/1000 (and (road_isfree road0) (not (at ?t depot0)) (at ?t market4)))))

(:action drive-m0-m4
    :parameters (?t - truck)
    :precondition (and (road_isfree road0) (at ?t depot0))
    :effect (and (increase (total-cost) 12) (not (at ?t depot0)) (at ?t market4)))

(:action try-drive-m4-m0
    :parameters (?t - truck)
    :precondition (and (road_isunknown road0) (at ?t market4))
    :effect (and (increase (total-cost) 12) (not (road_isunknown road0)) (probabilistic
    200/1000 (and (road_isblocked road0))
    800/1000 (and (road_isfree road0) (not (at ?t market4)) (at ?t depot0)))))

(:action drive-m4-m0
    :parameters (?t - truck)
    :precondition (and (road_isfree road0) (at ?t market4))
    :effect (and (increase (total-cost) 12) (not (at ?t market4)) (at ?t depot0)))

(:action try-drive-m0-m7
    :parameters (?t - truck)
    :precondition (and (road_isunknown road1) (at ?t depot0))
    :effect (and (increase (total-cost) 14) (not (road_isunknown road1)) (probabilistic
    200/1000 (and (road_isblocked road1))
    800/1000 (and (road_isfree road1) (not (at ?t depot0)) (at ?t market7)))))

(:action drive-m0-m7
    :parameters (?t - truck)
    :precondition (and (road_isfree road1) (at ?t depot0))
    :effect (and (increase (total-cost) 14) (not (at ?t depot0)) (at ?t market7)))

(:action try-drive-m7-m0
    :parameters (?t - truck)
    :precondition (and (road_isunknown road1) (at ?t market7))
    :effect (and (increase (total-cost) 14) (not (road_isunknown road1)) (probabilistic
    200/1000 (and (road_isblocked road1))
    800/1000 (and (road_isfree road1) (not (at ?t market7)) (at ?t depot0)))))

(:action drive-m7-m0
    :parameters (?t - truck)
    :precondition (and (road_isfree road1) (at ?t market7))
    :effect (and (increase (total-cost) 14) (not (at ?t market7)) (at ?t depot0)))

(:action try-drive-m1-m3
    :parameters (?t - truck)
    :precondition (and (road_isunknown road2) (at ?t market1))
    :effect (and (increase (total-cost) 8) (not (road_isunknown road2)) (probabilistic
    200/1000 (and (road_isblocked road2))
    800/1000 (and (road_isfree road2) (not (at ?t market1)) (at ?t market3)))))

(:action drive-m1-m3
    :parameters (?t - truck)
    :precondition (and (road_isfree road2) (at ?t market1))
    :effect (and (increase (total-cost) 8) (not (at ?t market1)) (at ?t market3)))

(:action try-drive-m3-m1
    :parameters (?t - truck)
    :precondition (and (road_isunknown road2) (at ?t market3))
    :effect (and (increase (total-cost) 8) (not (road_isunknown road2)) (probabilistic
    200/1000 (and (road_isblocked road2))
    800/1000 (and (road_isfree road2) (not (at ?t market3)) (at ?t market1)))))

(:action drive-m3-m1
    :parameters (?t - truck)
    :precondition (and (road_isfree road2) (at ?t market3))
    :effect (and (increase (total-cost) 8) (not (at ?t market3)) (at ?t market1)))

(:action try-drive-m1-m4
    :parameters (?t - truck)
    :precondition (and (road_isunknown road3) (at ?t market1))
    :effect (and (increase (total-cost) 23) (not (road_isunknown road3)) (probabilistic
    200/1000 (and (road_isblocked road3))
    800/1000 (and (road_isfree road3) (not (at ?t market1)) (at ?t market4)))))

(:action drive-m1-m4
    :parameters (?t - truck)
    :precondition (and (road_isfree road3) (at ?t market1))
    :effect (and (increase (total-cost) 23) (not (at ?t market1)) (at ?t market4)))

(:action try-drive-m4-m1
    :parameters (?t - truck)
    :precondition (and (road_isunknown road3) (at ?t market4))
    :effect (and (increase (total-cost) 23) (not (road_isunknown road3)) (probabilistic
    200/1000 (and (road_isblocked road3))
    800/1000 (and (road_isfree road3) (not (at ?t market4)) (at ?t market1)))))

(:action drive-m4-m1
    :parameters (?t - truck)
    :precondition (and (road_isfree road3) (at ?t market4))
    :effect (and (increase (total-cost) 23) (not (at ?t market4)) (at ?t market1)))

(:action try-drive-m1-m6
    :parameters (?t - truck)
    :precondition (and (road_isunknown road4) (at ?t market1))
    :effect (and (increase (total-cost) 22) (not (road_isunknown road4)) (probabilistic
    200/1000 (and (road_isblocked road4))
    800/1000 (and (road_isfree road4) (not (at ?t market1)) (at ?t market6)))))

(:action drive-m1-m6
    :parameters (?t - truck)
    :precondition (and (road_isfree road4) (at ?t market1))
    :effect (and (increase (total-cost) 22) (not (at ?t market1)) (at ?t market6)))

(:action try-drive-m6-m1
    :parameters (?t - truck)
    :precondition (and (road_isunknown road4) (at ?t market6))
    :effect (and (increase (total-cost) 22) (not (road_isunknown road4)) (probabilistic
    200/1000 (and (road_isblocked road4))
    800/1000 (and (road_isfree road4) (not (at ?t market6)) (at ?t market1)))))

(:action drive-m6-m1
    :parameters (?t - truck)
    :precondition (and (road_isfree road4) (at ?t market6))
    :effect (and (increase (total-cost) 22) (not (at ?t market6)) (at ?t market1)))

(:action try-drive-m2-m6
    :parameters (?t - truck)
    :precondition (and (road_isunknown road5) (at ?t market2))
    :effect (and (increase (total-cost) 2) (not (road_isunknown road5)) (probabilistic
    200/1000 (and (road_isblocked road5))
    800/1000 (and (road_isfree road5) (not (at ?t market2)) (at ?t market6)))))

(:action drive-m2-m6
    :parameters (?t - truck)
    :precondition (and (road_isfree road5) (at ?t market2))
    :effect (and (increase (total-cost) 2) (not (at ?t market2)) (at ?t market6)))

(:action try-drive-m6-m2
    :parameters (?t - truck)
    :precondition (and (road_isunknown road5) (at ?t market6))
    :effect (and (increase (total-cost) 2) (not (road_isunknown road5)) (probabilistic
    200/1000 (and (road_isblocked road5))
    800/1000 (and (road_isfree road5) (not (at ?t market6)) (at ?t market2)))))

(:action drive-m6-m2
    :parameters (?t - truck)
    :precondition (and (road_isfree road5) (at ?t market6))
    :effect (and (increase (total-cost) 2) (not (at ?t market6)) (at ?t market2)))

(:action try-drive-m5-m7
    :parameters (?t - truck)
    :precondition (and (road_isunknown road6) (at ?t market5))
    :effect (and (increase (total-cost) 1) (not (road_isunknown road6)) (probabilistic
    200/1000 (and (road_isblocked road6))
    800/1000 (and (road_isfree road6) (not (at ?t market5)) (at ?t market7)))))

(:action drive-m5-m7
    :parameters (?t - truck)
    :precondition (and (road_isfree road6) (at ?t market5))
    :effect (and (increase (total-cost) 1) (not (at ?t market5)) (at ?t market7)))

(:action try-drive-m7-m5
    :parameters (?t - truck)
    :precondition (and (road_isunknown road6) (at ?t market7))
    :effect (and (increase (total-cost) 1) (not (road_isunknown road6)) (probabilistic
    200/1000 (and (road_isblocked road6))
    800/1000 (and (road_isfree road6) (not (at ?t market7)) (at ?t market5)))))

(:action drive-m7-m5
    :parameters (?t - truck)
    :precondition (and (road_isfree road6) (at ?t market7))
    :effect (and (increase (total-cost) 1) (not (at ?t market7)) (at ?t market5)))

(:action try-drive-m6-m7
    :parameters (?t - truck)
    :precondition (and (road_isunknown road7) (at ?t market6))
    :effect (and (increase (total-cost) 5) (not (road_isunknown road7)) (probabilistic
    200/1000 (and (road_isblocked road7))
    800/1000 (and (road_isfree road7) (not (at ?t market6)) (at ?t market7)))))

(:action drive-m6-m7
    :parameters (?t - truck)
    :precondition (and (road_isfree road7) (at ?t market6))
    :effect (and (increase (total-cost) 5) (not (at ?t market6)) (at ?t market7)))

(:action try-drive-m7-m6
    :parameters (?t - truck)
    :precondition (and (road_isunknown road7) (at ?t market7))
    :effect (and (increase (total-cost) 5) (not (road_isunknown road7)) (probabilistic
    200/1000 (and (road_isblocked road7))
    800/1000 (and (road_isfree road7) (not (at ?t market7)) (at ?t market6)))))

(:action drive-m7-m6
    :parameters (?t - truck)
    :precondition (and (road_isfree road7) (at ?t market7))
    :effect (and (increase (total-cost) 5) (not (at ?t market7)) (at ?t market6)))

)

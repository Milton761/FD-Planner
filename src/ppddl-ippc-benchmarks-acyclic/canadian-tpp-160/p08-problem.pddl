(define (problem tpp-m7-g4-s773243)
(:domain dom--tpp-m7-g4-s773243)
(:objects
		truck0 - truck
	goods0 goods1 goods2 goods3 - goods
)
(:init
	(= (total-cost) 0)
	(road_isunknown road0)
	(road_isunknown road1)
	(road_isunknown road2)
	(road_isunknown road3)
	(road_isunknown road4)
	(road_isunknown road5)
	(road_isunknown road6)
	(road_isunknown road7)
	(on-sale goods0 market3)
	(= (price goods0 market3 ) 16)
	(on-sale goods1 market3)
	(= (price goods1 market3 ) 3)
	(on-sale goods1 market5)
	(= (price goods1 market5 ) 9)
	(on-sale goods1 market6)
	(= (price goods1 market6 ) 14)
	(on-sale goods1 market7)
	(= (price goods1 market7 ) 17)
	(on-sale goods2 market7)
	(= (price goods2 market7 ) 3)
	(on-sale goods3 market2)
	(= (price goods3 market2 ) 4)
	(on-sale goods3 market3)
	(= (price goods3 market3 ) 3)
	(on-sale goods3 market7)
	(= (price goods3 market7 ) 22)
	(at truck0 depot0)
)
(:goal (and
	(stored goods0)
	(stored goods1)
	(stored goods2)
	(stored goods3)
))
(:metric minimize (total-cost))
)

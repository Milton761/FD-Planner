(define (problem tpp-m4-g4-s487533)
(:domain dom--tpp-m4-g4-s487533)
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
	(on-sale goods0 market2)
	(= (price goods0 market2 ) 21)
	(on-sale goods1 market4)
	(= (price goods1 market4 ) 10)
	(on-sale goods2 market1)
	(= (price goods2 market1 ) 14)
	(on-sale goods2 market4)
	(= (price goods2 market4 ) 8)
	(on-sale goods3 market4)
	(= (price goods3 market4 ) 20)
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
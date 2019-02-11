

(
	define (domain)
	( :requirements :typing :strips :equlaity :probabilistic-effects)
	( :types location)
	( :predicates
		(robot-at ?loc - location)
		(path ?from - location ?to - location)
		(robot-dead)

	)
)

(:functions (total-cost))

(:action move-south
	predcondition ()
)

(:action move-north

)

(:action move-west

)

(:action move-east

)
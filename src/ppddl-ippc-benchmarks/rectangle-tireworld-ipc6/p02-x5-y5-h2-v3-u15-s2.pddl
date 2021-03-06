(define (problem rect-5-5-2-3-2)
  (:domain rectangle-world)
  (:objects n0 n1 n2 n3 n4 - int)
  (:init (= (total-cost) 0) (xpos n0)
	 (ypos n0)
         (next n0 n1)
         (next n1 n2)
         (next n2 n3)
         (next n3 n4)
	 (safeX n0)
	 (safeX n4)
	 (safeY n0)
	 (safeY n1)
	 (safeY n3)
	 (unsafe n0 n1)
	 (unsafe n0 n2)
	 (unsafe n1 n0)
	 (unsafe n1 n2)
	 (unsafe n2 n0)
	 (unsafe n2 n1)
	 (unsafe n2 n2)
	 (unsafe n2 n4)
	 (unsafe n3 n2)
	 (unsafe n3 n3)
	 (unsafe n3 n4)
	 (unsafe n4 n4)
  )
  (:goal (and (not (dead)) (xpos n4) (ypos n4)))
  (:metric minimize (total-cost))
)

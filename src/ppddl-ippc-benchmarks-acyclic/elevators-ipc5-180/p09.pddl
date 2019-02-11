(define (problem elev_3_8_2_6_1096)
  (:domain elevators)
  (:objects f2 f3 - floor p2 p3 p4 p5 p6 p7 p8 - pos e1 e2 - elevator c1 c2 c3 c4 c5 c6 - coin)
  (:init (at f1 p1) (dec_f f2 f1) (dec_f f3 f2) (dec_p p2 p1) (dec_p p3 p2) (dec_p p4 p3) (dec_p p5 p4) (dec_p p6 p5) (dec_p p7 p6) (dec_p p8 p7) (shaft e1 p3) (in e1 f3) (shaft e2 p7) (in e2 f1) (coin-at c1 f2 p7) (coin-at c2 f3 p3) (coin-at c3 f1 p1) (coin-at c4 f3 p8) (coin-at c5 f2 p1) (coin-at c6 f3 p3) (gate f2 p5) (gate f2 p8) (gate f3 p6))
  (:goal (and (have c1) (have c2) (have c3) (have c4) (have c5) (have c6)))

(:metric minimize (total-cost))
)

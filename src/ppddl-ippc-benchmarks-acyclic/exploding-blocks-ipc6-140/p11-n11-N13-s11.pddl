(define (problem ex_bw_13_p11)
  (:domain exploding-blocksworld)
  (:objects b1 b2 b3 b4 b5 b6 b7 b8 b9 b10 b11 b12 b13 - block)
  (:init (= (total-cost) 0)  (emptyhand) (on-table b1) (on b2 b12) (on b3 b13) (on b4 b5) (on b5 b7) (on b6 b11) (on-table b7) (on-table b8) (on b9 b2) (on b10 b9) (on-table b11) (on b12 b4) (on b13 b8) (clear b1) (clear b3) (clear b6) (clear b10) (no-detonated b1) (no-destroyed b1) (no-detonated b2) (no-destroyed b2) (no-detonated b3) (no-destroyed b3) (no-detonated b4) (no-destroyed b4) (no-detonated b5) (no-destroyed b5) (no-detonated b6) (no-destroyed b6) (no-detonated b7) (no-destroyed b7) (no-detonated b8) (no-destroyed b8) (no-detonated b9) (no-destroyed b9) (no-detonated b10) (no-destroyed b10) (no-detonated b11) (no-destroyed b11) (no-detonated b12) (no-destroyed b12) (no-detonated b13) (no-destroyed b13) (no-destroyed-table))
  (:goal (and  (on b2 b3) (on b3 b6) (on b4 b1) (on-table b5) (on-table b6) (on b7 b8) (on b8 b10) (on b9 b12) (on b10 b11) (on b11 b9) (on b12 b4)  )
)
  (:metric minimize (total-cost))
)

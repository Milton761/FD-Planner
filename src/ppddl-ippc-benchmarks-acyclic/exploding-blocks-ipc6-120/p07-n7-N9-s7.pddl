(define (problem ex_bw_9_p07)
  (:domain exploding-blocksworld)
  (:objects b1 b2 b3 b4 b5 b6 b7 b8 b9 - block)
  (:init (= (total-cost) 0)  (emptyhand) (on-table b1) (on b2 b3) (on-table b3) (on b4 b9) (on b5 b6) (on-table b6) (on-table b7) (on b8 b4) (on-table b9) (clear b1) (clear b2) (clear b5) (clear b7) (clear b8) (no-detonated b1) (no-destroyed b1) (no-detonated b2) (no-destroyed b2) (no-detonated b3) (no-destroyed b3) (no-detonated b4) (no-destroyed b4) (no-detonated b5) (no-destroyed b5) (no-detonated b6) (no-destroyed b6) (no-detonated b7) (no-destroyed b7) (no-detonated b8) (no-destroyed b8) (no-detonated b9) (no-destroyed b9) (no-destroyed-table))
  (:goal (and  (on-table b1) (on-table b2) (on b5 b4) (on b6 b1) (on b7 b8) (on b8 b5) (on-table b9)  )
)
  (:metric minimize (total-cost))
)

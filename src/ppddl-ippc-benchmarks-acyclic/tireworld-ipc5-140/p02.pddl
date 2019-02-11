(define (problem tire_19_0_28845)
  (:domain tire)
  (:objects n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 - location)
  (:init (vehicle-at n12)
         (road n0 n8) (road n8 n0)
         (road n1 n2) (road n2 n1)
         (road n1 n3) (road n3 n1)
         (road n1 n6) (road n6 n1)
         (road n1 n7) (road n7 n1)
         (road n1 n11) (road n11 n1)
         (road n1 n13) (road n13 n1)
         (road n2 n18) (road n18 n2)
         (road n3 n7) (road n7 n3)
         (road n3 n9) (road n9 n3)
         (road n3 n12) (road n12 n3)
         (road n3 n18) (road n18 n3)
         (road n4 n9) (road n9 n4)
         (road n5 n7) (road n7 n5)
         (road n6 n8) (road n8 n6)
         (road n6 n17) (road n17 n6)
         (road n7 n13) (road n13 n7)
         (road n7 n16) (road n16 n7)
         (road n8 n10) (road n10 n8)
         (road n8 n15) (road n15 n8)
         (road n8 n17) (road n17 n8)
         (road n8 n18) (road n18 n8)
         (road n9 n15) (road n15 n9)
         (road n9 n16) (road n16 n9)
         (road n10 n12) (road n12 n10)
         (road n10 n16) (road n16 n10)
         (road n11 n13) (road n13 n11)
         (road n12 n15) (road n15 n12)
         (road n12 n17) (road n17 n12)
         (road n12 n18) (road n18 n12)
         (road n13 n14) (road n14 n13)
         (road n13 n16) (road n16 n13)
         (road n13 n18) (road n18 n13)
         (road n14 n15) (road n15 n14)
         (road n14 n16) (road n16 n14)
         (road n14 n17) (road n17 n14)
         (road n15 n17) (road n17 n15)
         (road n16 n17) (road n17 n16)
         (road n16 n18) (road n18 n16)
         (road n17 n18) (road n18 n17)
         (spare-in n4)
         (spare-in n5)
         (spare-in n6)
         (spare-in n9)
         (spare-in n10)
         (spare-in n11)
         (spare-in n12)
         (spare-in n13)
         (spare-in n17)
         (spare-in n18)
         (not-flattire)
  )
  (:goal (vehicle-at n3))
  (:metric minimize (total-cost))
)

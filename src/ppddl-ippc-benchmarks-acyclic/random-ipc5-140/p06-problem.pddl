(define (problem random-problem598) 
 (:domain prob_domain) 
 (:init 
(not-clear)
(PZ HT) (PZ JO) (ZE VF PL) (BD HT VF) (PZ SV) (FZ JO ZT) (PZ VF) (OL JO) (FZ SV VF) (OL VF) (OL NT) (BD NT PL) (BD NT VO) (BD ZT SV) (ZE SD VO) (BD ZT JO) (FZ ZT NT) (BD SD VF) (FZ ZT PL) (ZE SV PL)  
)
 (:goal (and 
(BD PL  JO ) 
(BD VO  NT ) 
(BD SV  VF ) 
(BD EU  VF ) 
(BD VF  JO ) 
(BD EU  NT ) 
(BD EU  JO ) 
(BD SD  SD ) 
(BD HT  NT ) 
(BD VO  VF ) 
(BD EU  EU ) 
(BD PL  VF ) 
(BD VF  VF ) 
(BD PL  PL ) 
(BD SD  JO ) 
(BD SV  NT ) 
))
(:metric minimize (total-cost))
)

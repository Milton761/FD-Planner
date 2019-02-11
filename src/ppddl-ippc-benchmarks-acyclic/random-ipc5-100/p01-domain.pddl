(define (domain prob_domain) 
 (:requirements :strips :probabilistic-effects :conditional-effects) 
 (:constants PO OZ EF ZO JU AW XQ VF FB RA )
 (:predicates 
	 (IU ?X ?Y ) 
	 (KI ?X ) 
	 (NH ?X ?Y ) 
	 (UV ?X ) 
	 (OF ?X ?Y ) 
(clear)
(not-clear)
 )
(:functions (total-cost))
(:action CMW
 :parameters (?X ?Y )
 :precondition (and 
		 (OF ?X ?X) 
  )
 :effect (and (increase (total-cost) 1) (probabilistic 
		 51/100 (and (not (OF ?X ?X)) (UV ?X) (UV ?Y) )  
		 22/100 (and (NH ?X ?X) )  
		 27/100 (and (OF ?X ?Y) (KI ?X) )  
          ))
 )
(:action EOR
 :parameters (?X ?Y ?Z )
 :precondition (and 
		 (KI ?X) 
		 (UV ?Z) 
		 (KI ?Z) 
  )
 :effect (and (increase (total-cost) 1) (probabilistic 
		 91/100 (and (UV ?X) (OF ?Y ?X) )  
		 6/100 (and (UV ?Y) (IU ?Y ?Z) (NH ?Z ?Z) )  
		 3/100 (and (OF ?Y ?Y) (not (KI ?X)) (IU ?Z ?Y) )  
          ))
 )
(:action XIJ
 :parameters (?X ?Y ?Z )
 :precondition (and 
		 (IU ?X ?Z) 
		 (IU ?Y ?Y) 
		 (UV ?Y) 
  )
 :effect (and (increase (total-cost) 1) (probabilistic 
		 100/100 (and (not (UV ?Y)) (OF ?Z ?X) (NH ?X ?X) )  
          ))
 )
(:action KWM
 :parameters (?X ?Y )
 :precondition (and 
		 (KI ?Y) 
  )
 :effect (and (increase (total-cost) 1) (probabilistic 
		 37/100 (and (KI ?X) )  
		 63/100 (and (not (KI ?Y)) )  
          ))
 )
(:action KYL
 :parameters (?X ?Y )
 :precondition (and 
		 (OF ?X ?Y) 
  )
 :effect (and (increase (total-cost) 1) (probabilistic 
		 100/100 (and (UV ?Y) (NH ?X ?Y) )  
          ))
 )
(:action reset1 
 :precondition (not-clear)
 :effect (and (increase (total-cost) 1)  
	     (forall (?x) (and 
      (not (IU ?x PO)) 
      (not (IU ?x OZ)) 
      (not (IU ?x EF)) 
      (not (IU ?x ZO)) 
      (not (IU ?x JU)) 
      (not (IU ?x AW)) 
      (not (IU ?x XQ)) 
      (not (IU ?x VF)) 
      (not (IU ?x FB)) 
      (not (IU ?x RA)) 
      (not (KI ?x)) 
      (not (NH ?x PO)) 
      (not (NH ?x OZ)) 
      (not (NH ?x EF)) 
      (not (NH ?x ZO)) 
      (not (NH ?x JU)) 
      (not (NH ?x AW)) 
      (not (NH ?x XQ)) 
      (not (NH ?x VF)) 
      (not (NH ?x FB)) 
      (not (NH ?x RA)) 
      (not (UV ?x)) 
      (not (OF ?x PO)) 
      (not (OF ?x OZ)) 
      (not (OF ?x EF)) 
      (not (OF ?x ZO)) 
      (not (OF ?x JU)) 
      (not (OF ?x AW)) 
      (not (OF ?x XQ)) 
      (not (OF ?x VF)) 
      (not (OF ?x FB)) 
      (not (OF ?x RA)) 
))
(not (not-clear))
(clear)))

(:action reset2 
 :precondition (clear) 
 :effect (and (increase (total-cost) 1)  (not-clear)
              (not (clear))
(OF OZ PO) 
(OF XQ EF) 
(OF OZ OZ) 
(OF PO EF) 
(OF FB AW) 
(IU ZO EF) 
(NH AW JU) 
(KI RA) 
(NH JU EF) 
(IU OZ XQ) 
(UV AW) 
(IU OZ VF) 
(OF OZ EF) 
(NH EF EF) 
(IU JU PO) 
(NH OZ ZO) 
(NH ZO FB) 
(KI EF) 
(NH XQ PO) 
(IU XQ VF) 
)))

begin_version
3
end_version
begin_metric
1
end_metric
11
begin_variable
var0
-1
2
Atom hasspare()
NegatedAtom hasspare()
end_variable
begin_variable
var1
-1
2
Atom in-car()
NegatedAtom in-car()
end_variable
begin_variable
var2
-1
2
Atom not-flattire()
NegatedAtom not-flattire()
end_variable
begin_variable
var3
-1
2
Atom spare-in(n10)
NegatedAtom spare-in(n10)
end_variable
begin_variable
var4
-1
2
Atom spare-in(n12)
NegatedAtom spare-in(n12)
end_variable
begin_variable
var5
-1
2
Atom spare-in(n16)
NegatedAtom spare-in(n16)
end_variable
begin_variable
var6
-1
2
Atom spare-in(n4)
NegatedAtom spare-in(n4)
end_variable
begin_variable
var7
-1
2
Atom spare-in(n5)
NegatedAtom spare-in(n5)
end_variable
begin_variable
var8
-1
2
Atom spare-in(n7)
NegatedAtom spare-in(n7)
end_variable
begin_variable
var9
-1
2
Atom spare-in(n8)
NegatedAtom spare-in(n8)
end_variable
begin_variable
var10
-1
17
Atom vehicle-at(n0)
Atom vehicle-at(n1)
Atom vehicle-at(n10)
Atom vehicle-at(n11)
Atom vehicle-at(n12)
Atom vehicle-at(n13)
Atom vehicle-at(n14)
Atom vehicle-at(n15)
Atom vehicle-at(n16)
Atom vehicle-at(n2)
Atom vehicle-at(n3)
Atom vehicle-at(n4)
Atom vehicle-at(n5)
Atom vehicle-at(n6)
Atom vehicle-at(n7)
Atom vehicle-at(n8)
Atom vehicle-at(n9)
end_variable
1
begin_mutex_group
17
10 0
10 1
10 2
10 3
10 4
10 5
10 6
10 7
10 8
10 9
10 10
10 11
10 12
10 13
10 14
10 15
10 16
end_mutex_group
begin_state
1
1
0
0
0
0
0
0
0
0
9
end_state
begin_goal
1
10 0
end_goal
98
begin_operator
changetire_DETDUP_0_WEIGHT_1_2 
1
1 1
2
0 0 0 1
0 2 -1 0
1
end_operator
begin_operator
loadtire n10
1
10 2
2
0 0 -1 0
0 3 0 1
1
end_operator
begin_operator
loadtire n12
1
10 4
2
0 0 -1 0
0 4 0 1
1
end_operator
begin_operator
loadtire n16
1
10 8
2
0 0 -1 0
0 5 0 1
1
end_operator
begin_operator
loadtire n4
1
10 11
2
0 0 -1 0
0 6 0 1
1
end_operator
begin_operator
loadtire n5
1
10 12
2
0 0 -1 0
0 7 0 1
1
end_operator
begin_operator
loadtire n7
1
10 14
2
0 0 -1 0
0 8 0 1
1
end_operator
begin_operator
loadtire n8
1
10 15
2
0 0 -1 0
0 9 0 1
1
end_operator
begin_operator
move-agent-in 
0
1
0 1 1 0
0
end_operator
begin_operator
move-agent-out 
0
1
0 1 0 1
0
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n0 n12
1
1 0
2
0 2 0 1
0 10 0 4
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n0 n16
1
1 0
2
0 2 0 1
0 10 0 8
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n1 n2
1
1 0
2
0 2 0 1
0 10 1 9
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n1 n3
1
1 0
2
0 2 0 1
0 10 1 10
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n10 n12
1
1 0
2
0 2 0 1
0 10 2 4
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n10 n13
1
1 0
2
0 2 0 1
0 10 2 5
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n10 n5
1
1 0
2
0 2 0 1
0 10 2 12
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n11 n16
1
1 0
2
0 2 0 1
0 10 3 8
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n12 n0
1
1 0
2
0 2 0 1
0 10 4 0
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n12 n10
1
1 0
2
0 2 0 1
0 10 4 2
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n12 n16
1
1 0
2
0 2 0 1
0 10 4 8
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n12 n9
1
1 0
2
0 2 0 1
0 10 4 16
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n13 n10
1
1 0
2
0 2 0 1
0 10 5 2
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n13 n15
1
1 0
2
0 2 0 1
0 10 5 7
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n13 n3
1
1 0
2
0 2 0 1
0 10 5 10
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n13 n7
1
1 0
2
0 2 0 1
0 10 5 14
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n14 n16
1
1 0
2
0 2 0 1
0 10 6 8
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n14 n3
1
1 0
2
0 2 0 1
0 10 6 10
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n14 n6
1
1 0
2
0 2 0 1
0 10 6 13
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n15 n13
1
1 0
2
0 2 0 1
0 10 7 5
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n16 n0
1
1 0
2
0 2 0 1
0 10 8 0
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n16 n11
1
1 0
2
0 2 0 1
0 10 8 3
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n16 n12
1
1 0
2
0 2 0 1
0 10 8 4
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n16 n14
1
1 0
2
0 2 0 1
0 10 8 6
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n16 n5
1
1 0
2
0 2 0 1
0 10 8 12
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n16 n9
1
1 0
2
0 2 0 1
0 10 8 16
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n2 n1
1
1 0
2
0 2 0 1
0 10 9 1
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n3 n1
1
1 0
2
0 2 0 1
0 10 10 1
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n3 n13
1
1 0
2
0 2 0 1
0 10 10 5
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n3 n14
1
1 0
2
0 2 0 1
0 10 10 6
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n3 n4
1
1 0
2
0 2 0 1
0 10 10 11
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n4 n3
1
1 0
2
0 2 0 1
0 10 11 10
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n5 n10
1
1 0
2
0 2 0 1
0 10 12 2
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n5 n16
1
1 0
2
0 2 0 1
0 10 12 8
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n5 n8
1
1 0
2
0 2 0 1
0 10 12 15
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n6 n14
1
1 0
2
0 2 0 1
0 10 13 6
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n7 n13
1
1 0
2
0 2 0 1
0 10 14 5
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n7 n9
1
1 0
2
0 2 0 1
0 10 14 16
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n8 n5
1
1 0
2
0 2 0 1
0 10 15 12
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n8 n9
1
1 0
2
0 2 0 1
0 10 15 16
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n9 n12
1
1 0
2
0 2 0 1
0 10 16 4
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n9 n16
1
1 0
2
0 2 0 1
0 10 16 8
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n9 n7
1
1 0
2
0 2 0 1
0 10 16 14
1
end_operator
begin_operator
move-car_DETDUP_0_WEIGHT_2_5 n9 n8
1
1 0
2
0 2 0 1
0 10 16 15
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n0 n12
2
1 0
2 0
1
0 10 0 4
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n0 n16
2
1 0
2 0
1
0 10 0 8
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n1 n2
2
1 0
2 0
1
0 10 1 9
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n1 n3
2
1 0
2 0
1
0 10 1 10
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n10 n12
2
1 0
2 0
1
0 10 2 4
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n10 n13
2
1 0
2 0
1
0 10 2 5
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n10 n5
2
1 0
2 0
1
0 10 2 12
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n11 n16
2
1 0
2 0
1
0 10 3 8
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n12 n0
2
1 0
2 0
1
0 10 4 0
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n12 n10
2
1 0
2 0
1
0 10 4 2
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n12 n16
2
1 0
2 0
1
0 10 4 8
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n12 n9
2
1 0
2 0
1
0 10 4 16
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n13 n10
2
1 0
2 0
1
0 10 5 2
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n13 n15
2
1 0
2 0
1
0 10 5 7
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n13 n3
2
1 0
2 0
1
0 10 5 10
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n13 n7
2
1 0
2 0
1
0 10 5 14
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n14 n16
2
1 0
2 0
1
0 10 6 8
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n14 n3
2
1 0
2 0
1
0 10 6 10
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n14 n6
2
1 0
2 0
1
0 10 6 13
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n15 n13
2
1 0
2 0
1
0 10 7 5
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n16 n0
2
1 0
2 0
1
0 10 8 0
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n16 n11
2
1 0
2 0
1
0 10 8 3
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n16 n12
2
1 0
2 0
1
0 10 8 4
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n16 n14
2
1 0
2 0
1
0 10 8 6
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n16 n5
2
1 0
2 0
1
0 10 8 12
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n16 n9
2
1 0
2 0
1
0 10 8 16
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n2 n1
2
1 0
2 0
1
0 10 9 1
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n3 n1
2
1 0
2 0
1
0 10 10 1
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n3 n13
2
1 0
2 0
1
0 10 10 5
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n3 n14
2
1 0
2 0
1
0 10 10 6
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n3 n4
2
1 0
2 0
1
0 10 10 11
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n4 n3
2
1 0
2 0
1
0 10 11 10
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n5 n10
2
1 0
2 0
1
0 10 12 2
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n5 n16
2
1 0
2 0
1
0 10 12 8
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n5 n8
2
1 0
2 0
1
0 10 12 15
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n6 n14
2
1 0
2 0
1
0 10 13 6
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n7 n13
2
1 0
2 0
1
0 10 14 5
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n7 n9
2
1 0
2 0
1
0 10 14 16
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n8 n5
2
1 0
2 0
1
0 10 15 12
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n8 n9
2
1 0
2 0
1
0 10 15 16
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n9 n12
2
1 0
2 0
1
0 10 16 4
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n9 n16
2
1 0
2 0
1
0 10 16 8
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n9 n7
2
1 0
2 0
1
0 10 16 14
1
end_operator
begin_operator
move-car_DETDUP_1_WEIGHT_3_5 n9 n8
2
1 0
2 0
1
0 10 16 15
1
end_operator
0

#!/usr/bin/env gnuplot

set terminal pdf
set output 'time.pdf'

set xtics rotate # crucial line

set key horizontal
set key bottom
set key outside
  

set title "Domain Name"
set grid y 

set key spacing 3

set xlabel "Problem #" 
set ylabel "Time (seconds)"


plot "tvi.dat" 					using 4:xtic(1) t "tvi" 						with linespoints pt 2  ps 0.6, \
 	 "fretv-lrtdp.dat" 			using 4:xtic(1) t "fretv-lrtdp" 				with linespoints pt 12 ps 0.8, \
 	 "fretv-lrtdp-hmax.dat" 	using 4:xtic(1) t "fretv-lrtdp-hmax" 			with linespoints pt 13 ps 0.8, \
 	 "fretv-lrtdp-hpom.dat" 	using 4:xtic(1) t "fretv-lrtdp-hpom"	 		with linespoints pt 14 ps 0.8



set terminal pdf
set output 'states.pdf'

set xtics rotate # crucial line

set key horizontal
set key bottom
set key outside
  

set title "Problem vs states"
set grid y 
set key spacing 3

set ylabel "# states"


plot "tvi.dat" 					using 2:xtic(1) t "tvi" 							with linespoints pt 2  ps 0.6, \
	 "fretv-lrtdp.dat" 			using 2:xtic(1) t "fretv-lrtdp" 				with linespoints pt 12 ps 0.8, \
 	 "fretv-lrtdp-hmax.dat" 	using 2:xtic(1) t "fretv-lrtdp-hmax" 			with linespoints pt 13 ps 0.8, \
 	 "fretv-lrtdp-hpom.dat" 	using 2:xtic(1) t "fretv-lrtdp-hpom"	 		with linespoints pt 12 ps 0.8
 	 



set terminal pdf
set output 'maxprob.pdf'

set xtics rotate # crucial line

set key horizontal
set key bottom
set key outside
  

set title "Problem vs MaxProb"
set grid y 
set key spacing 3

set ylabel "Probability"
set xrange [1:15]


plot "tvi.dat" 					using 3:xtic(1) t "tvi" 							with linespoints pt 2  ps 0.6, \
	 "fretv-lrtdp.dat" 			using 3:xtic(1) t "fretv-lrtdp" 				with linespoints pt 12 ps 0.8, \
 	 "fretv-lrtdp-hmax.dat" 	using 3:xtic(1) t "fretv-lrtdp-hmax" 			with linespoints pt 13 ps 0.8, \
 	 "fretv-lrtdp-hpom.dat" 	using 3:xtic(1) t "fretv-lrtdp-hpom"	 		with linespoints pt 12 ps 0.8
 	 

set terminal pdf
set output 'memory.pdf'

set xtics rotate # crucial line

set key horizontal
set key bottom
set key outside
  

set title "Problem vs memory"
set grid y 
set log y 10

set key spacing 3

set ylabel "memory (KB)"


plot "tvi.dat" 					using 5:xtic(1) t "tvi" 							with linespoints pt 2  ps 0.6, \
	 "fretv-lrtdp.dat" 			using 5:xtic(1) t "fretv-lrtdp" 				with linespoints pt 12 ps 0.8, \
 	 "fretv-lrtdp-hmax.dat" 	using 5:xtic(1) t "fretv-lrtdp-hmax" 			with linespoints pt 13 ps 0.8, \
 	 "fretv-lrtdp-hpom.dat" 	using 5:xtic(1) t "fretv-lrtdp-hpom"	 		with linespoints pt 12 ps 0.8
 	 

set terminal pdf
set output 'updates.pdf'

set xtics rotate # crucial line

set key horizontal
set key bottom
set key outside
  

set title "Problem vs updates"
set grid y 
set log y 10	

set key spacing 3

set ylabel "#"


plot "tvi.dat" 					using 6:xtic(1) t "tvi" 							with linespoints pt 2  ps 0.6, \
	 "fretv-lrtdp.dat" 			using 6:xtic(1) t "fretv-lrtdp" 				with linespoints pt 12 ps 0.8, \
 	 "fretv-lrtdp-hmax.dat" 	using 6:xtic(1) t "fretv-lrtdp-hmax" 			with linespoints pt 13 ps 0.8, \
 	 "fretv-lrtdp-hpom.dat" 	using 6:xtic(1) t "fretv-lrtdp-hpom"	 		with linespoints pt 12 ps 0.8
 	 

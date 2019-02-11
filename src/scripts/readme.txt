to run a domain


At terminal:



python2 ../fast-downward.py --alias [arg1] --search-time-limit [arg2] --log-level warning [arg3] --criterion [arg4] >> output.dat

where: 

arg1 :

vi
vi-hmax
vi-ms100k
vi-msinf

fretv-lrtdp
fretv-lrtdp-hmax
fretv-lrtdp-ms100k
fretv-lrtdp-msinf

//for SSPs:
lrtdp
lrtdp-lmcut
lrtdp-ms100k
lrtdp-msinf

arg 2:
time in secs e.g : 300

arg3:
path of problem 

arg4:

maxprob
expcost


>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
to compile:

g++ -std=c++11 get_data.cpp -o get_data

to parse the output in output.dat

In terminal execute:
./get_data output.dat >> output_parse.dat

open output_parse

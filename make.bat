clear 
@echo off
rem del dsp1.o
rem del sample1.o

rem rmac -fb -u dsp1.s
rem rln -o dsp1.abs -w -rq -a 4000 x x dsp1.o

rem rmac -fb -u "dsp1 - replay sample au timer 1.s"
rem rln -o sample1.abs -w -rq -a 4000 x x "dsp1 - replay sample au timer 1.o"

@echo on

del ym1.o
rmac -fb -s -u ym1.s 
rln -o ym1.abs -w -rq -a 4000 x x ym1.o -m 

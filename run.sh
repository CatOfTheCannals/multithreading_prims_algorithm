#!/usr/bin/env bash

for i in {1..3000}
do 
	./TP1 -t ./test/simple.txt 6
	echo Corrida $i 
	echo 
done 
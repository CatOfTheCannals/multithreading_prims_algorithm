#!/usr/bin/env bash

for i in {1..1000}
do 
	./TP1 -t ./test/simple.txt 3
	echo Corrida $i 
	echo 
done 
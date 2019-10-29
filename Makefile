.PHONY: all clean

CC=g++
CPP_STANDARD=c++11
CFLAGS=-c -pthread -std=$(CPP_STANDARD)
LFLAGS=-pthread -std=$(CPP_STANDARD)

OBJS=grafo.o

all: TP1

TP1: $(OBJS) tp1_grafo.cpp
	$(CC) $(LFLAGS) $(OBJS) tp1_grafo.cpp -g -no-pie -o TP1

grafo.o: grafo.h grafo.cpp
	$(CC) $(CFLAGS) grafo.cpp -o grafo.o

clean:
	rm $(OBJS)
	rm TP1

test: TP1
	./TP1 $(input)

testParaleloVacio: TP1
	./TP1 test/vacio.txt paralelo $(threads)

testParaleloTrivial: TP1
	./TP1 test/trivial.txt paralelo $(threads)

testParaleloSimple: TP1
	./TP1 test/simple.txt paralelo $(threads)

testParaleloArbol1000: TP1
	./TP1 test/arbol1000.txt paralelo $(threads)

testParaleloRandom1000: TP1
	./TP1 test/random1000.txt paralelo $(threads)

testParaleloCompleto1000: TP1
	./TP1 test/completo1000.txt paralelo $(threads)

testParaleloCompleto2000: TP1
	./TP1 test/completo2000.txt paralelo $(threads)

testSecuencialVacio: TP1
	./TP1 test/vacio.txt secuencial

testSecuencialTrivial: TP1
	./TP1 test/trivial.txt secuencial

testSecuencialSimple: TP1
	./TP1 test/simple.txt secuencial

testSecuencialArbol1000: TP1
	./TP1 test/arbol1000.txt secuencial

testSecuencialRandom1000: TP1
	./TP1 test/random1000.txt secuencial

testSecuencialCompleto1000: TP1
	./TP1 test/completo1000.txt secuencial

testSecuencialCompleto2000: TP1
	./TP1 test/completo2000.txt secuencial

testSecuencialEstrella100: TP1
	./TP1 test/estrella/estrella100.txt secuencial

testSecuencialEstrella200: TP1
	./TP1 test/estrella/estrella200.txt secuencial

testSecuencialEstrella300: TP1
	./TP1 test/estrella/estrella300.txt secuencial

testSecuencialEstrella400: TP1
	./TP1 test/estrella/estrella400.txt secuencial

testSecuencialEstrella500: TP1
	./TP1 test/estrella/estrella500.txt secuencial

testSecuencialEstrella600: TP1
	./TP1 test/estrella/estrella600.txt secuencial

testSecuencialEstrella700: TP1
	./TP1 test/estrella/estrella700.txt secuencial

testSecuencialEstrella800: TP1
	./TP1 test/estrella/estrella800.txt secuencial

testSecuencialEstrella900: TP1
	./TP1 test/estrella/estrella900.txt secuencial

testSecuencialEstrella1000: TP1
	./TP1 test/estrella/estrella1000.txt secuencial

testSecuencialListaEnlazada100: TP1
	./TP1 test/listaenlazada/listaenlazada100.txt secuencial

testSecuencialListaEnlazada200: TP1
	./TP1 test/listaenlazada/listaenlazada200.txt secuencial

testSecuencialListaEnlazada300: TP1
	./TP1 test/listaenlazada/listaenlazada300.txt secuencial

testSecuencialListaEnlazada400: TP1
	./TP1 test/listaenlazada/listaenlazada400.txt secuencial

testSecuencialListaEnlazada500: TP1
	./TP1 test/listaenlazada/listaenlazada500.txt secuencial

testSecuencialListaEnlazada600: TP1
	./TP1 test/listaenlazada/listaenlazada600.txt secuencial

testSecuencialListaEnlazada700: TP1
	./TP1 test/listaenlazada/listaenlazada700.txt secuencial

testSecuencialListaEnlazada800: TP1
	./TP1 test/listaenlazada/listaenlazada800.txt secuencial

testSecuencialListaEnlazada900: TP1
	./TP1 test/listaenlazada/listaenlazada900.txt secuencial

testSecuencialListaEnlazada1000: TP1
	./TP1 test/listaenlazada/listaenlazada1000.txt secuencial

testSecuencialEstrella100: TP1
	./TP1 test/estrella/estrella100.txt secuencial

testSecuencialEstrella200: TP1
	./TP1 test/estrella/estrella200.txt secuencial

testSecuencialEstrella300: TP1
	./TP1 test/estrella/estrella300.txt secuencial

testSecuencialEstrella400: TP1
	./TP1 test/estrella/estrella400.txt secuencial

testSecuencialEstrella500: TP1
	./TP1 test/estrella/estrella500.txt secuencial

testSecuencialEstrella600: TP1
	./TP1 test/estrella/estrella600.txt secuencial

testSecuencialEstrella700: TP1
	./TP1 test/estrella/estrella700.txt secuencial

testSecuencialEstrella800: TP1
	./TP1 test/estrella/estrella800.txt secuencial

testSecuencialEstrella900: TP1
	./TP1 test/estrella/estrella900.txt secuencial

testSecuencialEstrella1000: TP1
	./TP1 test/estrella/estrella1000.txt secuencial

testSecuencialListaEnlazada100: TP1
	./TP1 test/listaenlazada/listaenlazada100.txt secuencial

testSecuencialListaEnlazada200: TP1
	./TP1 test/listaenlazada/listaenlazada200.txt secuencial

testSecuencialListaEnlazada300: TP1
	./TP1 test/listaenlazada/listaenlazada300.txt secuencial

testSecuencialListaEnlazada400: TP1
	./TP1 test/listaenlazada/listaenlazada400.txt secuencial

testSecuencialListaEnlazada500: TP1
	./TP1 test/listaenlazada/listaenlazada500.txt secuencial

testSecuencialListaEnlazada600: TP1
	./TP1 test/listaenlazada/listaenlazada600.txt secuencial

testSecuencialListaEnlazada700: TP1
	./TP1 test/listaenlazada/listaenlazada700.txt secuencial

testSecuencialListaEnlazada800: TP1
	./TP1 test/listaenlazada/listaenlazada800.txt secuencial

testSecuencialListaEnlazada900: TP1
	./TP1 test/listaenlazada/listaenlazada900.txt secuencial

testSecuencialListaEnlazada1000: TP1
	./TP1 test/listaenlazada/listaenlazada1000.txt secuencial

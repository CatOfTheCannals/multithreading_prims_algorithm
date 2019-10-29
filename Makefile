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
	./TP1 test/lista_enlazada/lista_enlazada100.txt secuencial

testSecuencialListaEnlazada200: TP1
	./TP1 test/lista_enlazada/lista_enlazada200.txt secuencial

testSecuencialListaEnlazada300: TP1
	./TP1 test/lista_enlazada/lista_enlazada300.txt secuencial

testSecuencialListaEnlazada400: TP1
	./TP1 test/lista_enlazada/lista_enlazada400.txt secuencial

testSecuencialListaEnlazada500: TP1
	./TP1 test/lista_enlazada/lista_enlazada500.txt secuencial

testSecuencialListaEnlazada600: TP1
	./TP1 test/lista_enlazada/lista_enlazada600.txt secuencial

testSecuencialListaEnlazada700: TP1
	./TP1 test/lista_enlazada/lista_enlazada700.txt secuencial

testSecuencialListaEnlazada800: TP1
	./TP1 test/lista_enlazada/lista_enlazada800.txt secuencial

testSecuencialListaEnlazada900: TP1
	./TP1 test/lista_enlazada/lista_enlazada900.txt secuencial

testSecuencialListaEnlazada1000: TP1
	./TP1 test/lista_enlazada/lista_enlazada1000.txt secuencial

testParaleloEstrella100: TP1
	./TP1 test/estrella/estrella100.txt $(threads)

testParaleloEstrella200: TP1
	./TP1 test/estrella/estrella200.txt $(threads)

testParaleloEstrella300: TP1
	./TP1 test/estrella/estrella300.txt $(threads)

testParaleloEstrella400: TP1
	./TP1 test/estrella/estrella400.txt $(threads)

testParaleloEstrella500: TP1
	./TP1 test/estrella/estrella500.txt $(threads)

testParaleloEstrella600: TP1
	./TP1 test/estrella/estrella600.txt $(threads)

testParaleloEstrella700: TP1
	./TP1 test/estrella/estrella700.txt $(threads)

testParaleloEstrella800: TP1
	./TP1 test/estrella/estrella800.txt $(threads)

testParaleloEstrella900: TP1
	./TP1 test/estrella/estrella900.txt $(threads)

testParaleloEstrella1000: TP1
	./TP1 test/estrella/estrella1000.txt $(threads)

testParaleloListaEnlazada100: TP1
	./TP1 test/lista_enlazada/lista_enlazada100.txt $(threads)

testParaleloListaEnlazada200: TP1
	./TP1 test/lista_enlazada/lista_enlazada200.txt $(threads)

testParaleloListaEnlazada300: TP1
	./TP1 test/lista_enlazada/lista_enlazada300.txt $(threads)

testParaleloListaEnlazada400: TP1
	./TP1 test/lista_enlazada/lista_enlazada400.txt $(threads)

testParaleloListaEnlazada500: TP1
	./TP1 test/lista_enlazada/lista_enlazada500.txt $(threads)

testParaleloListaEnlazada600: TP1
	./TP1 test/lista_enlazada/lista_enlazada600.txt $(threads)

testParaleloListaEnlazada700: TP1
	./TP1 test/lista_enlazada/lista_enlazada700.txt $(threads)

testParaleloListaEnlazada800: TP1
	./TP1 test/lista_enlazada/lista_enlazada800.txt $(threads)

testParaleloListaEnlazada900: TP1
	./TP1 test/lista_enlazada/lista_enlazada900.txt $(threads)

testParaleloListaEnlazada1000: TP1
	./TP1 test/lista_enlazada/lista_enlazada1000.txt $(threads)

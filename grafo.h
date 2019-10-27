#ifndef GRAFO_H_
#define GRAFO_H_

#include <iostream>
#include <vector>
#include <map>
#include <atomic>

#define BLANCO -10
#define GRIS -20
#define NEGRO -30

using namespace std;

typedef struct Eje {
  int nodoOrigen;
  int nodoDestino;
  int peso;

  Eje(int nodoOrigen, int nodoDestino, int peso) {
    this->nodoOrigen = nodoOrigen;
    this->nodoDestino = nodoDestino;
    this->peso = peso;
  }
  Eje():nodoOrigen(0), nodoDestino(0), peso(0) {}
  bool operator<(Eje other){
    return peso < other.peso;
  }
  bool operator>(Eje other){
    return peso > other.peso;
  }
  bool operator==(Eje other){
    return (nodoOrigen == other.nodoOrigen && nodoDestino == other.nodoDestino && peso == other.peso);
  }
} Eje;

class Grafo {

public:
  int numVertices;
  int numEjes;
  bool _verbose = false;

  map<int,vector<Eje>> listaDeAdyacencias;
  bool _finalizo=false;
  int _cantMerges =0;

  Grafo() {
    numVertices = 0;
    numEjes = 0;
  }

  int inicializar(string nombreArchivo);
  void imprimirGrafo();
  bool esConexo();
  vector<Eje>::iterator vecinosBegin(int num);
  vector<Eje>::iterator vecinosEnd(int num);
  void insertarEje(Eje e);
  void insertarEje(int nodoA, int nodoB, int peso);
  void insertarNodo(int nodo);
  bool noEsta(int nodo);
  void limpiarAuxiliares();
  void setFinalizo(bool finalizo);
  void setCantMerges(int cantMerges);
  
private:
  void incrementarTotalNodos();
  void incrementarTotalEjes();
};

class Compare {
  public:
    bool operator() (Eje eje_1, Eje eje_2){
      return eje_1 > eje_2;
    }
};

#endif

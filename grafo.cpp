#include "grafo.h"
#include <fstream>
#include <stack>
#include <algorithm> 


int Grafo::inicializar(string nombreArchivo) {

  ifstream archivo;
  archivo.open(nombreArchivo.c_str());
  if(!archivo) {
    cerr << nombreArchivo << " no existe en el directorio" << endl;
    return 0;
  }

  // Primera línea es la cantidad de vértices
  archivo >> numVertices;
  // La segunda la cantidad de ejes
  int numEjesArchivo;
  archivo >> numEjesArchivo;

  listaDeAdyacencias = map<int,vector<Eje>>();
  int v1, v2, peso;
  // Crear los ejes entre ambos nodos
  for(int ejes = 0; ejes < numEjesArchivo; ejes++){

    if(archivo.eof()){
      cerr << "Faltan ejes en el archivo" << endl;
      return 0;
    }

    archivo >> v1 >> v2 >> peso;

    insertarEje(v1,v2,peso);
  }
  archivo.close();
  return 1;
}

vector<Eje>::iterator Grafo::vecinosBegin(int num){
  return listaDeAdyacencias[num].begin();
}

vector<Eje>::iterator Grafo::vecinosEnd(int num){
  return listaDeAdyacencias[num].end();
}

void Grafo::insertarEje(Eje e){
  listaDeAdyacencias[e.nodoOrigen].push_back(e);
  bool agregado = false;
  Eje invertido = Eje(e.nodoDestino, e.nodoOrigen, e.peso);
  for (auto it : listaDeAdyacencias[e.nodoDestino]){
    if(it == invertido){
      agregado = true;
      break;
    }
  }
  if(!agregado){
    incrementarTotalEjes();
  }
}

void Grafo::insertarEje(int nodoA, int nodoB, int peso){
    //Agrego eje de nodoA a nodoB 
    Eje ejeA(nodoA,nodoB,peso);
    listaDeAdyacencias[nodoA].push_back(ejeA);

    //Agrego eje de nodoB a nodoA
    Eje ejeB(nodoB,nodoA,peso);
    listaDeAdyacencias[nodoB].push_back(ejeB);

    incrementarTotalEjes();
}

void Grafo::insertarNodo(int nodo){
    //Creo el vector que será la lista de adyacencias del nodo
    vector<Eje> ejesN;
    //Inserto el par (nodo, lista) en el diccionario
    listaDeAdyacencias.insert({nodo, ejesN});


    incrementarTotalNodos();
}

bool Grafo::noEsta(int nodo){
  return listaDeAdyacencias.find(nodo) == listaDeAdyacencias.end();
}

void Grafo::incrementarTotalNodos(){
    numVertices += 1;
}

void Grafo::incrementarTotalEjes(){
  numEjes += 1;
}

void Grafo::imprimirGrafo(){
  if(_verbose){
    int size = listaDeAdyacencias.size();
    int peso = 0;
    for (auto v = listaDeAdyacencias.begin() ; v != listaDeAdyacencias.end(); ++v){
        for(const Eje &Eje : v->second) {
            if (Eje.nodoDestino > v->first){
                peso += Eje.peso;
            }
        }
    }

    for (auto v = listaDeAdyacencias.begin() ; v != listaDeAdyacencias.end(); ++v) {
       cout << "\t" << v->first << ": - ";
      for(const Eje &Eje : v->second) {
        cout << "(" << Eje.nodoDestino << "," << Eje.peso << ") - ";
      }
        cout << endl;
    }
      cout << "Cantidad de nodos: " << numVertices << endl;
      cout << "Cantidad de ejes: " << numEjes << endl;
      cout << "Peso del arbol: " << peso << endl;
    if(this->esConexo()){
      cout << "El grafo es conexo" << endl;
    }else{
      cout << "El grafo no es conexo" << endl;
    }
  }
  //cout << "exp_result: qty_merges = "<< _cantMerges << endl;
}

bool Grafo::esConexo(){
    if (numVertices == 0){
        return true;
    }
    stack<int> vecinos;
    map<int, int> colores;
    int nodosPintados = 0;
    for (auto v = listaDeAdyacencias.begin() ; v != listaDeAdyacencias.end(); ++v){
        colores[v->first] = BLANCO;
    }
    vecinos.push(listaDeAdyacencias.begin()->first);
    while(!vecinos.empty()){
        int actual = vecinos.top();
        vecinos.pop();
        colores[actual] = NEGRO;
        nodosPintados++;
        for (auto v = listaDeAdyacencias[actual].begin() ; v != listaDeAdyacencias[actual].end(); ++v) {
            if(colores[v->nodoDestino] == BLANCO){
                vecinos.push(v->nodoDestino);
            }
        }
    }
    return nodosPintados == numVertices;
}

void Grafo::setCantMerges(int cantMerges){
  _cantMerges=cantMerges;
}
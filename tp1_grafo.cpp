#include "grafo.h"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <limits>
#include <stack>
#include <atomic>
#include <pthread.h>
#include <semaphore.h>
#include <chrono>
#include <utility>
#include <unordered_map>
#include <list>
#include <queue>          // std::priority_queue


#include <typeinfo>


using namespace std;

// Datos compartidos entre threads
struct sharedData{
    Grafo* _g;  // Grafo a cubrir
    vector<pthread_t> _nodeColorArray;
    vector<pthread_mutex_t> _nodesMutexes;
    vector<pthread_mutex_t> _threadsMutexes;
    vector<int> _freeNodes;
    pthread_mutex_t _mapMutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t _initMutex = PTHREAD_MUTEX_INITIALIZER;
};

class Thread{
 // Estructura que debe contener los colores de los vértices (actual y vecinos). Las distancias, el árbol, y la herramientas de sincronización necesarias para evitar race conditions y deadlocks.
  public:
    Thread(){};
    Thread& operator=(Thread other);
    int buscarNodo();
    void pintarNodo(Eje eje, sharedData* shared);
    void pintarVecinos(Grafo* g, int num);
    void reiniciarThread(sharedData* shared, unordered_map<pthread_t, Thread>* threadObjects);
    void initThread(sharedData* shared, unordered_map<pthread_t, Thread>* threadObjects);
    void processThread(sharedData* shared, unordered_map<pthread_t, Thread>* threadObjects);
    void assignIdx(pthread_t threadCreationIdx);
    pthread_t getIdx();
    Eje getNextEdge();
    Grafo* getMst();
    void procesarNodo(Eje eje, sharedData* shared, unordered_map<pthread_t, Thread>* threadObjects);
    Thread tomarNodo(int nodo);
    void requestMerge(Thread* other, int source_node, int dest_node);
    void merge(Thread* other, Grafo* g);
    friend void swap(Thread& lhs, Thread& rhs);


    priority_queue<int, vector<Eje>, Compare > _mstEjes;
    Grafo _mst;
    pthread_t _threadCreationIdx;
    queue<pair<Thread*, pair<int,int> > > _request_queue;
    bool _merged;
};




// Imprimir el grafo resultado durante los experimentos
bool imprimirResultado = true;


// Se sugieren usar variables (unas atómicas y otras no) para:

// Contener el estado global de la estructura de threads.

// Para coordinar el número de cada thread durante la inizializacion de threads.

// Para para coordinar el id de cada thread durante la inizializacion y reinicializacion de threads.

// Para coordinar las modificaciones de los colores.

// Para contener la estructura global que indica el estado actual de cada nodo.


void swap(Thread& lhs, Thread& rhs) {
    using std::swap; // enable ADL

    swap(lhs._mstEjes, rhs._mstEjes);
    swap(lhs._mst, rhs._mst);
    swap(lhs._threadCreationIdx, rhs._threadCreationIdx);
    swap(lhs._request_queue, rhs._request_queue);
    lhs._merged = rhs._merged;
    //lhs.merged.store(rhs.merged.load());

}

Thread& Thread::operator=(Thread other) {
    swap(*this, other);
    return *this;
}


//Retorna el nodo alcanzable a menor distancia
int Thread::buscarNodo(){
  // Se le pide el nodo apuntado por la cabeza de la cola de prioridad
  return _mstEjes.top().nodoDestino;
}


// Se pinta el nodo de negro para indicar que fue colocado en el árbol
void Thread::pintarNodo(Eje eje, sharedData* shared){

  shared->_nodeColorArray[eje.nodoDestino] = _threadCreationIdx;
  //auto pos = find(shared->_freeNodes.begin(), shared->_freeNodes.end(), eje.nodoDestino);
  //cout << "Posición del nodo a borrar " << *pos << endl;
  shared->_freeNodes.erase(find(shared->_freeNodes.begin(), shared->_freeNodes.end(), eje.nodoDestino));
  _mst.insertarNodo(eje.nodoDestino); //Inserto el nodo en el mst
  if(eje.nodoOrigen != -1){
    _mst.insertarEje(eje.nodoOrigen, eje.nodoDestino, eje.peso);
  }
}

// Se pintan los vecinos de gris para marcar que son alcanzables desde el árbol (salvo los que ya son del árbol)
void Thread::pintarVecinos(Grafo *g, int nodo){
   //Agrego todas las aristas que salen del nodo insertado y no apuntan a algún otro nodo de los que ya están en el mst
   for (int i = 0; i < g->listaDeAdyacencias[nodo].size(); i++) {
     if(_mst.noEsta(g->listaDeAdyacencias[nodo][i].nodoDestino)){
       _mstEjes.push(g->listaDeAdyacencias[nodo][i]);
     }
   }
}

//Reinicia las estructuras de un thread.
void Thread::reiniciarThread(sharedData* shared, unordered_map<pthread_t, Thread>* threadObjects){
    shared->_g->limpiarAuxiliares();
    initThread(shared, threadObjects);
}


// Iniciar un thread.
void Thread::initThread(sharedData* shared, unordered_map<pthread_t, Thread>* threadObjects){

    int node;
    bool nodeFound = false;
    // si no quedan nodos libres, se acabo la joda
    if(shared->_freeNodes.size() == 0) {
        pthread_exit(0);
    }
    nodeFound=false;
    while(!nodeFound){    
        node = shared->_freeNodes.back();

        // pido el mutex de ese Nodo
        pthread_mutex_lock(&shared->_nodesMutexes[node]);

        // Veo que nadie lo haya pintado
        if(shared->_nodeColorArray[node] == -1) {
            nodeFound = true;
            Eje eje(-1, node, -1);
            procesarNodo(eje, shared, threadObjects);
        }
        pthread_mutex_unlock(&shared->_nodesMutexes[node]);
        
    }

}

// Iniciar un thread.
void Thread::processThread(sharedData* shared, unordered_map<pthread_t, Thread>* threadObjects){
    int i = 0;
    cout << "Cantidad de nodos libres: " << shared->_freeNodes.size() << endl;
    cout << endl;
    while(shared->_freeNodes.size() > 0){
      cout << "Iteración número: " << i << endl;
      cout << "Eje a agregar: " << getNextEdge().nodoOrigen << "---" << getNextEdge().nodoDestino << endl;
      cout << "Peso a agregar: " << getNextEdge().peso << endl;
      Eje eje = getNextEdge();
      _mstEjes.pop();
      while(shared->_nodeColorArray[eje.nodoDestino] == _threadCreationIdx){
        cout << "Eje a agregar: " << getNextEdge().nodoOrigen << "---" << getNextEdge().nodoDestino << endl;
        cout << "Peso a agregar: " << getNextEdge().peso << endl;
        eje = getNextEdge();
        _mstEjes.pop();
      }
      procesarNodo(eje, shared, threadObjects);

      i++;
      cout << "Cantidad de nodos libres: " << shared->_freeNodes.size() << endl;
      cout << endl;
    }
}

void Thread::assignIdx(pthread_t threadCreationIdx){
  _threadCreationIdx = threadCreationIdx;
}

void Thread::procesarNodo(Eje eje, sharedData* shared, unordered_map<pthread_t, Thread>* threadObjects){
    pthread_t node_color = shared->_nodeColorArray[eje.nodoDestino];
    cout << node_color << endl;

    if(node_color == -1){
      pintarNodo(eje,shared);
      pintarVecinos(shared->_g, eje.nodoDestino);
    } else {
      // Hay que mergear 
      // Pido mi mutex para evitar que lleguen request mientras se resuelve mi merge
      if(pthread_mutex_trylock(&shared->_nodesMutexes[_threadCreationIdx]) == 0){
        bool busyWaiting = true;
        while(busyWaiting){
          if(pthread_mutex_trylock(&shared->_threadsMutexes[node_color]) == 0){
            // Pude pedir el mutex del thread candidato a ser dueño 
            if(node_color == shared->_nodeColorArray[eje.nodoDestino]){
              busyWaiting = false; 

              // hacer request 
              Thread other = (*threadObjects)[node_color];
              requestMerge(&other, eje.nodoOrigen, eje.nodoDestino); // Cambiar para que tome el eje

              pthread_mutex_unlock(&shared->_threadsMutexes[node_color]);

            } else {
              // Actualizo con el nuevo dueño 
              node_color == shared->_nodeColorArray[eje.nodoDestino];
            }
          } else {
            // Atiendo algo de mi cola porque podría estar esperándome a mi
            if(_request_queue.size() > 0){
              Thread* other = _request_queue.front().first;
              merge(other, shared->_g);
              _request_queue.pop();
            } 
          }
        }
        pthread_mutex_unlock(&shared->_nodesMutexes[_threadCreationIdx]);        
      }

    }
}

pthread_t Thread::getIdx(){
  return _threadCreationIdx;
}

Grafo* Thread::getMst(){
  return &_mst;
}

Eje Thread::getNextEdge(){
  return _mstEjes.top();
}

// Trata de reservar el nodo que se pasa como parametro para el thread

Thread Thread::tomarNodo(int nodo){

   // TODO
}

// Procurar agregar el thread con mayor id a la cola de fusiones del thread con menor id
void Thread::requestMerge(Thread* other, int source_node, int dest_node){
    // """TODO Se deben evitar race conditions, en los siguietes casos:
        // Un nodo hijo no puede estar en la cola de fusiones de otro nodo.
        // Solo se pueden agregar a la cola si el padre no está siendo fusionado por otro thread."""

    other->_request_queue.push(make_pair((this), make_pair(source_node, dest_node)));

}


// Realizar la fusión
void Thread::merge(Thread* other, Grafo *g){
    other->_merged=false;
    //Se determina el thread que tengo que fusionar
    if(_threadCreationIdx > other->_threadCreationIdx){
        //Se fusiona a mi
        int nodo;
        while(!other->_mstEjes.empty()){
            nodo=other->_mstEjes.top().nodoDestino;
            for (int i = 0; i < g->listaDeAdyacencias[nodo].size(); i++) {
                if(_mst.noEsta(g->listaDeAdyacencias[nodo][i].nodoDestino)){
                    _mstEjes.push(other->_mstEjes.top());
                }
                other->_mstEjes.pop();
            }
        }
        _request_queue.push(other->_request_queue.front()); // TODO(charli): esta bien que esta operacion de push solo se haga una vez?
        other->_request_queue.pop();
        other->_merged=true;
    }

    //Se determina el thread que tengo que fusionar

    //Se espera a que el thread esté listo para fusionarse

    //Se fusionan las colas de fusiones del hijo

    //Se fusionan las distancias del hijo
        // esto es fusionar ejes

    //Se notifica al hijo que se termino la fusion
}

// Para buscar un nodo libre en el grafo.
int buscarNodoLibre(){
   // TODO.
}


// Gestión principal del thread. Contiene el ciclo que le permite a cada thread hacer sus funciones.
void* mstParaleloThread(void *p){

    pair<unordered_map<pthread_t, Thread>, sharedData>* sharedPair = (pair<unordered_map<pthread_t, Thread>, sharedData>*) p;
    sharedData* shared = &sharedPair->second;
    unordered_map<pthread_t, Thread>* threadObjects = &sharedPair->first;

    // Se obtiene el numero de thread y se inicializan sus

    pthread_t tid = pthread_self();

    pthread_mutex_lock(&(shared->_mapMutex));

    (*threadObjects)[tid] = Thread();

    (*threadObjects)[tid].assignIdx(tid);

    //cout << "Fui creado y mi tid es " << (*threadObjects)[tid].getIdx() << " jeje" << endl;
    //cout << "Mi tid es: " << (*threadObjects)[tid].getIdx() << " y veo al mapa de tamaño " << threadObjects->size() <<  endl;

    pthread_mutex_unlock(&(shared->_mapMutex));

    (*threadObjects)[tid].initThread(shared, threadObjects);

    pthread_mutex_lock(&(shared->_initMutex));

    cout << "Soy " << tid << " y mi árbol es el siguiente " << endl;

    ((*threadObjects)[tid].getMst())->imprimirGrafo();

    pthread_mutex_unlock(&(shared->_initMutex));
    
    cout << "defined initThread succesfully" << endl;
    
    cout << "Colores" << endl;
    for (int i = 0; i < shared->_nodeColorArray.size(); ++i){
      cout << (shared->_nodeColorArray[i] == -1) << endl;
    }

    (*threadObjects)[tid].processThread(shared, threadObjects);
/*
    // Ciclo principal de cada thread
    while(true){

        break; // TODO(charli): sacar esto cuando agreguemos

        // Se termina la ejecución si el grafo ya no tiene vertices libres. Se imprime el resultado y se termina el thread

        // Si el thread está en la cola de fusiones de otro thread, lo notifica que puede fusionarse.

        // Se deben usar mecanismos de sincronización.

        // TO DO

	      // Si otro thread me está fusionando, esperar a que termine.

              		// Reinicializo las estructuras del thread y arranco de nuevo.

 	      // Si tiene elementos en la cola de fusion, debe fusionarlos.

             // Se busca el nodo más cercano que no esté en el árbol, pero que sea alcanzable
                  // nodoActual = buscarNodo(thread);

             // Se procura reservar el nodo que se quiere tomar, indicando la apropiación en la estructura usada.

                 // Thread info = tomarNodo(nodoActual, thread);

	     //Si se logra tomar, se procesa.

             //Si el nodo lo tiene otro thread, se tiene que fusionar

             //Espero hasta que liberen mi mutex -> para el thread i tengo threadsMutexes[i].wait()

             //Acá no me llegan más request

             // pregunto si el thread al que quiero hacer request esta bloqueado -> para el thread i intentando hacer request al j pthread_mutex_trylock(threadsMutexes[j])
                 // si?
                     // si me pidieron merges, los resuelvo y me restarteo -> atiendo cola
                 // no?
                     // entonces me encolo y me quedo esperando a que se resuelva el merge o me pidan algo -> para el thread i intentando pintar el nodo b desde el a requestMerge(threadObjects[j], a, b )

                  //requestFuse(.....);
    }
    */

}



void mstParalelo(Grafo *g, int cantThreads){

    //Verificar cantidad de threads para ejecutar el algoritmo

    if(cantThreads < 1){
        cerr << "El número de threads debe ser igual o mayor a 1" << endl;
    }

    //Si el numero de vertices del grafo es 0, imprimir el grafo vacio

    if(g->numVertices == 0){
        if(imprimirResultado){
            cout << endl << "********** RESULTADO *********** " << endl;
            Grafo().imprimirGrafo();
        }

        return;
    }

    // Se crean los threads
    pthread_t threads[cantThreads];
    // Se inicializan las estructuras globales

    unordered_map<pthread_t, Thread> threadObjects;

    //TODO(charli): asegurarnos de que cada vez que un nodo es pintado o fagocitado, esto cambia
    // tambien queremos que arranque inicializado en -1
    vector<pthread_t> nodeColorArray(g->numVertices, -1);

    vector<pthread_mutex_t> nodesMutexes(g->numVertices, PTHREAD_MUTEX_INITIALIZER);
    vector<pthread_mutex_t> threadsMutexes(cantThreads, PTHREAD_MUTEX_INITIALIZER);

    vector<int> freeNodes(g->numVertices);

    for (int i = 0; i < freeNodes.size(); i++) {
      freeNodes.at(i) = i;
    }

    sharedData shared;
    shared._g = g;
    //shared._threadObjects = threadObjects;
    shared._nodeColorArray = nodeColorArray;
    shared._nodesMutexes = nodesMutexes;
    shared._threadsMutexes = threadsMutexes;
    shared._freeNodes = freeNodes;

    pair<unordered_map<pthread_t, Thread>, sharedData> pair = make_pair(threadObjects, shared);

    // Se deben usar pthread_create y pthread_join.
    for (int threadIdx = 0; threadIdx < cantThreads; ++threadIdx) {
        pthread_create(&threads[threadIdx], NULL, mstParaleloThread, &pair); // TODO(charli): pasar todo lo que es memoria compartida
        //threadObjects[threadIdx] = Thread(threads[threadIdx], threadIdx);
    }
    for (int threadIdx = 0; threadIdx < cantThreads; ++threadIdx)
        pthread_join(threads[threadIdx], NULL);

}

//Reinicia la experimentación.
void resetExperimentacion(){
   // TO DO

}

//Procedimiento para realizar las pruebas o test mínimo de la cátedra.

void experimentacion(){
    imprimirResultado = false;
    std::cout << "instancia,n,grafo,threads, tiempo" << std::endl;
    int instancia = 0;
    string grafo;

    for (int n = 100; n<=1000 ; n += 100){
        for (int k = 0; k <=2; k++){
            Grafo g;
            if(k == 0){
                if( g.inicializar("test/experimentacion/arbol/arbol" + to_string(n) +".txt") != 1){
                    cerr << "No se pudo cargar el grafo correctamente" << endl;
                    return;
                }
            }
            if(k == 1){
                if( g.inicializar("test/experimentacion/ralo/ralo" + to_string(n) + ".txt") != 1){
                    cerr << "No se pudo cargar el grafo correctamente" << endl;
                    return;
                }
            }
            if(k == 2){
                if( g.inicializar("test/experimentacion/completo/completo" + to_string(n) + ".txt") != 1){
                    cerr << "No se pudo cargar el grafo correctamente" << endl;
                    return;
                }
            }
            for (int i = 0; i < 10; i++){

                if(k == 0){
                    grafo = "arbol";
                    auto start = std::chrono::steady_clock::now();
                    //mstSecuencial(&g);
                    auto end = std::chrono::steady_clock::now();

                    std::cout << instancia << "," << n << "," << grafo << "," << 1 << ","
                              << std::chrono::duration <double, std::milli> (end-start).count()
                              << std::endl;
                    instancia++;
                    resetExperimentacion();
                }

                if(k == 1){
                    grafo = "ralo";
                    auto start = std::chrono::steady_clock::now();
                    //mstSecuencial(&g);
                    auto end = std::chrono::steady_clock::now();

                    std::cout << instancia << "," << n << "," << grafo << "," << 1 << ","
                              << std::chrono::duration <double, std::milli> (end-start).count()
                              << std::endl;
                    instancia++;
                    resetExperimentacion();
                }

                if(k == 2){
                    grafo = "completo";
                    auto start = std::chrono::steady_clock::now();
                    //mstSecuencial(&g);
                    auto end = std::chrono::steady_clock::now();

                    std::cout << instancia << "," << n << "," << grafo << "," << 1 << ","
                              << std::chrono::duration <double, std::milli> (end-start).count()
                              << std::endl;
                    instancia++;
                    resetExperimentacion();
                }
                //for (int threads = 2; threads <= 32; threads *= 2){
                for (int threads = 1; threads <2; threads *= 2){
                    if(k == 0){
                        grafo = "arbol";
                        auto start = std::chrono::steady_clock::now();
                        mstParalelo(&g, threads);
                        auto end = std::chrono::steady_clock::now();

                        std::cout << instancia << "," << n << "," << grafo << "," << threads << ","
                                  << std::chrono::duration <double, std::milli> (end-start).count()
                                  << std::endl;
                        instancia++;
                        resetExperimentacion();
                    }

                    if(k == 1){
                        grafo = "ralo";
                        auto start = std::chrono::steady_clock::now();
                        mstParalelo(&g, threads);
                        auto end = std::chrono::steady_clock::now();

                        std::cout << instancia << "," << n << "," << grafo << "," << threads << ","
                                  << std::chrono::duration <double, std::milli> (end-start).count()
                                  << std::endl;
                        instancia++;
                        resetExperimentacion();
                    }

                    if(k == 2){
                        grafo = "completo";
                        auto start = std::chrono::steady_clock::now();
                        mstParalelo(&g, threads);
                        auto end = std::chrono::steady_clock::now();

                        std::cout << instancia << "," << n << "," << grafo << "," << threads << ","
                                  << std::chrono::duration <double, std::milli> (end-start).count()
                                  << std::endl;
                        instancia++;
                        resetExperimentacion();
                    }
                }

            }
        }
    }


}

int test(string path){
  Grafo g;
  if( g.inicializar(path) == 1){

          mstParalelo(&g, 1);

  }else{
    cerr << "Error: Grafo no cargado correctamente" << endl;
      return 1;
  }
}


int main(int argc, char const * argv[]) {

    if(argc <= 1){
    	cerr << "Introduzca el nombre del archivo o el parámetro \"-e\" para hacer varias pruebas " << endl;
        return 1;
    }

    if (string(argv[1]) == "-e"){
        experimentacion();
        return 0;
    }

    if (string(argv[1]) == "-t"){
        test(argv[2]);
        return 0;
    }

    string nombre;
    nombre = string(argv[1]);
    int threads = 1;
    if (argc > 2){
        string algoritmo = string(argv[2]);
    }

    if (argc > 3){
        threads = atoi(argv[3]);
    }


    Grafo g;
    if( g.inicializar(nombre) == 1){

            mstParalelo(&g, threads);

    }else{
	    cerr << "Error: Grafo no cargado correctamente" << endl;
        return 1;
    }

  return 0;
}

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
#include <queue> // std::priority_queue
#include <fstream>

#include <typeinfo>

using namespace std;

// Datos compartidos entre threads
struct sharedData
{
  Grafo *_g; // Grafo a cubrir
  vector<pthread_t> _nodeColorArray;
  vector<pthread_mutex_t> _nodesMutexes;
  //vector<pthread_mutex_t> _threadsMutexes;
  unordered_map<pthread_t, pthread_mutex_t> _threadsMutexes;
  vector<int> _freeNodes;
  pthread_mutex_t _mapMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t _initMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t _threadCreationMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t _freeNodesMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_barrier_t _initBarrier;
  pthread_mutex_t _mergeCounterMutex= PTHREAD_MUTEX_INITIALIZER;
  int _mergeCounter=0;
};

class Thread
{
  // Estructura que debe contener los colores de los vértices (actual y vecinos). Las distancias, el árbol, y la herramientas de sincronización necesarias para evitar race conditions y deadlocks.
public:
  Thread()
  {
    _merged = false;
    _die = false;
    _verbose = false;

  };
  Thread &operator=(Thread other);
  int buscarNodo();
  void pintarNodo(Eje eje, sharedData *shared);
  void pintarVecinos(sharedData *shared, int num);
  void reiniciarThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects);
  void initThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects);
  void processThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects);
  void fagocitar(Thread *other, Eje eje, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects);
  void assignIdx(pthread_t threadCreationIdx);
  void msgLog(string msg);
  pthread_t getIdx();
  Eje getNextEdge(sharedData *shared);
  Grafo *getMst();
  bool procesarNodo(Eje eje, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects);
  Thread tomarNodo(int nodo);
  void requestMerge(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects, Thread *other, Eje eje, pthread_t node_color);
  void merge(pair<Thread *, Eje> req, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects);
  void time_to_die();
  friend void swap(Thread &lhs, Thread &rhs);

  priority_queue<int, vector<Eje>, Compare> _mstEjes;
  Grafo _mst;
  pthread_t _threadCreationIdx;
  queue<pair<Thread *, Eje>> _request_queue; //TODO(charli): agregar eje como segundo elem
  bool _merged;
  bool _die;
  bool _verbose = false;
};

// Imprimir el grafo resultado durante los experimentos
bool imprimirResultado = false;

// Se sugieren usar variables (unas atómicas y otras no) para:

// Contener el estado global de la estructura de threads.

// Para coordinar el número de cada thread durante la inizializacion de threads.

// Para para coordinar el id de cada thread durante la inizializacion y reinicializacion de threads.

// Para coordinar las modificaciones de los colores.

// Para contener la estructura global que indica el estado actual de cada nodo.

void swap(Thread &lhs, Thread &rhs)
{
  using std::swap; // enable ADL

  swap(lhs._mstEjes, rhs._mstEjes);
  swap(lhs._mst, rhs._mst);
  swap(lhs._threadCreationIdx, rhs._threadCreationIdx);
  swap(lhs._request_queue, rhs._request_queue);
  lhs._merged = rhs._merged;
  //lhs.merged.store(rhs.merged.load());
}

Thread &Thread::operator=(Thread other)
{
  swap(*this, other);
  return *this;
}

//Retorna el nodo alcanzable a menor distancia
int Thread::buscarNodo()
{
  // Se le pide el nodo apuntado por la cabeza de la cola de prioridad
  return _mstEjes.top().nodoDestino;
}

// Se pinta el nodo de negro para indicar que fue colocado en el árbol
void Thread::pintarNodo(Eje eje, sharedData *shared)
{

  shared->_nodeColorArray[eje.nodoDestino] = _threadCreationIdx;
  //auto pos = find(shared->_freeNodes.begin(), shared->_freeNodes.end(), eje.nodoDestino);
  //cout << "Posición del nodo a borrar " << *pos << endl;
  pthread_mutex_lock(&shared->_freeNodesMutex);
  shared->_freeNodes.erase(find(shared->_freeNodes.begin(), shared->_freeNodes.end(), eje.nodoDestino));
  pthread_mutex_unlock(&shared->_freeNodesMutex);
  _mst.insertarNodo(eje.nodoDestino); //Inserto el nodo en el mst
  if (eje.nodoOrigen != -1)
  {
    _mst.insertarEje(eje.nodoOrigen, eje.nodoDestino, eje.peso);
  }
  //cout << "Estoy en pintarNodo y el tamaño del mapa es: " << shared->_g->listaDeAdyacencias.size() << endl;
}

// Se pintan los vecinos de gris para marcar que son alcanzables desde el árbol (salvo los que ya son del árbol)
void Thread::pintarVecinos(sharedData *shared, int nodo)
{
  //Agrego todas las aristas que salen del nodo insertado y no apuntan a algún otro nodo de los que ya están en el mst
  //cout << "Paso 16: Listo" << endl;
  ////cout << "Estoy en pintarVecinos y el tamaño del mapa es: " << g->listaDeAdyacencias[0].size() << endl;

  for (int i = 0; i < shared->_g->listaDeAdyacencias[nodo].size(); i++)
  {
    //cout << "Paso 17: Listo" << endl;
    //shared->_nodeColorArray.at(eje.nodoDestino) == _threadCreationIdx
    if (shared->_nodeColorArray.at(shared->_g->listaDeAdyacencias[nodo].at(i).nodoDestino) != _threadCreationIdx)
    {
      _mstEjes.push(shared->_g->listaDeAdyacencias[nodo].at(i));
    }
  }
}

//Reinicia las estructuras de un thread.
void Thread::reiniciarThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{
  msgLog("nodos libres al reiniciar" + to_string(shared->_freeNodes.size()));
  initThread(shared, threadObjects);
}

void Thread::time_to_die()
{
  if(_verbose){
      string filename = to_string((unsigned long)_threadCreationIdx) + ".out";
      string new_filename = filename.substr(filename.length() - 6, 6);
  
      char filename_char[filename.length() + 1];
      char new_filename_char[new_filename.length() + 1];
  
      copy(filename.begin(), filename.end(), filename_char);
      copy(new_filename.begin(), new_filename.end(), new_filename_char);
  
      filename_char[filename.length()] = '\0';
      new_filename_char[new_filename.length()] = '\0';
  
      rename(filename_char, new_filename_char);
      
    }
    pthread_exit(0);
}

// Iniciar un thread.
void Thread::initThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{

  int node;
  bool nodeFound = false;
  // si no quedan nodos libres, se acabo la joda

  //cout << "Paso 8: Listo" << endl;
  msgLog("Init start");
  while (!nodeFound){
    if (shared->_freeNodes.size() == 0)
    {
      msgLog("Time to die");
      _die = true;
      return;
    }
    msgLog("Todavía no conseguí");
    node = shared->_freeNodes.back();
    // pido el mutex de ese Nodo
    pthread_mutex_lock(&shared->_nodesMutexes[node]);
    //cout << "Paso 10: Listo" << endl;

    // Veo que nadie lo haya pintado
    if (((long)shared->_nodeColorArray[node]) == -1)
    {
      nodeFound = true;
      Eje eje(-1, node, -1);
      procesarNodo(eje, shared, threadObjects);
    }
    pthread_mutex_unlock(&shared->_nodesMutexes[node]);
    //msgLog("unlock init");
  }
  msgLog("Init end");
}

void Thread::msgLog(string msg){
  if(_verbose){
    string filename = to_string((unsigned long)_threadCreationIdx) + ".out";
    fstream outfile;
    outfile.open(filename, fstream::in | fstream::out | fstream::app);
    outfile << "tid: " + to_string((unsigned long)_threadCreationIdx) + " " + msg << endl;
    outfile.close();
  }
}

// Iniciar un thread.
void Thread::processThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{
  msgLog("Entré a processThread");
  bool procesado = true;
  if (_mstEjes.size() > 0){
    Eje eje = getNextEdge(shared);
    while (_mst.numVertices < shared->_g->numVertices){

      msgLog("conseguí eje: " + to_string(eje.nodoOrigen) + "----" + to_string(eje.nodoDestino) + " y el color del nodo destino es " + to_string((long)shared->_nodeColorArray[eje.nodoDestino]));

      bool tome_mutex = false;
      if(pthread_mutex_trylock(&shared->_nodesMutexes.at(eje.nodoDestino)) == 0){
        tome_mutex = true;
        procesado = procesarNodo(eje, shared, threadObjects);
        pthread_mutex_unlock(&shared->_nodesMutexes.at(eje.nodoDestino));
      }
      procesado = procesado && tome_mutex;
      //msgLog("unlock processThread");

      if (_request_queue.size() > 0){

        msgLog(" atiendo porque tengo " + to_string(_request_queue.size()) + " pedidos");
        auto pair = _request_queue.front();
        _request_queue.pop();
        merge(pair, shared, threadObjects);
        if (_die){
          msgLog("Debo morir luego de llamar a merge");
          time_to_die();
        }
        //_merged = false;
        msgLog(" merged vale " + to_string(_merged));
        eje = getNextEdge(shared);
        msgLog(" volví");
        procesado = false;
      }
      msgLog(" procesado vale " + to_string(procesado));

      eje = procesado ? getNextEdge(shared) : eje;
      //msgLog(" mi top es " + to_string((long)_mstEjes.top().nodoOrigen) + "----" + to_string((long)_mstEjes.top().nodoDestino));
      procesado = true;
      //getMst()->imprimirGrafo();
    }
    /*if(getMst()->numVertices == shared->_g->numVertices){ // thread contiene todos los nodos del grafo ==> thread ganador
          // TODO(charli): ver cual es el formato de output que se espera
          cout << endl;

      }*/
  }
  if (getMst()->numVertices == shared->_g->numVertices)
  { // thread contiene todos los nodos del grafo ==> thread ganador
    cout << "Printeando el grafo obtenido por el thread ganador" << endl;
    pthread_mutex_lock(&shared->_mergeCounterMutex);
    getMst()->setCantMerges(shared->_mergeCounter);
    pthread_mutex_unlock(&shared->_mergeCounterMutex);
    getMst()->setFinalizo(true);
    getMst()->imprimirGrafo();
  }
  time_to_die();
}

void Thread::assignIdx(pthread_t threadCreationIdx)
{
  _threadCreationIdx = threadCreationIdx;
}

bool Thread::procesarNodo(Eje eje, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{
  //cout << "Estoy en procesarNodo y el tamaño del mapa es: " << shared->_g->listaDeAdyacencias.size() << endl;
  bool res = true;
  pthread_t node_color = shared->_nodeColorArray[eje.nodoDestino];
  //cout << "Paso 13: Listo" << endl;
  if (((long)node_color) == -1)
  {
    //cout << "Paso 14: Listo" << endl;
    pintarNodo(eje, shared);
    //cout << "Paso 15: Listo" << endl;
    pintarVecinos(shared, eje.nodoDestino);

    return res;
  }
  else
  {
    // Hay que mergear
    // Pido mi mutex para evitar que lleguen request mientras se resuelve mi merge
    bool merge_solved = false;
    msgLog("trylock1 procesarNodo");
    if (pthread_mutex_trylock(&shared->_threadsMutexes.at(_threadCreationIdx)) == 0)
    {
      msgLog("lock1 procesarNodo");
      if (pthread_mutex_trylock(&shared->_threadsMutexes.at(node_color)) == 0)
      {
        msgLog("lock2 procesarNodo");
        if (node_color == shared->_nodeColorArray.at(eje.nodoDestino)){

          // hacer request
          Thread *other = &(*threadObjects).at(node_color);

          // TODO(charli): quedarse esperando hasta que el merge sea resuelto
          requestMerge(shared, threadObjects, other, eje, node_color); // Cambiar para que tome el eje

          msgLog(" pido merge");
          while (!_merged){}
          msgLog(" me atendieron y _merged es " + to_string(_merged));
          merge_solved = true;
        }
      }
      pthread_mutex_unlock(&shared->_threadsMutexes.at(_threadCreationIdx));
      msgLog("unlock1 procesarNodo");
      _merged = false;
    }
    if (_die){
      msgLog("Debo morir luego de encolarme");
      time_to_die();
    }
    return res && merge_solved;
  }
}

pthread_t Thread::getIdx()
{
  return _threadCreationIdx;
}

Grafo *Thread::getMst()
{
  return &_mst;
}

Eje Thread::getNextEdge(sharedData *shared){
  Eje eje = _mstEjes.top();
  _mstEjes.pop();
  if (_mst.numVertices != shared->_g->numVertices){
    while (shared->_nodeColorArray.at(eje.nodoDestino) == _threadCreationIdx)
    {
      eje = _mstEjes.top();
      _mstEjes.pop();
    }
  }
  //msgLog("¿son iguales? => " + to_string(shared->_nodeColorArray.at(eje.nodoDestino) == _threadCreationIdx ));
  return eje;
}

// Trata de reservar el nodo que se pasa como parametro para el thread

Thread Thread::tomarNodo(int nodo)
{

  // TODO
}

// Procurar agregar el thread con mayor id a la cola de fusiones del thread con menor id
void Thread::requestMerge(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects, Thread *other, Eje eje, pthread_t node_color){
  // """TODO Se deben evitar race conditions, en los siguietes casos:
  // Un nodo hijo no puede estar en la cola de fusiones de otro nodo.
  // Solo se pueden agregar a la cola si el padre no está siendo fusionado por otro thread."""
  //cout << "Espero por " << other->_threadCreationIdx << " y soy " << _threadCreationIdx << endl;
  other->_request_queue.push(make_pair((&((*threadObjects)[_threadCreationIdx])), eje));
  pthread_mutex_unlock(&shared->_threadsMutexes.at(node_color));
  msgLog("unlock2 procesarNodo");
}

void Thread::fagocitar(Thread *other, Eje eje, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{
  msgLog("Fagocito");
  msgLog("Eje que provocó el merge" + to_string(eje.nodoOrigen) + "----" + to_string(eje.nodoDestino));
  cout << "Mi grafo " << endl;
  _mst.imprimirGrafo();
  cout << "El grafo de other " << endl;
  other->_mst.imprimirGrafo();
  // nodos
  for (auto const &x : other->_mst.listaDeAdyacencias)
  {
    msgLog("Inserto nodo " + to_string(x.first));
    _mst.insertarNodo(x.first);
  }

  // ejes del mst
  for (auto x : other->_mst.listaDeAdyacencias)
  {
    auto listaDeEjes = x.second;
    for (auto e : listaDeEjes)
    {
      msgLog("Inserto eje " + to_string(e.nodoOrigen) + "----" + to_string(e.nodoDestino));
      _mst.insertarEje(e); // TODO(charli): verificar que esta asignacion no se haga mierda cuando vaciamos other
    }
  }

  msgLog("");

  _mst.insertarEje(eje.nodoOrigen, eje.nodoDestino, eje.peso);

  // pisar lista de adyacencias de other
  map<int, vector<Eje>> nuevaListaDeAdyacencias;
  other->_mst.listaDeAdyacencias = nuevaListaDeAdyacencias;

  // ejes a explorar
  // priority_queue<int, vector<Eje>, Compare > _mstEjes;
  /*while(other->_mstEjes.size() > 0){
        _mstEjes.push(other->_mstEjes.top());
        other->_mstEjes.pop();
    }*/

  priority_queue<int, vector<Eje>, Compare> newMstEjes;
  priority_queue<int, vector<Eje>, Compare> newOtherMstEjes;

  _mstEjes = newMstEjes;
  other->_mstEjes = newOtherMstEjes;

  // requests
  // queue<pair<Thread*, pair<int,int> > > _request_queue;
  while (other->_request_queue.size() > 0)
  {
    _request_queue.push(other->_request_queue.front());
    other->_request_queue.pop();
    msgLog("requests pendientes: " + to_string(other->_request_queue.size()));
  }
  msgLog("Transferí requests");

  // le pinto los nodos de mi color
  for (int i = 0; i < shared->_nodeColorArray.size(); ++i)
  {
    if (shared->_nodeColorArray[i] == other->_threadCreationIdx)
    {
      shared->_nodeColorArray[i] = _threadCreationIdx;
    }
  }

  msgLog("Pinté nodos");

  // obtengo nuevos ejes a explorar
  for (int i = 0; i < shared->_nodeColorArray.size(); ++i)
  {
    if (shared->_nodeColorArray[i] == _threadCreationIdx)
    {
      auto listaDeEjes = shared->_g->listaDeAdyacencias[i];
      for (auto x : listaDeEjes)
      {
        if (shared->_nodeColorArray[x.nodoDestino] != _threadCreationIdx)
        {
          msgLog(" pusheo " + to_string(x.nodoOrigen) + "----" + to_string(x.nodoDestino));
          _mstEjes.push(x);
        }
      }
    }
  }

  msgLog("_mstEjes actualizado");

  other->_mst = Grafo();

  other->reiniciarThread(shared, threadObjects);

  msgLog("other reiniciado");
  //msgLog(to_string((long)other->_threadCreationIdx) + " y en top the other queda el eje " + to_string(other->_mstEjes.top().nodoOrigen) + "----" + to_string(other->_mstEjes.top().nodoDestino));
  //msgLog(to_string((long)_threadCreationIdx) + " y en mi top queda el eje " + to_string(_mstEjes.top().nodoOrigen) + "----" + to_string(_mstEjes.top().nodoDestino));

  cout << "Mi grafo al final " << endl;
  _mst.imprimirGrafo();
  cout << "El grafo de other al final " << endl;
  other->_mst.imprimirGrafo();
}

// Realizar la fusión
void Thread::merge(pair<Thread *, Eje> req, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{
    pthread_mutex_lock(&(shared->_mergeCounterMutex));
  shared->_mergeCounter++;
  pthread_mutex_unlock(&(shared->_mergeCounterMutex));
  //msgLog(" mergeo");
  //other->_merged=false; // Habría que setearlo en true al final para avisarle al otro thread que terminó el merge
  if (_threadCreationIdx > req.first->_threadCreationIdx)
  {
    // yo ingiero al other
    fagocitar(req.first, req.second, shared, threadObjects);
  }
  else
  {
    // el other me ingiere cual globulo blanco a bacteria oprimida
    req.first->fagocitar(&((*threadObjects)[_threadCreationIdx]), req.second, shared, threadObjects);
  }
  req.first->_merged = true;
}

// Para buscar un nodo libre en el grafo.
int buscarNodoLibre()
{
  // TODO
}

// Gestión principal del thread. Contiene el ciclo que le permite a cada thread hacer sus funciones.
void *mstParaleloThread(void *p)
{

  pair<unordered_map<pthread_t, Thread>, sharedData> *sharedPair = (pair<unordered_map<pthread_t, Thread>, sharedData> *)p;
  sharedData *shared = &sharedPair->second;
  unordered_map<pthread_t, Thread> *threadObjects = &sharedPair->first;

  // Se obtiene el numero de thread y se inicializan sus
  //cout << "Paso 2: Listo" << endl;

  pthread_t tid = pthread_self();

  //msgLog(" voy a crear mi mutex");
  cout << "Soy " + to_string((long) tid) + " y creo mi mutex" << endl;

  pthread_mutex_t mu;
  pthread_mutex_lock(&(shared->_mapMutex));
  shared->_threadsMutexes.insert({tid, mu});
  pthread_mutex_init(&shared->_threadsMutexes.at(tid), NULL);
  //cout << "Paso 3: Listo" << endl;

  //msgLog("voy a crear mi objeto thread");

  cout << "Soy " + to_string((long) tid) + " y creo mi objeto thread" << endl;

  pthread_mutex_lock(&(shared->_threadCreationMutex));
  (*threadObjects)[tid] = Thread();
  pthread_mutex_unlock(&(shared->_threadCreationMutex));

  pthread_mutex_unlock(&(shared->_mapMutex));
  //(*threadObjects)[tid].msgLog("Creación de thread exitosa");
  //cout << "Paso 4: Listo" << endl;
  pthread_barrier_wait(&(shared->_initBarrier));
  //cout << "Paso 5: Listo" << endl;
  (*threadObjects).at(tid).assignIdx(tid);

  //cout << "Paso 6: Listo" << endl;

  pthread_mutex_lock(&(shared->_initMutex));

  //cout << "Paso 7: Listo" << endl;

  (*threadObjects).at(tid).initThread(shared, threadObjects);

  //((*threadObjects)[tid].getMst())->imprimirGrafo();

  cout << "Soy " + to_string((long) tid) + " y mi init fue exitoso" << endl;

  pthread_mutex_unlock(&(shared->_initMutex));

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

void mstParalelo(Grafo *g, int cantThreads)
{
  // system("./borrar_logs.sh"); // con el flag verbose no hace falta automatizar esto. aviso que rompe dependiendo del path en el que corras desde python ==> es gede

  //Verificar cantidad de threads para ejecutar el algoritmo
  //cout << "Estoy en mstParalelo y el tamaño del mapa es: " << g->listaDeAdyacencias.size() << endl;
  if (cantThreads < 1)
  {
    cerr << "El número de threads debe ser igual o mayor a 1" << endl;
  }

  //Si el numero de vertices del grafo es 0, imprimir el grafo vacio

  if (g->numVertices == 0)
  {
    if (imprimirResultado)
    {
      cout << endl
           << "********** RESULTADO *********** " << endl;
      Grafo().imprimirGrafo();
    }

    return;
  }

  // Se crean los threads
  pthread_t threads[cantThreads];
  // Se inicializan las estructuras globales

  unordered_map<pthread_t, Thread> threadObjects;
  unordered_map<pthread_t, pthread_mutex_t> threadsMutexes;

  //TODO(charli): asegurarnos de que cada vez que un nodo es pintado o fagocitado, esto cambia
  // tambien queremos que arranque inicializado en -1
  vector<pthread_t> nodeColorArray(g->numVertices, -1);

  vector<pthread_mutex_t> nodesMutexes(g->numVertices);
  for (int i = 0; i < nodesMutexes.size(); ++i)
  {
    pthread_mutex_init(&nodesMutexes.at(i), NULL);
  }

  vector<int> freeNodes(g->numVertices);

  for (int i = 0; i < freeNodes.size(); i++)
  {
    freeNodes.at(i) = i;
  }

  sharedData shared;
  shared._g = g;
  //shared._threadObjects = threadObjects;
  shared._nodeColorArray = nodeColorArray;
  shared._nodesMutexes = nodesMutexes;
  shared._threadsMutexes = threadsMutexes;
  shared._freeNodes = freeNodes;
  pthread_barrier_init(&(shared._initBarrier), NULL, cantThreads);
  pair<unordered_map<pthread_t, Thread>, sharedData> pair = make_pair(threadObjects, shared);
  //cout << "Paso 1: Listo" << endl;
  // Se deben usar pthread_create y pthread_join.
  cout << "Inicializo los llamados" << endl;
  for (int threadIdx = 0; threadIdx < cantThreads; ++threadIdx)
  {
    pthread_create(&threads[threadIdx], NULL, mstParaleloThread, &pair); // TODO(charli): pasar todo lo que es memoria compartida
                                                                         //threadObjects[threadIdx] = Thread(threads[threadIdx], threadIdx);
  }
  for (int threadIdx = 0; threadIdx < cantThreads; ++threadIdx)
    pthread_join(threads[threadIdx], NULL);
}

//Reinicia la experimentación.
void resetExperimentacion()
{
  // TO DO
}

//Procedimiento para realizar las pruebas o test mínimo de la cátedra.

void experimentacion()
{
  imprimirResultado = false;
  std::cout << "instancia,n,grafo,threads, tiempo" << std::endl;
  int instancia = 0;
  string grafo;

  for (int n = 100; n <= 1000; n += 100)
  {
    for (int k = 0; k <= 2; k++)
    {
      Grafo g;
      if (k == 0)
      {
        if (g.inicializar("test/experimentacion/arbol/arbol" + to_string(n) + ".txt") != 1)
        {
          cerr << "No se pudo cargar el grafo correctamente" << endl;
          return;
        }
      }
      if (k == 1)
      {
        if (g.inicializar("test/experimentacion/ralo/ralo" + to_string(n) + ".txt") != 1)
        {
          cerr << "No se pudo cargar el grafo correctamente" << endl;
          return;
        }
      }
      if (k == 2)
      {
        if (g.inicializar("test/experimentacion/completo/completo" + to_string(n) + ".txt") != 1)
        {
          cerr << "No se pudo cargar el grafo correctamente" << endl;
          return;
        }
      }
      for (int i = 0; i < 10; i++)
      {

        if (k == 0)
        {
          grafo = "arbol";
          auto start = std::chrono::steady_clock::now();
          //mstSecuencial(&g);
          auto end = std::chrono::steady_clock::now();

          std::cout << instancia << "," << n << "," << grafo << "," << 1 << ","
                    << std::chrono::duration<double, std::milli>(end - start).count()
                    << std::endl;
          instancia++;
          resetExperimentacion();
        }

        if (k == 1)
        {
          grafo = "ralo";
          auto start = std::chrono::steady_clock::now();
          //mstSecuencial(&g);
          auto end = std::chrono::steady_clock::now();

          std::cout << instancia << "," << n << "," << grafo << "," << 1 << ","
                    << std::chrono::duration<double, std::milli>(end - start).count()
                    << std::endl;
          instancia++;
          resetExperimentacion();
        }

        if (k == 2)
        {
          grafo = "completo";
          auto start = std::chrono::steady_clock::now();
          //mstSecuencial(&g);
          auto end = std::chrono::steady_clock::now();

          std::cout << instancia << "," << n << "," << grafo << "," << 1 << ","
                    << std::chrono::duration<double, std::milli>(end - start).count()
                    << std::endl;
          instancia++;
          resetExperimentacion();
        }
        for (int threads = 2; threads <= 32; threads *= 2)
        {
          //for (int threads = 1; threads <2; threads *= 2){
          if (k == 0)
          {
            grafo = "arbol";
            auto start = std::chrono::steady_clock::now();
            mstParalelo(&g, threads);
            auto end = std::chrono::steady_clock::now();

            std::cout << instancia << "," << n << "," << grafo << "," << threads << ","
                      << std::chrono::duration<double, std::milli>(end - start).count()
                      << std::endl;
            instancia++;
            resetExperimentacion();
          }

          if (k == 1)
          {
            grafo = "ralo";
            auto start = std::chrono::steady_clock::now();
            mstParalelo(&g, threads);
            auto end = std::chrono::steady_clock::now();

            std::cout << instancia << "," << n << "," << grafo << "," << threads << ","
                      << std::chrono::duration<double, std::milli>(end - start).count()
                      << std::endl;
            instancia++;
            resetExperimentacion();
          }

          if (k == 2)
          {
            grafo = "completo";
            auto start = std::chrono::steady_clock::now();
            mstParalelo(&g, threads);
            auto end = std::chrono::steady_clock::now();

            std::cout << instancia << "," << n << "," << grafo << "," << threads << ","
                      << std::chrono::duration<double, std::milli>(end - start).count()
                      << std::endl;
            instancia++;
            resetExperimentacion();
          }
        }
      }
    }
  }
}

int test(string path, int cantThreads)
{
  Grafo g;
  if (g.inicializar(path) == 1)
  {

    mstParalelo(&g, cantThreads);
  }
  else
  {
    cerr << "Error: Grafo no cargado correctamente" << endl;
    return 1;
  }
}

int main(int argc, char const *argv[])
{

  if (argc <= 1)
  {
    cerr << "Introduzca el nombre del archivo o el parámetro \"-e\" para hacer varias pruebas " << endl;
    return 1;
  }

  if (string(argv[1]) == "-e")
  {
    experimentacion();
    return 0;
  }

  if (string(argv[1]) == "-t")
  {
    test(argv[2], atoi(argv[3]));
    return 0;
  }

  string nombre;
  nombre = string(argv[1]);
  int threads = 1;
  if (argc > 2)
  {
    string algoritmo = string(argv[2]);
  }

  if (argc > 3)
  {
    threads = atoi(argv[3]);
  }

  Grafo g;
  if (g.inicializar(nombre) == 1)
  {

    mstParalelo(&g, threads);
  }
  else
  {
    cerr << "Error: Grafo no cargado correctamente" << endl;
    return 1;
  }

  return 0;
}

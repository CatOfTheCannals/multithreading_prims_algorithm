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
  unordered_map<pthread_t, pthread_mutex_t> _threadsMutexes;
  vector<int> _freeNodes;
  pthread_mutex_t _mapMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t _initMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t _threadCreationMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t _freeNodesMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_barrier_t _initBarrier;
  pthread_mutex_t _mergeCounterMutex= PTHREAD_MUTEX_INITIALIZER;
  int _mergeCounter=0;
  bool _verbose = false;
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
  void requestMerge(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects, Thread *other, Eje eje, pthread_t node_color);
  void merge(pair<Thread *, Eje> req, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects);
  void time_to_die();
  friend void swap(Thread &lhs, Thread &rhs);

  priority_queue<int, vector<Eje>, Compare> _mstEjes;
  Grafo _mst;
  pthread_t _threadCreationIdx;
  queue<pair<Thread *, Eje>> _request_queue;
  bool _merged;
  bool _die;
  bool _verbose = false;
};

// Imprimir el grafo resultado durante los experimentos
bool imprimirResultado = false;
bool _verbose = false;

void swap(Thread &lhs, Thread &rhs)
{
  using std::swap;

  swap(lhs._mstEjes, rhs._mstEjes);
  swap(lhs._mst, rhs._mst);
  swap(lhs._threadCreationIdx, rhs._threadCreationIdx);
  swap(lhs._request_queue, rhs._request_queue);
  lhs._merged = rhs._merged;
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
  pthread_mutex_lock(&shared->_freeNodesMutex);
  shared->_freeNodes.erase(find(shared->_freeNodes.begin(), shared->_freeNodes.end(), eje.nodoDestino));
  pthread_mutex_unlock(&shared->_freeNodesMutex);
  _mst.insertarNodo(eje.nodoDestino); //Inserto el nodo en el mst
  if (eje.nodoOrigen != -1)
  {
    _mst.insertarEje(eje.nodoOrigen, eje.nodoDestino, eje.peso);
  }
}

// Se pintan los vecinos de gris para marcar que son alcanzables desde el árbol (salvo los que ya son del árbol)
void Thread::pintarVecinos(sharedData *shared, int nodo)
{
  //Agrego todas las aristas que salen del nodo insertado y no apuntan a algún otro nodo de los que ya están en el mst
  for (int i = 0; i < shared->_g->listaDeAdyacencias[nodo].size(); i++)
  {
    if (shared->_nodeColorArray.at(shared->_g->listaDeAdyacencias[nodo].at(i).nodoDestino) != _threadCreationIdx)
    {
      _mstEjes.push(shared->_g->listaDeAdyacencias[nodo].at(i));
    }
  }
}

//Reinicia las estructuras de un thread.
void Thread::reiniciarThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{
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
    // Termina la ejecución del thread
    pthread_exit(0);
}

// Iniciar un thread.
void Thread::initThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{

  int node;
  bool nodeFound = false;
  // si no quedan nodos libres, se acabo la joda

  while (!nodeFound){
    if (shared->_freeNodes.size() == 0)
    {
      _die = true;
      return;
    }
    node = shared->_freeNodes.back();
    // pido el mutex de ese Nodo
    pthread_mutex_lock(&shared->_nodesMutexes[node]);

    // Veo que nadie lo haya pintado
    if (((long)shared->_nodeColorArray[node]) == -1)
    {
      nodeFound = true;
      Eje eje(-1, node, -1);
      procesarNodo(eje, shared, threadObjects);
    }
    pthread_mutex_unlock(&shared->_nodesMutexes[node]);
  }
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
void Thread::processThread(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects){
  bool procesado = true;
  if (_mstEjes.size() > 0){
    Eje eje = getNextEdge(shared);
    while (_mst.numVertices < shared->_g->numVertices){
      bool tome_mutex = false;
      if(pthread_mutex_trylock(&shared->_nodesMutexes.at(eje.nodoDestino)) == 0){
        tome_mutex = true;
        procesado = procesarNodo(eje, shared, threadObjects);
        pthread_mutex_unlock(&shared->_nodesMutexes.at(eje.nodoDestino));
      }
      if (_die){
        time_to_die();
      }
      procesado = procesado && tome_mutex;

      if (_request_queue.size() > 0){
        auto pair = _request_queue.front();
        _request_queue.pop();
        merge(pair, shared, threadObjects);
        if (_die){
          time_to_die();
        }
        eje = getNextEdge(shared);
        procesado = false;
      }

      eje = procesado ? getNextEdge(shared) : eje;
      procesado = true;
    }
  }
  if (getMst()->numVertices == shared->_g->numVertices)
  { // thread contiene todos los nodos del grafo ==> thread ganador
    if(_verbose){
      cout << "Printeando el grafo obtenido por el thread ganador" << endl;
    }
    pthread_mutex_lock(&shared->_mergeCounterMutex);
    getMst()->setCantMerges(shared->_mergeCounter);
    pthread_mutex_unlock(&shared->_mergeCounterMutex);
    getMst()->imprimirGrafo();
  }
  time_to_die();
}

void Thread::assignIdx(pthread_t threadCreationIdx){
  _threadCreationIdx = threadCreationIdx;
}

bool Thread::procesarNodo(Eje eje, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects){
  bool res = true;
  pthread_t node_color = shared->_nodeColorArray[eje.nodoDestino];

  if (((long)node_color) == -1){

    pintarNodo(eje, shared);
    pintarVecinos(shared, eje.nodoDestino);

    return res;
  } else {
    // Hay que mergear
    // Pido mi mutex para evitar que lleguen request mientras se resuelve mi merge
    bool merge_solved = false;
    if (pthread_mutex_trylock(&shared->_threadsMutexes.at(_threadCreationIdx)) == 0){
      if (pthread_mutex_trylock(&shared->_threadsMutexes.at(node_color)) == 0){
        if (node_color == shared->_nodeColorArray.at(eje.nodoDestino)){
          // hacer request
          Thread *other = &(*threadObjects).at(node_color);

          requestMerge(shared, threadObjects, other, eje, node_color); // Cambiar para que tome el eje

          while (!_merged){}
          merge_solved = true;
        }
        pthread_mutex_unlock(&shared->_threadsMutexes.at(node_color));
      } 
      pthread_mutex_unlock(&shared->_threadsMutexes.at(_threadCreationIdx));
      _merged = false;
    }
    return res && merge_solved;
  }
}

pthread_t Thread::getIdx(){
  return _threadCreationIdx;
}

Grafo *Thread::getMst(){
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
  return eje;
}

// Procurar agregar el thread con mayor id a la cola de fusiones del thread con menor id
void Thread::requestMerge(sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects, Thread *other, Eje eje, pthread_t node_color){

  // Se encola en el dueño del nodo
  other->_request_queue.push(make_pair((&((*threadObjects)[_threadCreationIdx])), eje));

}

void Thread::fagocitar(Thread *other, Eje eje, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects)
{
  // nodos
  for (auto const &x : other->_mst.listaDeAdyacencias)
  {
    _mst.insertarNodo(x.first);
  }

  // ejes del mst
  for (auto x : other->_mst.listaDeAdyacencias)
  {
    auto listaDeEjes = x.second;
    for (auto e : listaDeEjes)
    {
      _mst.insertarEje(e);
    }
  }

  _mst.insertarEje(eje.nodoOrigen, eje.nodoDestino, eje.peso);

  // pisar lista de adyacencias de other
  map<int, vector<Eje>> nuevaListaDeAdyacencias;
  other->_mst.listaDeAdyacencias = nuevaListaDeAdyacencias;

  priority_queue<int, vector<Eje>, Compare> newMstEjes;
  priority_queue<int, vector<Eje>, Compare> newOtherMstEjes;

  _mstEjes = newMstEjes;
  other->_mstEjes = newOtherMstEjes;

  // le pinto los nodos de mi color
  for (int i = 0; i < shared->_nodeColorArray.size(); ++i){
    if (shared->_nodeColorArray[i] == other->_threadCreationIdx)
    {
      shared->_nodeColorArray[i] = _threadCreationIdx;
    }
  }

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
          _mstEjes.push(x);
        }
      }
    }
  }

  other->_mst = Grafo();

  other->reiniciarThread(shared, threadObjects);
}

// Realizar la fusión
void Thread::merge(pair<Thread *, Eje> req, sharedData *shared, unordered_map<pthread_t, Thread> *threadObjects){
  /*pthread_mutex_lock(&(shared->_mergeCounterMutex));
  shared->_mergeCounter++;
  pthread_mutex_unlock(&(shared->_mergeCounterMutex));*/
 
  if (_threadCreationIdx < req.first->_threadCreationIdx){
    // yo ingiero al other
    fagocitar(req.first, req.second, shared, threadObjects);
  } else {
    // el other me ingiere
    req.first->fagocitar(&((*threadObjects)[_threadCreationIdx]), req.second, shared, threadObjects);
  }
  req.first->_merged = true;
}

// Gestión principal del thread. Contiene el ciclo que le permite a cada thread hacer sus funciones.
void *mstParaleloThread(void *p){

  pair<unordered_map<pthread_t, Thread>, sharedData> *sharedPair = (pair<unordered_map<pthread_t, Thread>, sharedData> *)p;
  sharedData *shared = &sharedPair->second;
  unordered_map<pthread_t, Thread> *threadObjects = &sharedPair->first;

  // Se obtiene el numero de thread
  pthread_t tid = pthread_self();

  pthread_mutex_t mu;
  pthread_mutex_lock(&(shared->_mapMutex));
  shared->_threadsMutexes.insert({tid, mu});
  pthread_mutex_init(&shared->_threadsMutexes.at(tid), NULL);

  pthread_mutex_lock(&(shared->_threadCreationMutex));
  (*threadObjects)[tid] = Thread();
  pthread_mutex_unlock(&(shared->_threadCreationMutex));
  (*threadObjects)[tid]._verbose= shared->_verbose;
  pthread_mutex_unlock(&(shared->_mapMutex));
  pthread_barrier_wait(&(shared->_initBarrier));
  (*threadObjects).at(tid).assignIdx(tid);

  pthread_mutex_lock(&(shared->_initMutex));

  (*threadObjects).at(tid).initThread(shared, threadObjects);

  pthread_mutex_unlock(&(shared->_initMutex));

  (*threadObjects)[tid].processThread(shared, threadObjects);
}

void mstParalelo(Grafo *g, int cantThreads)
{
  //Verificar cantidad de threads para ejecutar el algoritmo
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
    }

    return;
  }
  /*
  cout << "exp_result: qty_nodes = "<< g->numVertices << endl;
  cout << "exp_result: qty_edges = "<< g->numEjes << endl;
  */
  // Se crean los threads
  pthread_t threads[cantThreads];
  // Se inicializan las estructuras globales

  unordered_map<pthread_t, Thread> threadObjects;
  unordered_map<pthread_t, pthread_mutex_t> threadsMutexes;

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
  shared._nodeColorArray = nodeColorArray;
  shared._nodesMutexes = nodesMutexes;
  shared._threadsMutexes = threadsMutexes;
  shared._freeNodes = freeNodes;
  shared._verbose=_verbose;
  pthread_barrier_init(&(shared._initBarrier), NULL, cantThreads);
  pair<unordered_map<pthread_t, Thread>, sharedData> pair = make_pair(threadObjects, shared);

  // Se deben usar pthread_create y pthread_join.
  for (int threadIdx = 0; threadIdx < cantThreads; ++threadIdx)
  {
    pthread_create(&threads[threadIdx], NULL, mstParaleloThread, &pair); // TODO(charli): pasar todo lo que es memoria compartida
                                                                         //threadObjects[threadIdx] = Thread(threads[threadIdx], threadIdx);
  }
  for (int threadIdx = 0; threadIdx < cantThreads; ++threadIdx)
    pthread_join(threads[threadIdx], NULL);
}

void resetExperimentacion(){}

//Procedimiento para realizar las pruebas o test mínimo de la cátedra.

void experimentacion()
{
  imprimirResultado = true;
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

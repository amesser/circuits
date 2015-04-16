/*
 * worker.hpp
 *
 *  Created on: 17.03.2015
 *      Author: andi
 */

#include "pthread.h"
#include "semaphore.h"
#include "time.h"
#include <queue>

class Mutex
{
private:
  pthread_mutex_t _m;

public:
  Mutex();

  void lock()   {pthread_mutex_lock(&_m);}
  void unlock() {pthread_mutex_unlock(&_m);}
};

class MutexLocker
{
private:
  Mutex & _m;
public:
  MutexLocker(Mutex &m) : _m(m) {_m.lock();};
  ~MutexLocker()                {_m.unlock();}
};

class Semaphore
{
private:
  sem_t _s;
public:
  Semaphore();

  void put()
  {
    sem_post(&_s);
  }

  bool wait(unsigned int seconds)
  {
    struct timespec ts;

    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += seconds;

    int result = sem_timedwait(&_s, &ts);

    return result == 0;
  }
};

class Thread
{
private:
  pthread_t _t;
protected:
  volatile bool  _terminate;
public:
  Thread();

  virtual void run() = 0;

  void start();
  void terminate();

  static void* main(void* arg);
};

class Queue;

class Job
{
private:
  volatile bool _queued;

public:
  Job(void) : _queued(false) {};
  virtual ~Job () {}
  virtual void run() = 0;

  friend class Queue;
};

class Queue
{
private:
  Semaphore _Sem;
  Mutex     _QueueLock;

  ::std::queue<Job*> _Queue;
public:
  Queue();

  void  queue(Job *job);
  Job * get();
};

class Worker : public Thread
{
private:
  Queue & _queue;
public:
  Worker(Queue & queue);
  virtual void run();

};

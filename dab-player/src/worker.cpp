/*
 * worker.cpp
 *
 *  Created on: 17.03.2015
 *      Author: andi
 */

#include "worker.hpp"

Semaphore::Semaphore() :
  _s()
{
  sem_init(&_s, 0 , 0);
}

Mutex::Mutex() :
    _m(PTHREAD_MUTEX_INITIALIZER)
{

}

Queue::Queue() :
  _Sem(), _QueueLock(), _Queue()
{

}

void Queue::queue(Job * job)
{
  MutexLocker ml(_QueueLock);

  if(!job->_queued)
  {
    job->_queued = true;
    _Queue.push(job);
    _Sem.put();
  }
}

Job * Queue::get()
{
  Job * job = 0;

  if(_Sem.wait(1))
  {
    MutexLocker ml(_QueueLock);

    job = _Queue.front();
    _Queue.pop();

    job->_queued = false;
  }

  return job;
}

Thread::Thread() :
    _t()
{
}

void Thread::start()
{
  _terminate = false;
  pthread_create(&_t, NULL, main, this);
}

void Thread::terminate()
{
  _terminate = true;
  pthread_join(_t, NULL);
}

void* Thread::main(void *arg)
{
  Thread *self = reinterpret_cast<Thread*>(arg);
  self->run();

  return self;
}

Worker::Worker(Queue & queue) :
    Thread(),_queue(queue)
{

}

void Worker::run()
{
  while(!_terminate)
  {
    Job * job = _queue.get();

    if(job)
      job->run();
  }
}



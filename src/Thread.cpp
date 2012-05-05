
#include "Thread.h"

#ifndef WIN32
#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <errno.h>
#endif

using namespace net::ysuga;

Thread::Thread(void)
{

}

Thread::~Thread(void)
{

}


THREAD_ROUTINE StartRoutine(void* arg)
{
	Thread* threadObject = (Thread*)arg;
	threadObject->Run();
	threadObject->Exit(0);
#ifdef WIN32
	ExitThread(0);
#else 
	pthread_exit(0);
#endif
	return 0;
}


void Thread::Start()
{	
#ifdef WIN32
	m_Handle = CreateThread(NULL, 0, StartRoutine, (LPVOID)this, 0, &m_ThreadId);
#else
	int ret = pthread_create(&m_Handle, NULL, StartRoutine, (void*)this);
	if(ret != 0) {
	  perror("pthread_create");
	}
#endif
}

void Thread::Join()
{
#ifdef WIN32
	WaitForSingleObject(m_Handle, INFINITE);
#else
	void* retval;
	pthread_join(m_Handle, &retval);
#endif
}

void Thread::Sleep(unsigned long milliSeconds)
{
#ifdef WIN32
	::Sleep(milliSeconds);
#else
  struct timespec interval;
  interval.tv_sec = milliSeconds / 1000;
  interval.tv_nsec = (milliSeconds % 1000) * 1000000;
  nanosleep(&interval, NULL);
#endif
}

void Thread::Exit(unsigned long exitCode) {
#ifdef WIN32
	ExitThread(exitCode);
#else
	pthread_exit(0);
#endif
}

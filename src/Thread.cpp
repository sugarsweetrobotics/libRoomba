#include "Thread.h"

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

#ifdef WIN32
	ExitThread(0);
#else 

#endif
	return 0;
}


void Thread::Start()
{	
	m_Handle = CreateThread(NULL, 0, StartRoutine, (LPVOID)this, 0, &m_ThreadId);
}

void Thread::Join()
{
	WaitForSingleObject(m_Handle, INFINITE);
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

#endif
}

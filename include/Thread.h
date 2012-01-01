/********************************************************
 * SerialPort.h
 *
 * Portable Thread Class Library for Windows and Unix.
 * @author ysuga@ysuga.net
 * @date 2010/11/02
 ********************************************************/

#ifndef THREAD_HEADER_INCLUDED
#define THREAD_HEADER_INCLUDED

#ifdef WIN32
// 以下の ifdef ブロックは DLL からのエクスポートを容易にするマクロを作成するための 
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された LIBROOMBA_EXPORTS
// シンボルでコンパイルされます。このシンボルは、この DLL を使うプロジェクトで定義することはできません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// LIBREVAST_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。
#ifdef _USRDLL
#define LIBTHREAD_API __declspec(dllexport)
#else
#define LIBTHREAD_API __declspec(dllimport)
#endif

#endif // ifdef WIN32


#ifdef WIN32
#include <windows.h>
#define THREAD_ROUTINE DWORD WINAPI
#else

#endif


namespace net {
	namespace ysuga {

		class Thread
		{
		private:
#ifdef WIN32
			HANDLE m_Handle;
			DWORD m_ThreadId;
#else

#endif
		public:
			LIBTHREAD_API Thread(void);
			LIBTHREAD_API virtual ~Thread(void);

		public:
			LIBTHREAD_API void Start();

			LIBTHREAD_API virtual void Run() {};

			LIBTHREAD_API void Join();

			LIBTHREAD_API void Exit(unsigned long exitCode);

		public:
			LIBTHREAD_API static void Sleep(unsigned long milliSeconds);
		};

	};
};


#endif

/*******************************************************
 * Copyright  2010, ysuga.net all rights reserved.
 *******************************************************/
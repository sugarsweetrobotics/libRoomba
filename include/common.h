#ifndef LIBROOMBA_COMMON_HEADER_INCLUDED
#define LIBROOMBA_COMMON_HEADER_INCLUDED

#ifdef WIN32
// 以下の ifdef ブロックは DLL からのエクスポートを容易にするマクロを作成するための 
// 一般的な方法です。この DLL 内のすべてのファイルは、コマンド ラインで定義された LIBROOMBA_EXPORTS
// シンボルでコンパイルされます。このシンボルは、この DLL を使うプロジェクトで定義することはできません。
// ソースファイルがこのファイルを含んでいる他のプロジェクトは、 
// LIBREVAST_API 関数を DLL からインポートされたと見なすのに対し、この DLL は、このマクロで定義された
// シンボルをエクスポートされたと見なします。
#ifdef LIBROOMBA_EXPORTS
#define LIBROOMBA_API __declspec(dllexport)
#else
#ifdef LIBROOMBA_STATIC_EXPORTS
#define LIBROOMBA_API 
#else

#define LIBROOMBA_API __declspec(dllimport)
#endif // LIBROOMBA_STATIC_EXPORTS
#endif

#else 
#define LIBROOMBA_API 
#endif // ifdef WIN32





#endif // #ifndef COMMON_HEADER_INCLUDED

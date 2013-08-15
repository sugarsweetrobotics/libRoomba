#pragma once

#ifdef WIN32
#ifdef PCWRAPPER_EXPORTS
#define PCWRAPPER_API __declspec(dllexport)
#else
#ifdef PCWRAPPER_STATIC_EXPORTS
#define PCWRAPPER_API 
#else
#define PCWRAPPER_API __declspec(dllimport)
#endif
#endif
#else
#define PCWRAPPER_API
#endif



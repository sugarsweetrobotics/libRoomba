/***************************************************
 * ComAccessException.h
 *
 * Portable Serial Port Class Library for Windows and Unix.
 * @author ysuga@ysuga.net
 * @date 2010/11/02
 ********************************************************/

#ifndef COMACCESS_EXCEPTION_HEADER_INCLUDED
#define COMACCESS_EXCEPTION_HEADER_INCLUDED

#include "ComException.h"

namespace net {
  
  namespace ysuga {
    /**
     * @brief This exception is thrown when Accessing COM Port (Read/Write) is failed.
     */
    class  ComAccessException : public ComException {
    public:
      
      /** Constructor
       *
       */
    ComAccessException(void) : ComException("COM Access") {
      }
      
      /** Destructor
       *
       */
      ~ComAccessException(void) throw()	{
      }
    };
    
  }
}



#endif

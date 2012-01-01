/***************************************************
 * ComOpenException.h
 *
 * Portable Serial Port Class Library for Windows and Unix.
 * @author ysuga@ysuga.net
 * @date 2010/11/02
 ********************************************************/


#ifndef COM_OPEN_EXCEPTION_HEADER_INCLUDED
#define COM_OPEN_EXCEPTION_HEADER_INCLUDED



#include "ComException.h"


namespace net {
  namespace ysuga {
    
    /**
     * @brief This exception is thrown when Opening COM port is failed.
     */
    class ComOpenException : ComException  {
    public:
      
      /** Constructor.
       *
       */
    ComOpenException(void) : ComException ("COM Open Error") {
      }
      
      /** Destructor 
       *
       */
      ~ComOpenException(void) throw() {
      }
    };
  }
}



#endif

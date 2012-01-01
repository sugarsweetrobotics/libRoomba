/***************************************************
 * ComStateException.h
 *
 * Portable Serial Port Class Library for Windows and Unix.
 * @author ysuga@ysuga.net
 * @date 2010/11/02
 ********************************************************/


#ifndef COM_STATE_EXCEPTION_HEADER_INCLUDED
#define COM_STATE_EXCEPTION_HEADER_INCLUDED


#include "ComException.h"


namespace net {
  namespace ysuga {
    /**
     * @brief  This exception is thrown when COM port state is wrong.
     */
    class ComStateException : public ComException {
    public:
      
      /** Constructor
       *
       */
    ComStateException(void) : ComException ("COM State Exception") {
	}
      
      /** Destructor
       *
       */
      ~ComStateException(void) throw() {
	}
    };
  }
}


#endif

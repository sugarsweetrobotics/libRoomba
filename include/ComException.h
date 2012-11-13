#ifndef COM_EXCEPTION_HEADER_INCLUDED
#define COM_EXCEPTION_HEADER_INCLUDED

#include <exception>
#include <string>

namespace net {
  namespace ysuga {
class ComException : public std::exception {
 private:
  std::string msg;

 public:
  ComException(const char* msg) {
    this->msg = msg;
  }

  ~ComException() throw() {
  }

 public:
  virtual const char* what() const throw() {
    return msg.c_str();
  }
};

  }
}
#endif

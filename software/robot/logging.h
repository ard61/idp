#ifndef LOGGING_H_INCLUDED
#define LOGGING_H_INCLUDED

#include <iostream>
#include <iomanip>
#include <stopwatch.h>

namespace idp {

class logging {
public:
  static stopwatch logging_stopwatch;
  static void log_init() {
    logging_stopwatch.start();
  }
};
}

#define IDP_ERR std::cout << std::setw(6) << idp::logging::logging_stopwatch.read() << "ERROR  :  "
#define IDP_WARN std::cout << std::setw(6) << idp::logging::logging_stopwatch.read() << "WARNING:  "
#define IDP_INFO std::cout << std::setw(6) << idp::logging::logging_stopwatch.read() << "INFO:     "

#endif
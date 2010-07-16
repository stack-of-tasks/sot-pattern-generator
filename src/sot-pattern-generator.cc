/*
 *  Copyright
 */

#include <iostream>
#include "sot-pattern-generator/sot-pattern-generator.hh" 

// Select verbosity at configuration by setting CXXFLAGS="... -DDEBUG=[1 or 2]"
#if DEBUG==2
#define ODEBUG2(x) std::cerr << "sot-pattern-generator:" << x << std::endl
#define ODEBUG1(x) std::cerr << "sot-pattern-generator:" << x << std::endl
#elif DEBUG==1
#define ODEBUG2(x)
#define ODEBUG1(x) std::cerr << "sot-pattern-generator:" << x << std::endl
#else
#define ODEBUG2(x)
#define ODEBUG1(x)
#endif

using namespace sot;

Pattern::Pattern()
{
  ODEBUG2("message of level 2");
  ODEBUG1("message of level 1");
}

Pattern::~Pattern()
{
}

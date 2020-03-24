/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet JRL-Japan, 2008
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      ExceptionPatternGenerator.cpp
 * Project:   SOT
 * Author:    Olivier Stasse
 *
 * Version control
 * ===============
 *
 *  $Id$
 *
 * Description
 * ============
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <sot/pattern-generator/exception-pg.h>
#include <stdarg.h>
#include <cstdio>

namespace dynamicgraph {
namespace sot {

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

const std::string ExceptionPatternGenerator::EXCEPTION_NAME =
    "PatternGenerator";

ExceptionPatternGenerator::ExceptionPatternGenerator(
    const ExceptionPatternGenerator::ErrorCodeEnum &errcode,
    const std::string &msg)
    : ExceptionAbstract(errcode, msg) {}

ExceptionPatternGenerator::ExceptionPatternGenerator(
    const ExceptionPatternGenerator::ErrorCodeEnum &errcode,
    const std::string &msg, const char *format, ...)
    : ExceptionAbstract(errcode, msg) {
  va_list args;
  va_start(args, format);

  const unsigned int SIZE = 256;
  char buffer[SIZE];
  vsnprintf(buffer, SIZE, format, args);

  message += buffer;

  va_end(args);
}

}  // namespace sot
}  // namespace dynamicgraph

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

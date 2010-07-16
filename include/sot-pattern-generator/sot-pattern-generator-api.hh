/*
 *  Copyright
 */

#ifndef SOT_PATTERN_GENERATOR_API_HH
#define SOT_PATTERN_GENERATOR_API_HH

#if defined (WIN32)
#  ifdef SOT_PATTERN_GENERATOR_EXPORTS
#    define SOT_PATTERN_GENERATOR_EXPORT __declspec(dllexport)
#  else
#    define SOT_PATTERN_GENERATOR_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOT_PATTERN_GENERATOR_EXPORT
#endif

#endif

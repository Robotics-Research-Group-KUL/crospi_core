//  Boost compiler configuration selection header file

//  (C) Copyright John Maddock 2001 - 2003. 
//  (C) Copyright Martin Wille 2003.
//  (C) Copyright Guillaume Melquiond 2003.
//
//  Distributed under the Boost Software License, Version 1.0.
//  (See accompanying file LICENSE_1_0.txt or copy at
//   http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org/ for most recent version.

// locate which compiler we are using and define
// CROSPI_BOOST_COMPILER_CONFIG as needed: 

#if defined __CUDACC__
//  NVIDIA CUDA C++ compiler for GPU
#   include "crospi_boost/config/compiler/nvcc.hpp"

#endif

#if defined(__GCCXML__)
// GCC-XML emulates other compilers, it has to appear first here!
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/gcc_xml.hpp"

#elif defined(_CRAYC)
// EDG based Cray compiler:
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/cray.hpp"

#elif defined __COMO__
//  Comeau C++
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/comeau.hpp"

#elif defined(__PATHSCALE__) && (__PATHCC__ >= 4)
// PathScale EKOPath compiler (has to come before clang and gcc)
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/pathscale.hpp"

#elif defined(__INTEL_COMPILER) || defined(__ICL) || defined(__ICC) || defined(__ECC)
//  Intel
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/intel.hpp"

#elif defined __clang__ && !defined(__ibmxl__) && !defined(__CODEGEARC__)
//  Clang C++ emulates GCC, so it has to appear early.
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/clang.hpp"

#elif defined __DMC__
//  Digital Mars C++
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/digitalmars.hpp"

#elif defined __DCC__
//  Wind River Diab C++
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/diab.hpp"

#elif defined(__PGI)
//  Portland Group Inc.
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/pgi.hpp"

# elif defined(__GNUC__) && !defined(__ibmxl__)
//  GNU C++:
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/gcc.hpp"

#elif defined __KCC
//  Kai C++
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/kai.hpp"

#elif defined __sgi
//  SGI MIPSpro C++
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/sgi_mipspro.hpp"

#elif defined __DECCXX
//  Compaq Tru64 Unix cxx
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/compaq_cxx.hpp"

#elif defined __ghs
//  Greenhills C++
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/greenhills.hpp"

#elif defined __CODEGEARC__
//  CodeGear - must be checked for before Borland
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/codegear.hpp"

#elif defined __BORLANDC__
//  Borland
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/borland.hpp"

#elif defined  __MWERKS__
//  Metrowerks CodeWarrior
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/metrowerks.hpp"

#elif defined  __SUNPRO_CC
//  Sun Workshop Compiler C++
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/sunpro_cc.hpp"

#elif defined __HP_aCC
//  HP aCC
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/hp_acc.hpp"

#elif defined(__MRC__) || defined(__SC__)
//  MPW MrCpp or SCpp
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/mpw.hpp"

#elif defined(__IBMCPP__) && defined(__COMPILER_VER__) && defined(__MVS__)
//  IBM z/OS XL C/C++
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/xlcpp_zos.hpp"

#elif defined(__ibmxl__)
//  IBM XL C/C++ for Linux (Little Endian)
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/xlcpp.hpp"

#elif defined(__IBMCPP__)
//  IBM Visual Age or IBM XL C/C++ for Linux (Big Endian)
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/vacpp.hpp"

#elif defined _MSC_VER
//  Microsoft Visual C++
//
//  Must remain the last #elif since some other vendors (Metrowerks, for
//  example) also #define _MSC_VER
#   define CROSPI_BOOST_COMPILER_CONFIG "crospi_boost/config/compiler/visualc.hpp"

#elif defined (CROSPI_BOOST_ASSERT_CONFIG)
// this must come last - generate an error if we don't
// recognise the compiler:
#  error "Unknown compiler - please configure (http://www.boost.org/libs/config/config.htm#configuring) and report the results to the main boost mailing list (http://www.boost.org/more/mailing_lists.htm#main)"

#endif

#if 0
//
// This section allows dependency scanners to find all the headers we *might* include:
//
#include <crospi_boost/config/compiler/gcc_xml.hpp>
#include <crospi_boost/config/compiler/cray.hpp>
#include <crospi_boost/config/compiler/comeau.hpp>
#include <crospi_boost/config/compiler/pathscale.hpp>
#include <crospi_boost/config/compiler/intel.hpp>
#include <crospi_boost/config/compiler/clang.hpp>
#include <crospi_boost/config/compiler/digitalmars.hpp>
#include <crospi_boost/config/compiler/gcc.hpp>
#include <crospi_boost/config/compiler/kai.hpp>
#include <crospi_boost/config/compiler/sgi_mipspro.hpp>
#include <crospi_boost/config/compiler/compaq_cxx.hpp>
#include <crospi_boost/config/compiler/greenhills.hpp>
#include <crospi_boost/config/compiler/codegear.hpp>
#include <crospi_boost/config/compiler/borland.hpp>
#include <crospi_boost/config/compiler/metrowerks.hpp>
#include <crospi_boost/config/compiler/sunpro_cc.hpp>
#include <crospi_boost/config/compiler/hp_acc.hpp>
#include <crospi_boost/config/compiler/mpw.hpp>
#include <crospi_boost/config/compiler/xlcpp_zos.hpp>
#include <crospi_boost/config/compiler/xlcpp.hpp>
#include <crospi_boost/config/compiler/vacpp.hpp>
#include <crospi_boost/config/compiler/pgi.hpp>
#include <crospi_boost/config/compiler/visualc.hpp>

#endif


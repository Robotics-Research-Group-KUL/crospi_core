
#ifndef CROSPI_BOOST_MPL_AUX_ADL_BARRIER_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2002-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/config/adl.hpp>
#include <crospi_boost/mpl/aux_/config/gcc.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_ADL_BARRIER_NAMESPACE)

#   define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE crospi_mpl_
#   define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_OPEN namespace crospi_mpl_ {
#   define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_CLOSE }
#   define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_DECL(type) \
    namespace crospi_boost { namespace mpl { \
    using ::CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::type; \
    } } \
/**/

#if !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE)
namespace CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE { namespace aux {} }
namespace crospi_boost { namespace mpl { using namespace CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE; 
namespace aux { using namespace CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE::aux; }
}}
#endif

#else // CROSPI_BOOST_MPL_CFG_NO_ADL_BARRIER_NAMESPACE

#   define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE crospi_boost::mpl
#   define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_OPEN namespace crospi_boost { namespace mpl {
#   define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_NAMESPACE_CLOSE }}
#   define CROSPI_BOOST_MPL_AUX_ADL_BARRIER_DECL(type) /**/

#endif

#endif // CROSPI_BOOST_MPL_AUX_ADL_BARRIER_HPP_INCLUDED

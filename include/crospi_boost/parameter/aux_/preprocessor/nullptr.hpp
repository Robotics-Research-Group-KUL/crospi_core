// Copyright Cromwell D. Enage 2019.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PREPROCESSOR_NULLPTR_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PREPROCESSOR_NULLPTR_HPP

#include <crospi_boost/config.hpp>

#if defined(CROSPI_BOOST_NO_CXX11_NULLPTR)
#define CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR 0
#else
#define CROSPI_BOOST_PARAMETER_AUX_PP_NULLPTR nullptr
#endif

#endif  // include guard


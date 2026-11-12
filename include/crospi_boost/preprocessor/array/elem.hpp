# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002.
#  *     Distributed under the Boost Software License, Version 1.0. (See
#  *     accompanying file LICENSE_1_0.txt or copy at
#  *     http://www.boost.org/LICENSE_1_0.txt)
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_ARRAY_ELEM_HPP
# define CROSPI_BOOST_PREPROCESSOR_ARRAY_ELEM_HPP
#
# include <crospi_boost/preprocessor/array/data.hpp>
# include <crospi_boost/preprocessor/array/size.hpp>
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/tuple/elem.hpp>
#
# /* CROSPI_BOOST_PP_ARRAY_ELEM */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_ARRAY_ELEM(i, array) CROSPI_BOOST_PP_TUPLE_ELEM(CROSPI_BOOST_PP_ARRAY_SIZE(array), i, CROSPI_BOOST_PP_ARRAY_DATA(array))
# else
#    define CROSPI_BOOST_PP_ARRAY_ELEM(i, array) CROSPI_BOOST_PP_ARRAY_ELEM_I(i, array)
#    define CROSPI_BOOST_PP_ARRAY_ELEM_I(i, array) CROSPI_BOOST_PP_TUPLE_ELEM(CROSPI_BOOST_PP_ARRAY_SIZE(array), i, CROSPI_BOOST_PP_ARRAY_DATA(array))
# endif
#
# endif

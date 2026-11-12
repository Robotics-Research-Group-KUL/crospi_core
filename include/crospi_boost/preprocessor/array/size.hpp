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
# ifndef CROSPI_BOOST_PREPROCESSOR_ARRAY_SIZE_HPP
# define CROSPI_BOOST_PREPROCESSOR_ARRAY_SIZE_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/tuple/elem.hpp>
#
# /* CROSPI_BOOST_PP_ARRAY_SIZE */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_ARRAY_SIZE(array) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, array)
# else
#    define CROSPI_BOOST_PP_ARRAY_SIZE(array) CROSPI_BOOST_PP_ARRAY_SIZE_I(array)
#    define CROSPI_BOOST_PP_ARRAY_SIZE_I(array) CROSPI_BOOST_PP_ARRAY_SIZE_II array
#    define CROSPI_BOOST_PP_ARRAY_SIZE_II(size, data) size
# endif
#
# endif

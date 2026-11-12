# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2011.                                  *
#  *     (C) Copyright Edward Diener 2011.                                    *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_FACILITIES_OVERLOAD_HPP
# define CROSPI_BOOST_PREPROCESSOR_FACILITIES_OVERLOAD_HPP
#
# include <crospi_boost/preprocessor/cat.hpp>
# include <crospi_boost/preprocessor/variadic/size.hpp>
#
# /* CROSPI_BOOST_PP_OVERLOAD */
#
# define CROSPI_BOOST_PP_OVERLOAD(prefix, ...) CROSPI_BOOST_PP_CAT(prefix, CROSPI_BOOST_PP_VARIADIC_SIZE(__VA_ARGS__))
#
# endif

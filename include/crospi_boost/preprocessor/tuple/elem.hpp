# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  */
#
# /* Revised by Paul Mensonides (2002-2011) */
# /* Revised by Edward Diener (2011,2014,2020) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_TUPLE_ELEM_HPP
# define CROSPI_BOOST_PREPROCESSOR_TUPLE_ELEM_HPP
#
# include <crospi_boost/preprocessor/cat.hpp>
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/facilities/expand.hpp>
# include <crospi_boost/preprocessor/facilities/overload.hpp>
# include <crospi_boost/preprocessor/tuple/rem.hpp>
# include <crospi_boost/preprocessor/variadic/elem.hpp>
# include <crospi_boost/preprocessor/tuple/detail/is_single_return.hpp>
#
# if CROSPI_BOOST_PP_VARIADICS_MSVC
#     define CROSPI_BOOST_PP_TUPLE_ELEM(...) CROSPI_BOOST_PP_TUPLE_ELEM_I(CROSPI_BOOST_PP_OVERLOAD(CROSPI_BOOST_PP_TUPLE_ELEM_O_, __VA_ARGS__), (__VA_ARGS__))
#     define CROSPI_BOOST_PP_TUPLE_ELEM_I(m, args) CROSPI_BOOST_PP_TUPLE_ELEM_II(m, args)
#     define CROSPI_BOOST_PP_TUPLE_ELEM_II(m, args) CROSPI_BOOST_PP_CAT(m ## args,)
/*
  Use CROSPI_BOOST_PP_REM_CAT if it is a single element tuple ( which might be empty )
  else use CROSPI_BOOST_PP_REM. This fixes a VC++ problem with an empty tuple and CROSPI_BOOST_PP_TUPLE_ELEM
  functionality. See tuple_elem_bug_test.cxx.
*/
#     define CROSPI_BOOST_PP_TUPLE_ELEM_O_2(n, tuple) \
         CROSPI_BOOST_PP_VARIADIC_ELEM(n, CROSPI_BOOST_PP_EXPAND(CROSPI_BOOST_PP_TUPLE_IS_SINGLE_RETURN(CROSPI_BOOST_PP_REM_CAT,CROSPI_BOOST_PP_REM,tuple) tuple)) \
         /**/
# else
#     define CROSPI_BOOST_PP_TUPLE_ELEM(...) CROSPI_BOOST_PP_OVERLOAD(CROSPI_BOOST_PP_TUPLE_ELEM_O_, __VA_ARGS__)(__VA_ARGS__)
#     define CROSPI_BOOST_PP_TUPLE_ELEM_O_2(n, tuple) CROSPI_BOOST_PP_VARIADIC_ELEM(n, CROSPI_BOOST_PP_REM tuple)
# endif
# define CROSPI_BOOST_PP_TUPLE_ELEM_O_3(size, n, tuple) CROSPI_BOOST_PP_TUPLE_ELEM_O_2(n, tuple)
#
# /* directly used elsewhere in Boost... */
#
# define CROSPI_BOOST_PP_TUPLE_ELEM_1_0(a) a
#
# define CROSPI_BOOST_PP_TUPLE_ELEM_2_0(a, b) a
# define CROSPI_BOOST_PP_TUPLE_ELEM_2_1(a, b) b
#
# define CROSPI_BOOST_PP_TUPLE_ELEM_3_0(a, b, c) a
# define CROSPI_BOOST_PP_TUPLE_ELEM_3_1(a, b, c) b
# define CROSPI_BOOST_PP_TUPLE_ELEM_3_2(a, b, c) c
#
# endif

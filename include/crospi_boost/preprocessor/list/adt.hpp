# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  *
#  * See http://www.boost.org for most recent version.
#  */
#
# /* Revised by Paul Mensonides (2002) */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_LIST_ADT_HPP
# define CROSPI_BOOST_PREPROCESSOR_LIST_ADT_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
# include <crospi_boost/preprocessor/detail/is_binary.hpp>
# include <crospi_boost/preprocessor/logical/compl.hpp>
# include <crospi_boost/preprocessor/tuple/eat.hpp>
#
# /* CROSPI_BOOST_PP_LIST_CONS */
#
# define CROSPI_BOOST_PP_LIST_CONS(head, tail) (head, tail)
#
# /* CROSPI_BOOST_PP_LIST_NIL */
#
# define CROSPI_BOOST_PP_LIST_NIL CROSPI_BOOST_PP_NIL
#
# /* CROSPI_BOOST_PP_LIST_FIRST */
#
# define CROSPI_BOOST_PP_LIST_FIRST(list) CROSPI_BOOST_PP_LIST_FIRST_D(list)
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_LIST_FIRST_D(list) CROSPI_BOOST_PP_LIST_FIRST_I list
# else
#    define CROSPI_BOOST_PP_LIST_FIRST_D(list) CROSPI_BOOST_PP_LIST_FIRST_I ## list
# endif
#
# define CROSPI_BOOST_PP_LIST_FIRST_I(head, tail) head
#
# /* CROSPI_BOOST_PP_LIST_REST */
#
# define CROSPI_BOOST_PP_LIST_REST(list) CROSPI_BOOST_PP_LIST_REST_D(list)
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_LIST_REST_D(list) CROSPI_BOOST_PP_LIST_REST_I list
# else
#    define CROSPI_BOOST_PP_LIST_REST_D(list) CROSPI_BOOST_PP_LIST_REST_I ## list
# endif
#
# define CROSPI_BOOST_PP_LIST_REST_I(head, tail) tail
#
# /* CROSPI_BOOST_PP_LIST_IS_CONS */
#
# if CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_BCC()
#    define CROSPI_BOOST_PP_LIST_IS_CONS(list) CROSPI_BOOST_PP_LIST_IS_CONS_D(list)
#    define CROSPI_BOOST_PP_LIST_IS_CONS_D(list) CROSPI_BOOST_PP_LIST_IS_CONS_ ## list
#    define CROSPI_BOOST_PP_LIST_IS_CONS_(head, tail) 1
#    define CROSPI_BOOST_PP_LIST_IS_CONS_CROSPI_BOOST_PP_NIL 0
# else
#    define CROSPI_BOOST_PP_LIST_IS_CONS(list) CROSPI_BOOST_PP_IS_BINARY(list)
# endif
#
# /* CROSPI_BOOST_PP_LIST_IS_NIL */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_BCC()
#    define CROSPI_BOOST_PP_LIST_IS_NIL(list) CROSPI_BOOST_PP_COMPL(CROSPI_BOOST_PP_IS_BINARY(list))
# else
#    define CROSPI_BOOST_PP_LIST_IS_NIL(list) CROSPI_BOOST_PP_COMPL(CROSPI_BOOST_PP_LIST_IS_CONS(list))
# endif
#
# endif

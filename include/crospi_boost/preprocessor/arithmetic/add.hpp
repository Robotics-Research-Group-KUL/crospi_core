# /* Copyright (C) 2001
#  * Housemarque Oy
#  * http://www.housemarque.com
#  *
#  * Distributed under the Boost Software License, Version 1.0. (See
#  * accompanying file LICENSE_1_0.txt or copy at
#  * http://www.boost.org/LICENSE_1_0.txt)
#  */
#
# /* Revised by Paul Mensonides (2002) */
# /* Revised by Edward Diener (2020) */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_ARITHMETIC_ADD_HPP
# define CROSPI_BOOST_PREPROCESSOR_ARITHMETIC_ADD_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_STRICT()
#
# include <crospi_boost/preprocessor/arithmetic/dec.hpp>
# include <crospi_boost/preprocessor/arithmetic/inc.hpp>
# include <crospi_boost/preprocessor/control/while.hpp>
# include <crospi_boost/preprocessor/tuple/elem.hpp>
#
# /* CROSPI_BOOST_PP_ADD */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_ADD(x, y) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, CROSPI_BOOST_PP_WHILE(CROSPI_BOOST_PP_ADD_P, CROSPI_BOOST_PP_ADD_O, (x, y)))
# else
#    define CROSPI_BOOST_PP_ADD(x, y) CROSPI_BOOST_PP_ADD_I(x, y)
#    define CROSPI_BOOST_PP_ADD_I(x, y) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, CROSPI_BOOST_PP_WHILE(CROSPI_BOOST_PP_ADD_P, CROSPI_BOOST_PP_ADD_O, (x, y)))
# endif
#
# define CROSPI_BOOST_PP_ADD_P(d, xy) CROSPI_BOOST_PP_TUPLE_ELEM(2, 1, xy)
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_ADD_O(d, xy) CROSPI_BOOST_PP_ADD_O_I xy
# else
#    define CROSPI_BOOST_PP_ADD_O(d, xy) CROSPI_BOOST_PP_ADD_O_I(CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, xy), CROSPI_BOOST_PP_TUPLE_ELEM(2, 1, xy))
# endif
#
# define CROSPI_BOOST_PP_ADD_O_I(x, y) (CROSPI_BOOST_PP_INC(x), CROSPI_BOOST_PP_DEC(y))
#
# /* CROSPI_BOOST_PP_ADD_D */
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_ADD_D(d, x, y) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, CROSPI_BOOST_PP_WHILE_ ## d(CROSPI_BOOST_PP_ADD_P, CROSPI_BOOST_PP_ADD_O, (x, y)))
# else
#    define CROSPI_BOOST_PP_ADD_D(d, x, y) CROSPI_BOOST_PP_ADD_D_I(d, x, y)
#    define CROSPI_BOOST_PP_ADD_D_I(d, x, y) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, CROSPI_BOOST_PP_WHILE_ ## d(CROSPI_BOOST_PP_ADD_P, CROSPI_BOOST_PP_ADD_O, (x, y)))
# endif
#
# else
#
# include <crospi_boost/preprocessor/arithmetic/dec.hpp>
# include <crospi_boost/preprocessor/arithmetic/inc.hpp>
# include <crospi_boost/preprocessor/control/iif.hpp>
# include <crospi_boost/preprocessor/control/while.hpp>
# include <crospi_boost/preprocessor/facilities/identity.hpp>
# include <crospi_boost/preprocessor/logical/bitand.hpp>
# include <crospi_boost/preprocessor/logical/bitor.hpp>
# include <crospi_boost/preprocessor/logical/bool.hpp>
# include <crospi_boost/preprocessor/logical/compl.hpp>
# include <crospi_boost/preprocessor/tuple/elem.hpp>
# include <crospi_boost/preprocessor/arithmetic/detail/is_maximum_number.hpp>
# include <crospi_boost/preprocessor/arithmetic/detail/is_minimum_number.hpp>
#
# /* CROSPI_BOOST_PP_ADD */
#
#    define CROSPI_BOOST_PP_ADD(x, y) CROSPI_BOOST_PP_IIF(CROSPI_BOOST_PP_BITOR(CROSPI_BOOST_PP_DETAIL_IS_MAXIMUM_NUMBER(y),CROSPI_BOOST_PP_DETAIL_IS_MINIMUM_NUMBER(x)),CROSPI_BOOST_PP_IDENTITY_N(y,2),CROSPI_BOOST_PP_ADD_DO)(x,y)
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_ADD_DO(x, y) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, CROSPI_BOOST_PP_WHILE(CROSPI_BOOST_PP_ADD_P, CROSPI_BOOST_PP_ADD_O, (x, y)))
# else
#    define CROSPI_BOOST_PP_ADD_DO(x, y) CROSPI_BOOST_PP_ADD_I(x, y)
#    define CROSPI_BOOST_PP_ADD_I(x, y) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, CROSPI_BOOST_PP_WHILE(CROSPI_BOOST_PP_ADD_P, CROSPI_BOOST_PP_ADD_O, (x, y)))
# endif
#
# define CROSPI_BOOST_PP_ADD_P(d, xy) CROSPI_BOOST_PP_BITAND(CROSPI_BOOST_PP_BOOL(CROSPI_BOOST_PP_TUPLE_ELEM(2, 1, xy)),CROSPI_BOOST_PP_COMPL(CROSPI_BOOST_PP_DETAIL_IS_MAXIMUM_NUMBER(CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, xy))))
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_MWCC()
#    define CROSPI_BOOST_PP_ADD_O(d, xy) CROSPI_BOOST_PP_ADD_O_I xy
# else
#    define CROSPI_BOOST_PP_ADD_O(d, xy) CROSPI_BOOST_PP_ADD_O_I(CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, xy), CROSPI_BOOST_PP_TUPLE_ELEM(2, 1, xy))
# endif
#
# define CROSPI_BOOST_PP_ADD_O_I(x, y) (CROSPI_BOOST_PP_INC(x), CROSPI_BOOST_PP_DEC(y))
#
# /* CROSPI_BOOST_PP_ADD_D */
#
#    define CROSPI_BOOST_PP_ADD_D(d, x, y) CROSPI_BOOST_PP_IIF(CROSPI_BOOST_PP_BITOR(CROSPI_BOOST_PP_DETAIL_IS_MAXIMUM_NUMBER(y),CROSPI_BOOST_PP_DETAIL_IS_MINIMUM_NUMBER(x)),CROSPI_BOOST_PP_IDENTITY_N(y,3),CROSPI_BOOST_PP_ADD_DO_D)(d,x,y)
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    define CROSPI_BOOST_PP_ADD_DO_D(d, x, y) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, CROSPI_BOOST_PP_WHILE_ ## d(CROSPI_BOOST_PP_ADD_P, CROSPI_BOOST_PP_ADD_O, (x, y)))
# else
#    define CROSPI_BOOST_PP_ADD_DO_D(d, x, y) CROSPI_BOOST_PP_ADD_D_I(d, x, y)
#    define CROSPI_BOOST_PP_ADD_D_I(d, x, y) CROSPI_BOOST_PP_TUPLE_ELEM(2, 0, CROSPI_BOOST_PP_WHILE_ ## d(CROSPI_BOOST_PP_ADD_P, CROSPI_BOOST_PP_ADD_O, (x, y)))
# endif
#
# endif
#
# endif

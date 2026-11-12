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
# ifndef CROSPI_BOOST_PREPROCESSOR_LIST_FOLD_RIGHT_HPP
# define CROSPI_BOOST_PREPROCESSOR_LIST_FOLD_RIGHT_HPP
#
# include <crospi_boost/preprocessor/config/config.hpp>
#
# if ~CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_STRICT()
#
# include <crospi_boost/preprocessor/cat.hpp>
# include <crospi_boost/preprocessor/control/while.hpp>
# include <crospi_boost/preprocessor/debug/error.hpp>
# include <crospi_boost/preprocessor/detail/auto_rec.hpp>
#
# if 0
#    define CROSPI_BOOST_PP_LIST_FOLD_RIGHT(op, state, list)
# endif
#
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_LIST_FOLD_RIGHT_, CROSPI_BOOST_PP_AUTO_REC(CROSPI_BOOST_PP_WHILE_P, 256))
#
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_257(o, s, l) CROSPI_BOOST_PP_ERROR(0x0004)
#
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_D(d, o, s, l) CROSPI_BOOST_PP_LIST_FOLD_RIGHT_ ## d(o, s, l)
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_2ND CROSPI_BOOST_PP_LIST_FOLD_RIGHT
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_2ND_D CROSPI_BOOST_PP_LIST_FOLD_RIGHT_D
#
# if CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    include <crospi_boost/preprocessor/list/detail/edg/fold_right.hpp>
# else
#    include <crospi_boost/preprocessor/list/detail/fold_right.hpp>
# endif
#
# else
#
# include <crospi_boost/preprocessor/arithmetic/dec.hpp>
# include <crospi_boost/preprocessor/cat.hpp>
# include <crospi_boost/preprocessor/control/while.hpp>
# include <crospi_boost/preprocessor/debug/error.hpp>
# include <crospi_boost/preprocessor/detail/auto_rec.hpp>
#
# if 0
#    define CROSPI_BOOST_PP_LIST_FOLD_RIGHT(op, state, list)
# endif
#
# include <crospi_boost/preprocessor/config/limits.hpp>
#
# if CROSPI_BOOST_PP_LIMIT_WHILE == 256
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_LIST_FOLD_RIGHT_, CROSPI_BOOST_PP_DEC(CROSPI_BOOST_PP_AUTO_REC(CROSPI_BOOST_PP_WHILE_P, 256)))
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_257(o, s, l) CROSPI_BOOST_PP_ERROR(0x0004)
# elif CROSPI_BOOST_PP_LIMIT_WHILE == 512
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_LIST_FOLD_RIGHT_, CROSPI_BOOST_PP_DEC(CROSPI_BOOST_PP_AUTO_REC(CROSPI_BOOST_PP_WHILE_P, 512)))
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_513(o, s, l) CROSPI_BOOST_PP_ERROR(0x0004)
# elif CROSPI_BOOST_PP_LIMIT_WHILE == 1024
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT CROSPI_BOOST_PP_CAT(CROSPI_BOOST_PP_LIST_FOLD_RIGHT_, CROSPI_BOOST_PP_DEC(CROSPI_BOOST_PP_AUTO_REC(CROSPI_BOOST_PP_WHILE_P, 1024)))
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_1025(o, s, l) CROSPI_BOOST_PP_ERROR(0x0004)
# else
# error Incorrect value for the CROSPI_BOOST_PP_LIMIT_WHILE limit
# endif
#
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_D(d, o, s, l) CROSPI_BOOST_PP_LIST_FOLD_RIGHT_ ## d(o, s, l)
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_2ND CROSPI_BOOST_PP_LIST_FOLD_RIGHT
# define CROSPI_BOOST_PP_LIST_FOLD_RIGHT_2ND_D CROSPI_BOOST_PP_LIST_FOLD_RIGHT_D
#
# if CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_EDG()
#    include <crospi_boost/preprocessor/list/detail/edg/fold_right.hpp>
# else
#    include <crospi_boost/preprocessor/list/detail/fold_right.hpp>
# endif
#
# endif
#
# endif

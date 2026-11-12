# /* **************************************************************************
#  *                                                                          *
#  *     (C) Copyright Paul Mensonides 2002-2011.                             *
#  *     (C) Copyright Edward Diener 2011-2020.                               *
#  *     Distributed under the Boost Software License, Version 1.0. (See      *
#  *     accompanying file LICENSE_1_0.txt or copy at                         *
#  *     http://www.boost.org/LICENSE_1_0.txt)                                *
#  *                                                                          *
#  ************************************************************************** */
#
# /* See http://www.boost.org for most recent version. */
#
# ifndef CROSPI_BOOST_PREPROCESSOR_CONFIG_CONFIG_HPP
# define CROSPI_BOOST_PREPROCESSOR_CONFIG_CONFIG_HPP
#
# /* CROSPI_BOOST_PP_CONFIG_FLAGS */
#
# define CROSPI_BOOST_PP_CONFIG_STRICT() 0x0001
# define CROSPI_BOOST_PP_CONFIG_IDEAL() 0x0002
#
# define CROSPI_BOOST_PP_CONFIG_MSVC() 0x0004
# define CROSPI_BOOST_PP_CONFIG_MWCC() 0x0008
# define CROSPI_BOOST_PP_CONFIG_BCC() 0x0010
# define CROSPI_BOOST_PP_CONFIG_EDG() 0x0020
# define CROSPI_BOOST_PP_CONFIG_DMC() 0x0040
#
# ifndef CROSPI_BOOST_PP_CONFIG_FLAGS
#    if defined(__GCCXML__) || defined(__WAVE__) || defined(__MWERKS__) && __MWERKS__ >= 0x3200
#        define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_STRICT())
#    elif defined(__EDG__) || defined(__EDG_VERSION__)
#        if defined(_MSC_VER) && !defined(__clang__) && (defined(__INTELLISENSE__) || __EDG_VERSION__ >= 308)
#           if !defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL
#               define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_MSVC())
#           else
#               define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_STRICT())
#           endif
#        else
#            define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_EDG() | CROSPI_BOOST_PP_CONFIG_STRICT())
#        endif
#    elif defined(_MSC_VER) && defined(__clang__)
#        define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_STRICT())
#    elif defined(__MWERKS__)
#        define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_MWCC())
#    elif defined(__DMC__)
#        define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_DMC())
#    elif defined(__BORLANDC__) && __BORLANDC__ >= 0x581
#        define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_STRICT())
#    elif defined(__BORLANDC__) || defined(__IBMC__) || defined(__IBMCPP__) || defined(__SUNPRO_CC)
#        define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_BCC())
#    elif defined(_MSC_VER)
#        if !defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL
#           define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_MSVC())
#        else
#           define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_STRICT())
#        endif
#    else
#        define CROSPI_BOOST_PP_CONFIG_FLAGS() (CROSPI_BOOST_PP_CONFIG_STRICT())
#    endif
# endif
#
# /* CROSPI_BOOST_PP_CONFIG_EXTENDED_LINE_INFO */
#
# ifndef CROSPI_BOOST_PP_CONFIG_EXTENDED_LINE_INFO
#    define CROSPI_BOOST_PP_CONFIG_EXTENDED_LINE_INFO 0
# endif
#
# /* CROSPI_BOOST_PP_CONFIG_ERRORS */
#
# ifndef CROSPI_BOOST_PP_CONFIG_ERRORS
#    ifdef NDEBUG
#        define CROSPI_BOOST_PP_CONFIG_ERRORS 0
#    else
#        define CROSPI_BOOST_PP_CONFIG_ERRORS 1
#    endif
# endif
#
# /* CROSPI_BOOST_PP_VARIADICS */
#
# if defined CROSPI_BOOST_PP_VARIADICS
#    undef CROSPI_BOOST_PP_VARIADICS
# endif
# if defined CROSPI_BOOST_PP_VARIADICS_MSVC
#    undef CROSPI_BOOST_PP_VARIADICS_MSVC
# endif
# define CROSPI_BOOST_PP_VARIADICS 1
# if defined _MSC_VER && _MSC_VER >= 1400 && !defined(__clang__) && (defined(__INTELLISENSE__) || (defined(__INTEL_COMPILER) && __INTEL_COMPILER >= 1700) || !(defined __EDG__ || defined __GCCXML__ || defined __PATHSCALE__ || defined __DMC__ || defined __CODEGEARC__ || defined __BORLANDC__ || defined __MWERKS__ || defined __SUNPRO_CC || defined __HP_aCC || defined __MRC__ || defined __SC__ || defined __IBMCPP__ || defined __PGI)) && (!defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL)
#     define CROSPI_BOOST_PP_VARIADICS_MSVC 1
# else
#     define CROSPI_BOOST_PP_VARIADICS_MSVC 0
# endif
#
# if CROSPI_BOOST_PP_CONFIG_FLAGS() & CROSPI_BOOST_PP_CONFIG_STRICT()
# define CROSPI_BOOST_PP_IS_STANDARD() 1
# else
# define CROSPI_BOOST_PP_IS_STANDARD() 0
# endif
#
# endif

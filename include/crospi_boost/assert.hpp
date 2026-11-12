//
//  crospi_boost/assert.hpp - CROSPI_BOOST_ASSERT(expr)
//                     CROSPI_BOOST_ASSERT_MSG(expr, msg)
//                     CROSPI_BOOST_VERIFY(expr)
//                     CROSPI_BOOST_VERIFY_MSG(expr, msg)
//                     CROSPI_BOOST_ASSERT_IS_VOID
//
//  Copyright (c) 2001, 2002 Peter Dimov and Multi Media Ltd.
//  Copyright (c) 2007, 2014 Peter Dimov
//  Copyright (c) Beman Dawes 2011
//  Copyright (c) 2015 Ion Gaztanaga
//
//  Distributed under the Boost Software License, Version 1.0.
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
//
//  Note: There are no include guards. This is intentional.
//
//  See http://www.boost.org/libs/assert/assert.html for documentation.
//

//
// Stop inspect complaining about use of 'assert':
//
// boostinspect:naassert_macro
//

//
// CROSPI_BOOST_ASSERT, CROSPI_BOOST_ASSERT_MSG, CROSPI_BOOST_ASSERT_IS_VOID
//

#undef CROSPI_BOOST_ASSERT
#undef CROSPI_BOOST_ASSERT_MSG
#undef CROSPI_BOOST_ASSERT_IS_VOID

#if defined(CROSPI_BOOST_DISABLE_ASSERTS) || ( defined(CROSPI_BOOST_ENABLE_ASSERT_DEBUG_HANDLER) && defined(NDEBUG) )

# define CROSPI_BOOST_ASSERT(expr) ((void)0)
# define CROSPI_BOOST_ASSERT_MSG(expr, msg) ((void)0)
# define CROSPI_BOOST_ASSERT_IS_VOID

#elif defined(CROSPI_BOOST_ENABLE_ASSERT_HANDLER) || ( defined(CROSPI_BOOST_ENABLE_ASSERT_DEBUG_HANDLER) && !defined(NDEBUG) )

#include <crospi_boost/config.hpp> // for CROSPI_BOOST_LIKELY
#include <crospi_boost/current_function.hpp>

namespace crospi_boost
{
#if defined(CROSPI_BOOST_ASSERT_HANDLER_IS_NORETURN)
    CROSPI_BOOST_NORETURN
#endif
    void assertion_failed(char const * expr, char const * function, char const * file, long line); // user defined
#if defined(CROSPI_BOOST_ASSERT_HANDLER_IS_NORETURN)
    CROSPI_BOOST_NORETURN
#endif
    void assertion_failed_msg(char const * expr, char const * msg, char const * function, char const * file, long line); // user defined
} // namespace crospi_boost

#define CROSPI_BOOST_ASSERT(expr) (CROSPI_BOOST_LIKELY(!!(expr))? ((void)0): ::crospi_boost::assertion_failed(#expr, CROSPI_BOOST_CURRENT_FUNCTION, __FILE__, __LINE__))
#define CROSPI_BOOST_ASSERT_MSG(expr, msg) (CROSPI_BOOST_LIKELY(!!(expr))? ((void)0): ::crospi_boost::assertion_failed_msg(#expr, msg, CROSPI_BOOST_CURRENT_FUNCTION, __FILE__, __LINE__))

#else

# include <assert.h> // .h to support old libraries w/o <cassert> - effect is the same

# define CROSPI_BOOST_ASSERT(expr) assert(expr)
# define CROSPI_BOOST_ASSERT_MSG(expr, msg) assert((expr)&&(msg))
#if defined(NDEBUG)
# define CROSPI_BOOST_ASSERT_IS_VOID
#endif

#endif

//
// CROSPI_BOOST_VERIFY, CROSPI_BOOST_VERIFY_MSG
//

#undef CROSPI_BOOST_VERIFY
#undef CROSPI_BOOST_VERIFY_MSG

#if defined(CROSPI_BOOST_DISABLE_ASSERTS) || ( !defined(CROSPI_BOOST_ENABLE_ASSERT_HANDLER) && defined(NDEBUG) )

# define CROSPI_BOOST_VERIFY(expr) ((void)(expr))
# define CROSPI_BOOST_VERIFY_MSG(expr, msg) ((void)(expr))

#else

# define CROSPI_BOOST_VERIFY(expr) CROSPI_BOOST_ASSERT(expr)
# define CROSPI_BOOST_VERIFY_MSG(expr, msg) CROSPI_BOOST_ASSERT_MSG(expr,msg)

#endif

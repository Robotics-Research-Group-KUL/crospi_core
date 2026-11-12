/*
Copyright 2014 Glen Joseph Fernandes
(glenjofe@gmail.com)

Distributed under the Boost Software License, Version 1.0.
(http://www.boost.org/LICENSE_1_0.txt)
*/
#ifndef CROSPI_BOOST_ALIGN_DETAIL_ALIGNED_ALLOC_POSIX_HPP
#define CROSPI_BOOST_ALIGN_DETAIL_ALIGNED_ALLOC_POSIX_HPP

#include <crospi_boost/align/detail/is_alignment.hpp>
#include <crospi_boost/assert.hpp>
#include <stdlib.h>

namespace crospi_boost {
namespace alignment {

inline void*
aligned_alloc(std::size_t alignment, std::size_t size) CROSPI_BOOST_NOEXCEPT
{
    CROSPI_BOOST_ASSERT(detail::is_alignment(alignment));
    if (alignment < sizeof(void*)) {
        alignment = sizeof(void*);
    }
    void* p;
    if (::posix_memalign(&p, alignment, size) != 0) {
        p = 0;
    }
    return p;
}

inline void
aligned_free(void* ptr) CROSPI_BOOST_NOEXCEPT
{
    ::free(ptr);
}

} /* alignment */
} /* boost */

#endif

//  Copyright (C) 2011-2013, 2016 Tim Blechmann
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_LOCKFREE_DETAIL_ATOMIC_HPP
#define CROSPI_BOOST_LOCKFREE_DETAIL_ATOMIC_HPP

#if defined( CROSPI_BOOST_LOCKFREE_FORCE_CROSPI_BOOST_ATOMIC )
#    include <crospi_boost/atomic.hpp>
#else
#    include <atomic>
#endif

namespace crospi_boost { namespace lockfree {
namespace detail {

#if defined( CROSPI_BOOST_LOCKFREE_FORCE_CROSPI_BOOST_ATOMIC )
using crospi_boost::atomic;
using crospi_boost::memory_order_acquire;
using crospi_boost::memory_order_consume;
using crospi_boost::memory_order_relaxed;
using crospi_boost::memory_order_release;
#else
using std::atomic;
using std::memory_order_acquire;
using std::memory_order_consume;
using std::memory_order_relaxed;
using std::memory_order_release;
#endif

} // namespace detail
using detail::atomic;
using detail::memory_order_acquire;
using detail::memory_order_consume;
using detail::memory_order_relaxed;
using detail::memory_order_release;

}}     // namespace crospi_boost::lockfree

#endif /* CROSPI_BOOST_LOCKFREE_DETAIL_ATOMIC_HPP */

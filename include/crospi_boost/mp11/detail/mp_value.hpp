#ifndef CROSPI_BOOST_MP11_DETAIL_MP_VALUE_HPP_INCLUDED
#define CROSPI_BOOST_MP11_DETAIL_MP_VALUE_HPP_INCLUDED

// Copyright 2023 Peter Dimov
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt

#include <crospi_boost/mp11/detail/config.hpp>
#include <type_traits>

#if defined(CROSPI_BOOST_MP11_HAS_TEMPLATE_AUTO)

namespace crospi_boost
{
namespace mp11
{

template<auto A> using mp_value = std::integral_constant<decltype(A), A>;

} // namespace mp11
} // namespace crospi_boost

#endif // #if defined(CROSPI_BOOST_MP11_HAS_TEMPLATE_AUTO)

#endif // #ifndef CROSPI_BOOST_MP11_DETAIL_MP_VALUE_HPP_INCLUDED

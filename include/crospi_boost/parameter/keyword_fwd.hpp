// Copyright Cromwell D. Enage 2017.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_KEYWORD_FWD_HPP
#define CROSPI_BOOST_PARAMETER_KEYWORD_FWD_HPP

namespace crospi_boost { namespace parameter {

    struct in_reference;
    struct out_reference;
    typedef ::crospi_boost::parameter::out_reference in_out_reference;
    struct forward_reference;
    struct consume_reference;
    typedef ::crospi_boost::parameter::consume_reference move_from_reference;

    template <typename Tag>
    struct keyword;
}} // namespace crospi_boost::parameter

#endif  // include guard


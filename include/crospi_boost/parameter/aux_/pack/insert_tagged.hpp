// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_INSERT_TAGGED_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_INSERT_TAGGED_HPP

#include <crospi_boost/parameter/aux_/set.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // Inserts Tagged::key_type into the UserArgs set.
    // Extra indirection to lazily evaluate Tagged::key_type.
    template <typename UsedArgs, typename Tagged>
    struct insert_tagged
      : ::crospi_boost::parameter::aux::insert_<UsedArgs,typename Tagged::key_type>
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard


// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_AS_PARAMETER_REQUIREMENTS_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_AS_PARAMETER_REQUIREMENTS_HPP

#include <crospi_boost/parameter/aux_/pack/parameter_requirements.hpp>
#include <crospi_boost/parameter/aux_/pack/tag_type.hpp>
#include <crospi_boost/parameter/aux_/pack/predicate.hpp>
#include <crospi_boost/parameter/deduced.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    // Converts a ParameterSpec into a specialization of
    // parameter_requirements.  We need to do this in order to get the
    // tag_type into the type in a way that can be conveniently matched
    // by a satisfies(...) member function in arg_list.
    template <typename ParameterSpec>
    struct as_parameter_requirements
    {
        typedef ::crospi_boost::parameter::aux::parameter_requirements<
            typename ::crospi_boost::parameter::aux::tag_type<ParameterSpec>::type
          , typename ::crospi_boost::parameter::aux::predicate<ParameterSpec>::type
          , ::crospi_boost::parameter::aux::has_default<ParameterSpec>
        > type;
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard


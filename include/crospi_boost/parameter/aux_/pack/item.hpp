// Copyright David Abrahams, Daniel Wallin 2003.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_AUX_PACK_ITEM_HPP
#define CROSPI_BOOST_PARAMETER_AUX_PACK_ITEM_HPP

#include <crospi_boost/parameter/aux_/void.hpp>
#include <crospi_boost/config.hpp>
#include <crospi_boost/config/workaround.hpp>

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
#include <crospi_boost/type_traits/is_const.hpp>
#include <crospi_boost/type_traits/remove_reference.hpp>
#endif

namespace crospi_boost { namespace parameter { namespace aux {

    // A parameter spec item typelist.
    template <
        typename Spec
      , typename Arg
      , typename Tail = ::crospi_boost::parameter::void_
    >
    struct item
    {
        typedef Spec spec;
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
        typedef ::crospi_boost::is_const<
            typename ::crospi_boost::remove_reference<Arg>::type
        > is_arg_const;
#endif
        typedef Arg arg;
        typedef Tail tail;
    };

    template <typename Spec, typename Arg, typename Tail>
    struct make_item
    {
        typedef crospi_boost::parameter::aux
        ::item<Spec,Arg,typename Tail::type> type;
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // include guard


// Copyright Daniel Wallin 2006.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_PARAMETER_SET_060912_HPP
#define CROSPI_BOOST_PARAMETER_SET_060912_HPP

#include <crospi_boost/parameter/config.hpp>

#if defined(CROSPI_BOOST_PARAMETER_CAN_USE_MP11)
#include <crospi_boost/mp11/list.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    typedef ::crospi_boost::mp11::mp_list<> set0;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mp11/algorithm.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename S, typename K>
    struct insert_
    {
        using type = ::crospi_boost::mp11::mp_insert_c<S,0,K>;
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mp11/integral.hpp>
#include <crospi_boost/mp11/utility.hpp>
#include <type_traits>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Set, typename K>
    struct has_key_
    {
        using type = ::crospi_boost::mp11::mp_if<
            ::crospi_boost::mp11::mp_empty<Set>
          , ::crospi_boost::mp11::mp_false
          , ::std::is_same<
                ::crospi_boost::mp11::mp_find<Set,K>
              , ::crospi_boost::mp11::mp_size<Set>
            >
        >;
    };
}}} // namespace crospi_boost::parameter::aux

#elif CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
#include <crospi_boost/mpl/list.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    typedef ::crospi_boost::mpl::list0<> set0;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mpl/push_front.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Set, typename K>
    struct insert_ : ::crospi_boost::mpl::push_front<Set,K>
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mpl/bool.hpp>
#include <crospi_boost/mpl/if.hpp>
#include <crospi_boost/mpl/end.hpp>
#include <crospi_boost/mpl/find.hpp>
#include <crospi_boost/type_traits/is_same.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Set, typename K>
    struct has_key_
    {
        typedef typename ::crospi_boost::mpl::find<Set,K>::type iter;
        typedef typename ::crospi_boost::mpl::if_<
            ::crospi_boost::is_same<iter,typename ::crospi_boost::mpl::end<Set>::type>
          , ::crospi_boost::mpl::false_
          , ::crospi_boost::mpl::true_
        >::type type;
    };
}}} // namespace crospi_boost::parameter::aux

#else   // !CROSPI_BOOST_PARAMETER_CAN_USE_MP11 && Borland workarounds not needed
#include <crospi_boost/mpl/set/set0.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    typedef ::crospi_boost::mpl::set0<> set0;
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mpl/insert.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Set, typename K>
    struct insert_ : ::crospi_boost::mpl::insert<Set,K>
    {
    };
}}} // namespace crospi_boost::parameter::aux

#include <crospi_boost/mpl/has_key.hpp>

namespace crospi_boost { namespace parameter { namespace aux {

    template <typename Set, typename K>
    struct has_key_ : ::crospi_boost::mpl::has_key<Set,K>
    {
    };
}}} // namespace crospi_boost::parameter::aux

#endif  // CROSPI_BOOST_PARAMETER_CAN_USE_MP11 || Borland workarounds needed
#endif  // include guard


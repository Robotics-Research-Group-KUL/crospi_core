/*
Copyright (C) 2002 Brad King (brad.king@kitware.com)
                   Douglas Gregor (gregod@cs.rpi.edu)

Copyright (C) 2002, 2008, 2013 Peter Dimov

Copyright (C) 2017 Glen Joseph Fernandes (glenjofe@gmail.com)

Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_CORE_ADDRESSOF_HPP
#define CROSPI_BOOST_CORE_ADDRESSOF_HPP

#include <crospi_boost/config.hpp>

#if defined(CROSPI_BOOST_MSVC_FULL_VER) && CROSPI_BOOST_MSVC_FULL_VER >= 190024215
#define CROSPI_BOOST_CORE_HAS_BUILTIN_ADDRESSOF
#elif defined(CROSPI_BOOST_GCC) && CROSPI_BOOST_GCC >= 70000
#define CROSPI_BOOST_CORE_HAS_BUILTIN_ADDRESSOF
#elif defined(__has_builtin)
#if __has_builtin(__builtin_addressof)
#define CROSPI_BOOST_CORE_HAS_BUILTIN_ADDRESSOF
#endif
#endif

#if defined(CROSPI_BOOST_CORE_HAS_BUILTIN_ADDRESSOF)
#if defined(CROSPI_BOOST_NO_CXX11_CONSTEXPR)
#define CROSPI_BOOST_CORE_NO_CONSTEXPR_ADDRESSOF
#endif

namespace crospi_boost {

template<class T>
CROSPI_BOOST_CONSTEXPR inline T*
addressof(T& o) CROSPI_BOOST_NOEXCEPT
{
    return __builtin_addressof(o);
}

} /* boost */
#else
#include <crospi_boost/config/workaround.hpp>
#include <cstddef>

namespace crospi_boost {
namespace detail {

template<class T>
class addrof_ref {
public:
    CROSPI_BOOST_FORCEINLINE addrof_ref(T& o) CROSPI_BOOST_NOEXCEPT
        : o_(o) { }
    CROSPI_BOOST_FORCEINLINE operator T&() const CROSPI_BOOST_NOEXCEPT {
        return o_;
    }
private:
    addrof_ref& operator=(const addrof_ref&);
    T& o_;
};

template<class T>
struct addrof {
    static CROSPI_BOOST_FORCEINLINE T* get(T& o, long) CROSPI_BOOST_NOEXCEPT {
        return reinterpret_cast<T*>(&
            const_cast<char&>(reinterpret_cast<const volatile char&>(o)));
    }
    static CROSPI_BOOST_FORCEINLINE T* get(T* p, int) CROSPI_BOOST_NOEXCEPT {
        return p;
    }
};

#if !defined(CROSPI_BOOST_NO_CXX11_NULLPTR)
#if !defined(CROSPI_BOOST_NO_CXX11_DECLTYPE) && \
    (defined(__INTEL_COMPILER) || \
        (defined(__clang__) && !defined(_LIBCPP_VERSION)))
typedef decltype(nullptr) addrof_null_t;
#else
typedef std::nullptr_t addrof_null_t;
#endif

template<>
struct addrof<addrof_null_t> {
    typedef addrof_null_t type;
    static CROSPI_BOOST_FORCEINLINE type* get(type& o, int) CROSPI_BOOST_NOEXCEPT {
        return &o;
    }
};

template<>
struct addrof<const addrof_null_t> {
    typedef const addrof_null_t type;
    static CROSPI_BOOST_FORCEINLINE type* get(type& o, int) CROSPI_BOOST_NOEXCEPT {
        return &o;
    }
};

template<>
struct addrof<volatile addrof_null_t> {
    typedef volatile addrof_null_t type;
    static CROSPI_BOOST_FORCEINLINE type* get(type& o, int) CROSPI_BOOST_NOEXCEPT {
        return &o;
    }
};

template<>
struct addrof<const volatile addrof_null_t> {
    typedef const volatile addrof_null_t type;
    static CROSPI_BOOST_FORCEINLINE type* get(type& o, int) CROSPI_BOOST_NOEXCEPT {
        return &o;
    }
};
#endif

} /* detail */

#if defined(CROSPI_BOOST_NO_CXX11_SFINAE_EXPR) || \
    defined(CROSPI_BOOST_NO_CXX11_CONSTEXPR) || \
    defined(CROSPI_BOOST_NO_CXX11_DECLTYPE)
#define CROSPI_BOOST_CORE_NO_CONSTEXPR_ADDRESSOF

template<class T>
CROSPI_BOOST_FORCEINLINE T*
addressof(T& o) CROSPI_BOOST_NOEXCEPT
{
#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610)) || \
    CROSPI_BOOST_WORKAROUND(__SUNPRO_CC, <= 0x5120)
    return crospi_boost::detail::addrof<T>::get(o, 0);
#else
    return crospi_boost::detail::addrof<T>::get(crospi_boost::detail::addrof_ref<T>(o), 0);
#endif
}

#if CROSPI_BOOST_WORKAROUND(__SUNPRO_CC, CROSPI_BOOST_TESTED_AT(0x590))
namespace detail {

template<class T>
struct addrof_result {
    typedef T* type;
};

} /* detail */

template<class T, std::size_t N>
CROSPI_BOOST_FORCEINLINE typename crospi_boost::detail::addrof_result<T[N]>::type
addressof(T (&o)[N]) CROSPI_BOOST_NOEXCEPT
{
    return &o;
}
#endif

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x564))
template<class T, std::size_t N>
CROSPI_BOOST_FORCEINLINE
T (*addressof(T (&o)[N]) CROSPI_BOOST_NOEXCEPT)[N]
{
   return reinterpret_cast<T(*)[N]>(&o);
}

template<class T, std::size_t N>
CROSPI_BOOST_FORCEINLINE
const T (*addressof(const T (&o)[N]) CROSPI_BOOST_NOEXCEPT)[N]
{
   return reinterpret_cast<const T(*)[N]>(&o);
}
#endif
#else
namespace detail {

template<class T>
T addrof_declval() CROSPI_BOOST_NOEXCEPT;

template<class>
struct addrof_void {
    typedef void type;
};

template<class T, class E = void>
struct addrof_member_operator {
    static constexpr bool value = false;
};

template<class T>
struct addrof_member_operator<T, typename
    addrof_void<decltype(addrof_declval<T&>().operator&())>::type> {
    static constexpr bool value = true;
};

#if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_INTEL, < 1600)
struct addrof_addressable { };

addrof_addressable*
operator&(addrof_addressable&) CROSPI_BOOST_NOEXCEPT;
#endif

template<class T, class E = void>
struct addrof_non_member_operator {
    static constexpr bool value = false;
};

template<class T>
struct addrof_non_member_operator<T, typename
    addrof_void<decltype(operator&(addrof_declval<T&>()))>::type> {
    static constexpr bool value = true;
};

template<class T, class E = void>
struct addrof_expression {
    static constexpr bool value = false;
};

template<class T>
struct addrof_expression<T,
    typename addrof_void<decltype(&addrof_declval<T&>())>::type> {
    static constexpr bool value = true;
};

template<class T>
struct addrof_is_constexpr {
    static constexpr bool value = addrof_expression<T>::value &&
        !addrof_member_operator<T>::value &&
        !addrof_non_member_operator<T>::value;
};

template<bool E, class T>
struct addrof_if { };

template<class T>
struct addrof_if<true, T> {
    typedef T* type;
};

template<class T>
CROSPI_BOOST_FORCEINLINE
typename addrof_if<!addrof_is_constexpr<T>::value, T>::type
addressof(T& o) CROSPI_BOOST_NOEXCEPT
{
    return addrof<T>::get(addrof_ref<T>(o), 0);
}

template<class T>
constexpr CROSPI_BOOST_FORCEINLINE
typename addrof_if<addrof_is_constexpr<T>::value, T>::type
addressof(T& o) CROSPI_BOOST_NOEXCEPT
{
    return &o;
}

} /* detail */

template<class T>
constexpr CROSPI_BOOST_FORCEINLINE T*
addressof(T& o) CROSPI_BOOST_NOEXCEPT
{
    return crospi_boost::detail::addressof(o);
}
#endif

} /* boost */
#endif

#if !defined(CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES) && \
    !defined(CROSPI_BOOST_NO_CXX11_DELETED_FUNCTIONS)
namespace crospi_boost {

template<class T>
const T* addressof(const T&&) = delete;

} /* boost */
#endif

#endif

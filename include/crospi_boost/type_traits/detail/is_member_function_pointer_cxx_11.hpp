
//  (C) Copyright Dave Abrahams, Steve Cleary, Beman Dawes, Howard
//  Hinnant & John Maddock 2000.  
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.


#ifndef CROSPI_BOOST_TT_IS_MEMBER_FUNCTION_POINTER_CXX_11_HPP_INCLUDED
#define CROSPI_BOOST_TT_IS_MEMBER_FUNCTION_POINTER_CXX_11_HPP_INCLUDED

#include <crospi_boost/type_traits/integral_constant.hpp>

namespace crospi_boost {

#ifdef _MSC_VER
#define CROSPI_BOOST_TT_DEF_CALL __thiscall
#else
#define CROSPI_BOOST_TT_DEF_CALL
#endif


   template <class T>
   struct is_member_function_pointer : public false_type {};
   template <class T>
   struct is_member_function_pointer<T const> : public is_member_function_pointer<T> {};
   template <class T>
   struct is_member_function_pointer<T volatile> : public is_member_function_pointer<T> {};
   template <class T>
   struct is_member_function_pointer<T const volatile> : public is_member_function_pointer<T> {};

#if defined(CROSPI_BOOST_TT_NO_DEDUCED_NOEXCEPT_PARAM)
   // MSVC can't handle noexcept(b) as a deduced template parameter 
   // so we will have to write everything out :(
#define CROSPI_BOOST_TT_NOEXCEPT_PARAM
#define CROSPI_BOOST_TT_NOEXCEPT_DECL
#elif defined(__cpp_noexcept_function_type)
#define CROSPI_BOOST_TT_NOEXCEPT_PARAM , bool NE
#define CROSPI_BOOST_TT_NOEXCEPT_DECL noexcept(NE)
#else
#define CROSPI_BOOST_TT_NOEXCEPT_PARAM
#define CROSPI_BOOST_TT_NOEXCEPT_DECL
#endif

   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (C::*)(Args..., ...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

   // Reference qualified:

   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

   // rvalue reference qualified:

   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

#if defined(_MSC_VER) && !defined(_M_ARM) && !defined(_M_ARM64)
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__clrcall C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // reference qualified:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
 
   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // rvalue reference qualified:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif
 
   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__stdcall C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__fastcall C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret (__vectorcall C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

#endif


#if defined(CROSPI_BOOST_TT_NO_DEDUCED_NOEXCEPT_PARAM)  && !defined(CROSPI_BOOST_TT_NO_NOEXCEPT_SEPARATE_TYPE)

#undef CROSPI_BOOST_TT_NOEXCEPT_DECL
#define CROSPI_BOOST_TT_NOEXCEPT_DECL noexcept

   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

   // Reference qualified:

   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)& CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const volatile & CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

   // rvalue reference qualified:

   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const qualified:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // volatile:
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   // const volatile
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(CROSPI_BOOST_TT_DEF_CALL C::*)(Args...)const volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class ...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(C::*)(Args..., ...)const volatile && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};

#if defined(_MSC_VER) && !defined(_M_ARM) && !defined(_M_ARM64)
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)const CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)const volatile CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // reference qualified:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)const &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)const volatile &CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // rvalue reference qualified:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...) && CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)const &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

   // const volatile:
#if !defined(_M_X64) && !defined(_M_CEE_SAFE) && !defined(_M_CEE_PURE)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__stdcall C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__cdecl C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#ifdef _MANAGED
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__clrcall C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#else
#ifndef _M_AMD64
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__fastcall C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#if defined(_M_IX86_FP) && (_M_IX86_FP >= 2) || defined(_M_X64)
   template <class Ret, class C, class...Args CROSPI_BOOST_TT_NOEXCEPT_PARAM>
   struct is_member_function_pointer<Ret(__vectorcall C::*)(Args...)const volatile &&CROSPI_BOOST_TT_NOEXCEPT_DECL> : public true_type {};
#endif
#endif

#endif // defined(_MSC_VER) && !defined(_M_ARM) && !defined(_M_ARM64)


#endif

#undef CROSPI_BOOST_TT_NOEXCEPT_DECL
#undef CROSPI_BOOST_TT_NOEXCEPT_PARAM
#undef CROSPI_BOOST_TT_DEF_CALL
}

#endif // CROSPI_BOOST_TT_IS_MEMBER_FUNCTION_POINTER_CXX_11_HPP_INCLUDED

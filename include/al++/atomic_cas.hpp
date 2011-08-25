/***************************************************************************
 *   Copyright (C) 2011 by Naomasa Matsubayashi   *
 *   fadis@quaternion.sakura.ne.jp   *
 *                                                                         *
 *   All rights reserved.                                                  *
 *                                                                         *
 * Redistribution and use in source and binary forms, with or without      *
 * modification, are permitted provided that the following conditions are  *
 * met:                                                                    *
 *                                                                         *
 *  1. Redistributions of source code must retain the above copyright      *
 *     notice, this list of conditions and the following disclaimer.       *
 *  2. Redistributions in binary form must reproduce the above copyright   *
 *     notice, this list of conditions and the following disclaimer in the *
 *     documentation and/or other materials provided with the distribution.*
 *                                                                         *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT      *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ***************************************************************************/

// This code is inspired from atomic_cas32 in Boost.Interprocess
// Since Boost.Interprocess was marged into the stable version of Boost,
// this code should be replaced in near future.

#ifndef ALXX_ATOMIC_CAS_HPP
#define ALXX_ATOMIC_CAS_HPP

#include <boost/cstdint.hpp>

#if defined(__GNUC__)
  static inline int32_t atomicCAS( volatile int32_t *ptr, int32_t expected, int32_t desired ) {
    return __sync_val_compare_and_swap_4(ptr, expected, desired);
  }
#elif defined(__ICL) || defined(_MSC_VER)
#if defined(_MSC_VER)
#include <Windows.h>
#include <intrin.h>
#endif
  static inline int32_t atomicCAS( volatile int32_t *ptr, int32_t expected, int32_t desired ) {
    return _InterlockedCompareExchange(reinterpret_cast<volatile long*>(ptr), desired, expected);
  }
#elif (defined(__ICC) || defined(__ECC))
  static inline int32_t atomicCAS( volatile int32_t *ptr, int32_t expected, int32_t desired ) {
    return _InterlockedCompareExchange((void*)ptr, desired, expected);
  }
#elif (defined(__SUNPRO_CC) && defined(__sparc))
  static inline int32_t atomicCAS( volatile int32_t *ptr, int32_t expected, int32_t desired ) {
    return atomic_cas_32((volatile unsigned int*)ptr, expected, desired);
  }
#endif

#endif

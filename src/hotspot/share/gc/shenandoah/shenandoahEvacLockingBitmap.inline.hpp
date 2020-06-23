/*
 * Copyright (c) 2020, Red Hat, Inc. All rights reserved.
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This code is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 only, as
 * published by the Free Software Foundation.
 *
 * This code is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * version 2 for more details (a copy is included in the LICENSE file that
 * accompanied this code).
 *
 * You should have received a copy of the GNU General Public License version
 * 2 along with this work; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Please contact Oracle, 500 Oracle Parkway, Redwood Shores, CA 94065 USA
 * or visit www.oracle.com if you need additional information or have any
 * questions.
 *
 */

#ifndef SHARE_GC_SHENANDOAH_SHENANDOAHEVACLOCKINGBITMAP_INLINE_HPP
#define SHARE_GC_SHENANDOAH_SHENANDOAHEVACLOCKINGBITMAP_INLINE_HPP

#include "gc/shenandoah/shenandoahEvacLockingBitmap.hpp"
#include "memory/memRegion.hpp"
#include "oops/oopsHierarchy.hpp"
#include "utilities/bitMap.hpp"

inline size_t ShenandoahEvacLockingBitmap::addr_to_offset(const HeapWord* addr) const {
  return pointer_delta(addr, _covered.start()) >> _shifter;
}

inline void ShenandoahEvacLockingBitmap::acquire(oop obj) {
  size_t offset = addr_to_offset(cast_from_oop<HeapWord*>(obj));
  int ctr = 0;
  int yields = 0;
  while (!_bm.par_set_bit(offset)) {
    // Spin/yield/sleep-strategy inspired by Thread::SpinAcquire()
    ++ctr;
    if ((ctr & 0xFFF) == 0 || !os::is_MP()) {
      if (yields > 5) {
        os::naked_short_sleep(1);
      } else {
        os::naked_yield();
        ++yields;
      }
    } else {
      SpinPause();
    }

  }
}

inline void ShenandoahEvacLockingBitmap::release(oop obj) {
  size_t offset = addr_to_offset(cast_from_oop<HeapWord*>(obj));
  assert(_bm.at(offset), "");
  bool success = _bm.par_clear_bit(offset);
  assert(success, "");
}

inline ShenandoahEvacLocker::ShenandoahEvacLocker(ShenandoahEvacLockingBitmap* bitmap, oop obj):
_bitmap(bitmap), _obj(obj) {
  _bitmap->acquire(_obj);
}

inline ShenandoahEvacLocker::~ShenandoahEvacLocker() {
  _bitmap->release(_obj);
}

#endif // SHARE_GC_SHENANDOAH_SHENANDOAHEVACLOCKINGBITMAP_INLINE_HPP

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

#ifndef SHARE_GC_SHENANDOAH_SHENANDOAHEVACLOCKINGBITMAP_HPP
#define SHARE_GC_SHENANDOAH_SHENANDOAHEVACLOCKINGBITMAP_HPP

#include "memory/allocation.hpp"
#include "memory/memRegion.hpp"
#include "oops/oopsHierarchy.hpp"
#include "runtime/globals.hpp"
#include "utilities/bitMap.hpp"

/*
 * Protects the evacuation slow-path from concurrent access/races:
 * - Only one thread must be allowed to (successfully) evacuate an object
 * - In case of evacuation failure by one thread, no other thread must be allowed to evacuate
 *   an object
 *
 * This is achieved by a locking bitmap. Each bit corresponds to a card of
 * 2^ShenandoahEvacGranularity heap words. Before evacuating, a thread must acquire
 * the corresponding bit, and release it after evacuation is done. It is preferable
 * to use the scoped ShenandoahEvacLocker for that.
 */
class ShenandoahEvacLockingBitmap : public CHeapObj<mtGC> {
private:
  MemRegion _covered;    // The heap area covered by this bitmap.
  const int _shifter;    // Shift amount from heap index to bit index in the bitmap.
  CHeapBitMap _bm;        // The actual bitmap.

  // Convert from address to bit offset.
  inline size_t addr_to_offset(const HeapWord* addr) const;

public:
  ShenandoahEvacLockingBitmap(MemRegion heap) :
  _covered(heap),
  _shifter((1 + ShenandoahEvacLockGranularity) * LogHeapWordSize),
  _bm(_covered.word_size() >> _shifter, mtGC) {
}

  inline void acquire(oop obj);
  inline void release(oop obj);
};

class ShenandoahEvacLocker : public StackObj {
private:
  ShenandoahEvacLockingBitmap* _bitmap;
  oop const _obj;
public:
  inline ShenandoahEvacLocker(ShenandoahEvacLockingBitmap* bitmap, oop obj);
  inline ~ShenandoahEvacLocker();
};

#endif // SHARE_GC_SHENANDOAH_SHENANDOAHEVACLOCKINGBITMAP_HPP

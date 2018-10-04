/*
 * Copyright (c) 2018, Red Hat, Inc. and/or its affiliates.
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

#include "precompiled.hpp"

#include "gc/shenandoah/shenandoahHeap.inline.hpp"
#include "gc/shenandoah/shenandoahSATBMarkQueueSet.hpp"
#include "gc/shenandoah/shenandoahThreadLocalData.hpp"

ShenandoahSATBMarkQueueSet::ShenandoahSATBMarkQueueSet() : _heap(NULL) {}

void ShenandoahSATBMarkQueueSet::initialize(ShenandoahHeap* const heap,
                                            Monitor* cbl_mon, Mutex* fl_lock,
                                            int process_completed_threshold,
                                            uint buffer_enqueue_threshold_percentage,
                                            Mutex* lock) {
  SATBMarkQueueSet::initialize(cbl_mon, fl_lock,
                               process_completed_threshold,
                               buffer_enqueue_threshold_percentage,
                               lock);
  _heap = heap;
}

SATBMarkQueue& ShenandoahSATBMarkQueueSet::satb_queue_for_thread(JavaThread* const t) const {
  return ShenandoahThreadLocalData::satb_mark_queue(t);
}

static inline bool discard_entry(const void* entry, ShenandoahHeap* heap) {
  return !heap->requires_marking(entry);
}

class ShenandoahSATBMarkQueueFilterFn {
  ShenandoahHeap* _heap;

public:
  ShenandoahSATBMarkQueueFilterFn(ShenandoahHeap* heap) : _heap(heap) {}

  // Return true if entry should be filtered out (removed), false if
  // it should be retained.
  bool operator()(const void* entry) const {
    return discard_entry(entry, _heap);
  }
};

void ShenandoahSATBMarkQueueSet::filter(SATBMarkQueue* queue) {
  assert(_heap != NULL, "SATB queue set not initialized");
  apply_filter(ShenandoahSATBMarkQueueFilterFn(_heap), queue);
}

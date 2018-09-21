/*
 * Copyright (c) 2013, 2015, Red Hat, Inc. and/or its affiliates.
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
#include "gc/shenandoah/shenandoahUtils.hpp"
#include "gc/shenandoah/vm_operations_shenandoah.hpp"

bool VM_ShenandoahReferenceOperation::doit_prologue() {
  Heap_lock->lock();
  return true;
}

void VM_ShenandoahReferenceOperation::doit_epilogue() {
  if (Universe::has_reference_pending_list()) {
    Heap_lock->notify_all();
  }
  Heap_lock->unlock();
}

void VM_ShenandoahInitMark::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::CONCURRENT);
  ShenandoahHeap::heap()->entry_init_mark();
}

void VM_ShenandoahFinalMarkStartEvac::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::CONCURRENT);
  ShenandoahHeap::heap()->entry_final_mark();
}

void VM_ShenandoahFinalEvac::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::CONCURRENT);
  ShenandoahHeap::heap()->entry_final_evac();
}

void VM_ShenandoahFullGC::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::FULL);
  ShenandoahHeap::heap()->entry_full(_gc_cause);
}

void VM_ShenandoahDegeneratedGC::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::CONCURRENT);
  ShenandoahHeap::heap()->entry_degenerated(_point);
}

void VM_ShenandoahInitTraversalGC::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::CONCURRENT);
  ShenandoahHeap::heap()->entry_init_traversal();
}

void VM_ShenandoahFinalTraversalGC::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::CONCURRENT);
  ShenandoahHeap::heap()->entry_final_traversal();
}

void VM_ShenandoahInitUpdateRefs::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::CONCURRENT);
  ShenandoahHeap::heap()->entry_init_updaterefs();
}

void VM_ShenandoahFinalUpdateRefs::doit() {
  ShenandoahGCPauseMark mark(_gc_id, SvcGCMarker::CONCURRENT);
  ShenandoahHeap::heap()->entry_final_updaterefs();
}

/*
 * Copyright (c) 2019, Red Hat, Inc. All rights reserved.
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

#include "gc/shenandoah/shenandoahConcurrentRoots.hpp"
#include "gc/shenandoah/shenandoahHeap.inline.hpp"
#include "gc/shenandoah/shenandoahNMethod.hpp"
#include "memory/resourceArea.hpp"

ShenandoahNMethod::ShenandoahNMethod(nmethod* nm, GrowableArray<oop*>& oops, bool non_immediate_oops) :
  _nm(nm), _oops(NULL), _oops_count(0) {

  if (!oops.is_empty()) {
    _oops_count = oops.length();
    _oops = NEW_C_HEAP_ARRAY(oop*, _oops_count, mtGC);
    for (int c = 0; c < _oops_count; c++) {
      _oops[c] = oops.at(c);
    }
  }
  _has_non_immed_oops = non_immediate_oops;

  assert_same_oops();
}

ShenandoahNMethod::~ShenandoahNMethod() {
  assert_same_oops(true /*allow_dead*/);

  if (_oops != NULL) {
    FREE_C_HEAP_ARRAY(oop*, _oops);
  }
}

class ShenandoahHasCSetOopClosure : public OopClosure {
private:
  ShenandoahHeap* const _heap;
  bool                  _has_cset_oops;

public:
  ShenandoahHasCSetOopClosure() :
    _heap(ShenandoahHeap::heap()),
    _has_cset_oops(false) {
  }

  bool has_cset_oops() const {
    return _has_cset_oops;
  }

  void do_oop(oop* p) {
    oop value = RawAccess<>::oop_load(p);
    if (!_has_cset_oops && _heap->in_collection_set(value)) {
      _has_cset_oops = true;
    }
  }

  void do_oop(narrowOop* p) {
    ShouldNotReachHere();
  }
};

bool ShenandoahNMethod::has_cset_oops(ShenandoahHeap *heap) {
  ShenandoahHasCSetOopClosure cl;
  oops_do(&cl);
  return cl.has_cset_oops();
}

void ShenandoahNMethod::update() {
  ResourceMark rm;
  bool non_immediate_oops = false;
  GrowableArray<oop*> oops;

  detect_reloc_oops(nm(), oops, non_immediate_oops);
  if (oops.length() != _oops_count) {
    if (_oops != NULL) {
      FREE_C_HEAP_ARRAY(oop*, _oops);
      _oops = NULL;
    }

    _oops_count = oops.length();
    if (_oops_count > 0) {
      _oops = NEW_C_HEAP_ARRAY(oop*, _oops_count, mtGC);
    }
  }

  for (int index = 0; index < _oops_count; index ++) {
    _oops[index] = oops.at(index);
  }
  _has_non_immed_oops = non_immediate_oops;

  assert_same_oops();
}

void ShenandoahNMethod::oops_do(OopClosure* oops, bool fix_relocations) {
  for (int c = 0; c < _oops_count; c ++) {
    oops->do_oop(_oops[c]);
  }

  oop* const begin = _nm->oops_begin();
  oop* const end = _nm->oops_end();
  for (oop* p = begin; p < end; p++) {
    if (*p != Universe::non_oop_word()) {
      oops->do_oop(p);
    }
  }

  if (fix_relocations && _has_non_immed_oops) {
    _nm->fix_oop_relocations();
  }
}

void ShenandoahNMethod::detect_reloc_oops(nmethod* nm, GrowableArray<oop*>& oops, bool& has_non_immed_oops) {
  has_non_immed_oops = false;
  // Find all oops relocations
  RelocIterator iter(nm);
  while (iter.next()) {
    if (iter.type() != relocInfo::oop_type) {
      // Not an oop
      continue;
    }

    oop_Relocation* r = iter.oop_reloc();
    if (!r->oop_is_immediate()) {
      // Non-immediate oop found
      has_non_immed_oops = true;
      continue;
    }

    if (r->oop_value() != NULL) {
      // Non-NULL immediate oop found. NULL oops can safely be
      // ignored since the method will be re-registered if they
      // are later patched to be non-NULL.
      oops.push(r->oop_addr());
    }
  }
}

ShenandoahNMethod* ShenandoahNMethod::for_nmethod(nmethod* nm) {
  ResourceMark rm;
  bool non_immediate_oops = false;
  GrowableArray<oop*> oops;

  detect_reloc_oops(nm, oops, non_immediate_oops);

  // No embedded oops
  if(oops.is_empty() && nm->oops_begin() >= nm->oops_end()) {
    return NULL;
  }

  return new ShenandoahNMethod(nm, oops, non_immediate_oops);
}

#ifdef ASSERT
void ShenandoahNMethod::assert_alive_and_correct() {
  assert(_nm->is_alive(), "only alive nmethods here");
  ShenandoahHeap* heap = ShenandoahHeap::heap();
  for (int c = 0; c < _oops_count; c++) {
    oop *loc = _oops[c];
    assert(_nm->code_contains((address) loc) || _nm->oops_contains(loc), "nmethod should contain the oop*");
    oop o = RawAccess<>::oop_load(loc);
    shenandoah_assert_correct_except(loc, o, o == NULL || heap->is_full_gc_move_in_progress());
  }

  oop* const begin = _nm->oops_begin();
  oop* const end = _nm->oops_end();
  for (oop* p = begin; p < end; p++) {
    if (*p != Universe::non_oop_word()) {
      oop o = RawAccess<>::oop_load(p);
      shenandoah_assert_correct_except(p, o, o == NULL || heap->is_full_gc_move_in_progress());
    }
  }
}

class ShenandoahNMethodOopDetector : public OopClosure {
private:
  ResourceMark rm; // For growable array allocation below.
  GrowableArray<oop*> _oops;

public:
  ShenandoahNMethodOopDetector() : _oops(10) {};

  void do_oop(oop* o) {
    _oops.append(o);
  }
  void do_oop(narrowOop* o) {
    fatal("NMethods should not have compressed oops embedded.");
  }

  GrowableArray<oop*>* oops() {
    return &_oops;
  }

  bool has_oops() {
    return !_oops.is_empty();
  }
};

void ShenandoahNMethod::assert_same_oops(bool allow_dead) {
  ShenandoahNMethodOopDetector detector;
  nm()->oops_do(&detector, allow_dead);

  GrowableArray<oop*>* oops = detector.oops();

  assert(oops->length() == oop_count(), "Must match");

  for (int index = 0; index < _oops_count; index ++) {
    assert(oops->contains(_oops[index]), "Must contain this oop");
  }

  for (oop* p = nm()->oops_begin(); p < nm()->oops_end(); p ++) {
    assert(oops->contains(p), "Must contain this oop");
  }
}

void ShenandoahNMethod::assert_no_oops(nmethod* nm, bool allow_dead) {
  ShenandoahNMethodOopDetector detector;
  nm->oops_do(&detector, allow_dead);
  assert(detector.oops()->length() == 0, "Should not have oops");
}
#endif

ShenandoahNMethodTable::ShenandoahNMethodTable() :
  _heap(ShenandoahHeap::heap()),
  _size(minSize),
  _index(0),
  _claimed(0) {
  _array = NEW_C_HEAP_ARRAY(ShenandoahNMethod*, _size, mtGC);
  DEBUG_ONLY(_iteration_in_progress = false;)
}

ShenandoahNMethodTable::~ShenandoahNMethodTable() {
  assert(_array != NULL, "Sanity");
  FREE_C_HEAP_ARRAY(ShenandoahNMethod*, _array);
}

void ShenandoahNMethodTable::register_nmethod(nmethod* nm) {
  assert(CodeCache_lock->owned_by_self(), "Must have CodeCache_lock held");
  assert(_index >= 0 && _index <= _size, "Sanity");

  ShenandoahLocker locker(&_lock);
  int idx = index_of(nm);
  if (idx != -1) {
    at(idx)->update();
    return;
  }

  ShenandoahNMethod* snm = ShenandoahNMethod::for_nmethod(nm);
  if (snm == NULL) {
    return;
  }

  log_register_nmethod(nm);
  append(snm);
}

void ShenandoahNMethodTable::unregister_nmethod(nmethod* nm) {
  assert_locked_or_safepoint(CodeCache_lock);
  ShenandoahLocker locker(&_lock);

  int idx = index_of(nm);
  if (idx != -1) {
    log_unregister_nmethod(nm);
    remove(idx);
  } else {
    ShenandoahNMethod::assert_no_oops(nm, true /*allow_dead*/);
  }
}

bool ShenandoahNMethodTable::contain(nmethod* nm) const {
  return index_of(nm) != -1;
}

ShenandoahNMethod* ShenandoahNMethodTable::at(int index) const {
  assert(index >= 0 && index < _index, "Out of bound");
  return _array[index];
}

int ShenandoahNMethodTable::index_of(nmethod* nm) const {
  for (int index = 0; index < length(); index ++) {
    if (_array[index]->nm() == nm) {
      return index;
    }
  }
  return -1;
}

void ShenandoahNMethodTable::remove(int idx) {
  shenandoah_assert_locked_or_safepoint(CodeCache_lock);
  assert(!_iteration_in_progress, "Can not happen");
  assert(_index >= 0 && _index < _size, "Sanity");

  assert(idx >= 0 && idx < _index, "Out of bound");
  assert(_index > 0 && _index < _size, "Corrupted table");
  ShenandoahNMethod* snm = _array[idx];

  _index --;
  _array[idx] = _array[_index];

  delete snm;
}

void ShenandoahNMethodTable::append(ShenandoahNMethod* snm) {
  if (is_full()) {
    int new_size = 2 * _size;
    ShenandoahNMethod** old_table = _array;

    // Rebuild table and replace current one
    rebuild(new_size);

    FREE_C_HEAP_ARRAY(nmethod*, old_table);
  }

  _array[_index ++] = snm;
  assert(_index >= 0 && _index <= _size, "Sanity");
}

void ShenandoahNMethodTable::rebuild(int size) {
  ShenandoahNMethod** arr = NEW_C_HEAP_ARRAY(ShenandoahNMethod*, size, mtGC);
  for (int index = 0; index < _index; index ++) {
      arr[index] = _array[index];
  }
  _array = arr;
  _size = size;
}

void ShenandoahNMethodTable::prepare_for_iteration() {
  assert(!_iteration_in_progress, "Already in progress");
  DEBUG_ONLY(_iteration_in_progress = true;)
  _claimed = 0;
}

void ShenandoahNMethodTable::finish_iteration() {
  assert(_iteration_in_progress, "Why we here?");
  DEBUG_ONLY(_iteration_in_progress = false;)
}

void ShenandoahNMethodTable::log_register_nmethod(nmethod* nm) {
  LogTarget(Debug, gc, nmethod) log;
  if (!log.is_enabled()) {
    return;
  }

  ResourceMark rm;
  log.print("Register NMethod: %s.%s [" PTR_FORMAT "] (%s)",
            nm->method()->method_holder()->external_name(),
            nm->method()->name()->as_C_string(),
            p2i(nm),
            nm->compiler_name());
}

void ShenandoahNMethodTable::log_unregister_nmethod(nmethod* nm) {
  LogTarget(Debug, gc, nmethod) log;
  if (!log.is_enabled()) {
    return;
  }

  ResourceMark rm;
  log.print("Unregister NMethod: %s.%s [" PTR_FORMAT "]",
            nm->method()->method_holder()->external_name(),
            nm->method()->name()->as_C_string(),
            p2i(nm));
}

#ifdef ASSERT
void ShenandoahNMethodTable::assert_nmethods_alive_and_correct() {
  assert_locked_or_safepoint(CodeCache_lock);

  for (int index = 0; index < length(); index ++) {
    ShenandoahNMethod* m = _array[index];
    m->assert_alive_and_correct();
  }
}
#endif

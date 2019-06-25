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

#ifndef SHARE_GC_SHENANDOAH_SHENANDOAHNMETHOD_HPP
#define SHARE_GC_SHENANDOAH_SHENANDOAHNMETHOD_HPP

#include "code/nmethod.hpp"
#include "gc/shenandoah/shenandoahHeap.hpp"
#include "gc/shenandoah/shenandoahLock.hpp"
#include "memory/allocation.hpp"
#include "utilities/growableArray.hpp"

// ShenandoahNMethod tuple records the internal locations of oop slots within reclocation stream in
// the nmethod. This allows us to quickly scan the oops without doing the nmethod-internal scans,
// that sometimes involves parsing the machine code. Note it does not record the oops themselves,
// because it would then require handling these tuples as the new class of roots.
class ShenandoahNMethod : public CHeapObj<mtGC> {
private:
  nmethod* _nm;
  oop**    _oops;
  int      _oops_count;
  bool     _has_non_immed_oops;

public:
  ShenandoahNMethod(nmethod *nm, GrowableArray<oop*>& oops, bool has_non_immed_oops);
  ~ShenandoahNMethod();

  nmethod* nm() const {
    return _nm;
  }

  void oops_do(OopClosure* oops, bool fix_relocations = false);

  // Update oops when the nmethod is re-registered
  void update();

  bool has_cset_oops(ShenandoahHeap* heap);
  int oop_count() const {
    return _oops_count + static_cast<int>(nm()->oops_end() - nm()->oops_begin());
  }

  bool has_oops() const {
    return oop_count() > 0;
  }

  static ShenandoahNMethod* for_nmethod(nmethod* nm);

  void assert_alive_and_correct() NOT_DEBUG_RETURN;
  void assert_same_oops(bool allow_dead = false) NOT_DEBUG_RETURN;
  static void assert_no_oops(nmethod* nm, bool allow_dea = false) NOT_DEBUG_RETURN;

private:
  bool has_non_immed_oops() const { return _has_non_immed_oops; }
  static void detect_reloc_oops(nmethod* nm, GrowableArray<oop*>& oops, bool& _has_non_immed_oops);
};


class ShenandoahNMethodTable : public CHeapObj<mtGC> {
private:
  enum {
    minSize = 1024
  };

  ShenandoahHeap* const _heap;
  ShenandoahNMethod**   _array;
  int                   _size;
  int                   _index;
  ShenandoahLock        _lock;
  DEBUG_ONLY(bool       _iteration_in_progress;)

  DEFINE_PAD_MINUS_SIZE(0, DEFAULT_CACHE_LINE_SIZE, sizeof(volatile size_t));
  volatile size_t       _claimed;
  DEFINE_PAD_MINUS_SIZE(1, DEFAULT_CACHE_LINE_SIZE, 0);

public:
  ShenandoahNMethodTable();
  ~ShenandoahNMethodTable();

  void register_nmethod(nmethod* nm);
  void unregister_nmethod(nmethod* nm);

  bool contain(nmethod* nm) const;
  int length() const { return _index; }

  // Table iteration support
  void prepare_for_iteration();
  void finish_iteration();

  template<bool CSET_FILTER>
  void parallel_blobs_do(CodeBlobClosure *f);

  void assert_nmethods_alive_and_correct() NOT_DEBUG_RETURN;
private:
  // Rebuild table and replace current one
  void rebuild(int size);

  bool is_full() const {
    assert(_index <= _size, "Sanity");
    return _index == _size;
  }

  ShenandoahNMethod* at(int index) const;
  int  index_of(nmethod* nm) const;
  void remove(int index);
  void append(ShenandoahNMethod* snm);

  // Logging support
  void log_register_nmethod(nmethod* nm);
  void log_unregister_nmethod(nmethod* nm);
};

#endif // SHARE_GC_SHENANDOAH_SHENANDOAHNMETHOD_HPP

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
#include "gc/shenandoah/shenandoahHeap.hpp"
#include "gc/shenandoah/shenandoahHeuristics.hpp"
#include "gc/shenandoah/shenandoahRuntime.hpp"
#include "gc/shenandoah/c2/shenandoahBarrierSetC2.hpp"
#include "gc/shenandoah/c2/shenandoahSupport.hpp"
#include "opto/graphKit.hpp"
#include "opto/idealKit.hpp"
#include "opto/macro.hpp"

ShenandoahBarrierSetC2* ShenandoahBarrierSetC2::bsc2() {
  return reinterpret_cast<ShenandoahBarrierSetC2*>(BarrierSet::barrier_set()->barrier_set_c2());
}

ShenandoahBarrierSetC2State::ShenandoahBarrierSetC2State(Arena* comp_arena)
  : _shenandoah_barriers(new (comp_arena) GrowableArray<ShenandoahWriteBarrierNode*>(comp_arena, 8,  0, NULL)) {
}

int ShenandoahBarrierSetC2State::shenandoah_barriers_count() const {
  return _shenandoah_barriers->length();
}

ShenandoahWriteBarrierNode* ShenandoahBarrierSetC2State::shenandoah_barrier(int idx) const {
  return _shenandoah_barriers->at(idx);
}


void ShenandoahBarrierSetC2State::add_shenandoah_barrier(ShenandoahWriteBarrierNode * n) {
  assert(!_shenandoah_barriers->contains(n), "duplicate entry in barrier list");
  _shenandoah_barriers->append(n);
}

void ShenandoahBarrierSetC2State::remove_shenandoah_barrier(ShenandoahWriteBarrierNode * n) {
  if (_shenandoah_barriers->contains(n)) {
    _shenandoah_barriers->remove(n);
  }
}

#define __ kit->

Node* ShenandoahBarrierSetC2::shenandoah_read_barrier(GraphKit* kit, Node* obj) const {
  if (ShenandoahReadBarrier) {
    obj = shenandoah_read_barrier_impl(kit, obj, false, true, true);
  }
  return obj;
}

Node* ShenandoahBarrierSetC2::shenandoah_storeval_barrier(GraphKit* kit, Node* obj) const {
  if (ShenandoahStoreValEnqueueBarrier) {
    obj = shenandoah_write_barrier(kit, obj);
    obj = shenandoah_enqueue_barrier(kit, obj);
  }
  if (ShenandoahStoreValReadBarrier) {
    obj = shenandoah_read_barrier_impl(kit, obj, true, false, false);
  }
  return obj;
}

Node* ShenandoahBarrierSetC2::shenandoah_read_barrier_acmp(GraphKit* kit, Node* obj) {
  return shenandoah_read_barrier_impl(kit, obj, true, true, false);
}

Node* ShenandoahBarrierSetC2::shenandoah_read_barrier_impl(GraphKit* kit, Node* obj, bool use_ctrl, bool use_mem, bool allow_fromspace) const {

  const Type* obj_type = obj->bottom_type();
  if (obj_type->higher_equal(TypePtr::NULL_PTR)) {
    return obj;
  }
  const TypePtr* adr_type = ShenandoahBarrierNode::brooks_pointer_type(obj_type);
  Node* mem = use_mem ? __ memory(adr_type) : __ immutable_memory();

  if (! ShenandoahBarrierNode::needs_barrier(&__ gvn(), NULL, obj, mem, allow_fromspace)) {
    // We know it is null, no barrier needed.
    return obj;
  }


  if (obj_type->meet(TypePtr::NULL_PTR) == obj_type->remove_speculative()) {

    // We don't know if it's null or not. Need null-check.
    enum { _not_null_path = 1, _null_path, PATH_LIMIT };
    RegionNode* region = new RegionNode(PATH_LIMIT);
    Node*       phi    = new PhiNode(region, obj_type);
    Node* null_ctrl = __ top();
    Node* not_null_obj = __ null_check_oop(obj, &null_ctrl);

    region->init_req(_null_path, null_ctrl);
    phi   ->init_req(_null_path, __ zerocon(T_OBJECT));

    Node* ctrl = use_ctrl ? __ control() : NULL;
    ShenandoahReadBarrierNode* rb = new ShenandoahReadBarrierNode(ctrl, mem, not_null_obj, allow_fromspace);
    Node* n = __ gvn().transform(rb);

    region->init_req(_not_null_path, __ control());
    phi   ->init_req(_not_null_path, n);

    __ set_control(__ gvn().transform(region));
    __ record_for_igvn(region);
    return __ gvn().transform(phi);

  } else {
    // We know it is not null. Simple barrier is sufficient.
    Node* ctrl = use_ctrl ? __ control() : NULL;
    ShenandoahReadBarrierNode* rb = new ShenandoahReadBarrierNode(ctrl, mem, obj, allow_fromspace);
    Node* n = __ gvn().transform(rb);
    __ record_for_igvn(n);
    return n;
  }
}

Node* ShenandoahBarrierSetC2::shenandoah_write_barrier_helper(GraphKit* kit, Node* obj, const TypePtr* adr_type) const {
  ShenandoahWriteBarrierNode* wb = new ShenandoahWriteBarrierNode(kit->C, kit->control(), kit->memory(adr_type), obj);
  Node* n = __ gvn().transform(wb);
  if (n == wb) { // New barrier needs memory projection.
    Node* proj = __ gvn().transform(new ShenandoahWBMemProjNode(n));
    __ set_memory(proj, adr_type);
  }
  return n;
}

Node* ShenandoahBarrierSetC2::shenandoah_write_barrier(GraphKit* kit, Node* obj) const {

  if (ShenandoahWriteBarrier) {
    obj = shenandoah_write_barrier_impl(kit, obj);
  }
  return obj;
}

Node* ShenandoahBarrierSetC2::shenandoah_write_barrier_impl(GraphKit* kit, Node* obj) const {
  if (! ShenandoahBarrierNode::needs_barrier(&__ gvn(), NULL, obj, NULL, true)) {
    return obj;
  }
  const Type* obj_type = obj->bottom_type();
  const TypePtr* adr_type = ShenandoahBarrierNode::brooks_pointer_type(obj_type);
  Node* n = shenandoah_write_barrier_helper(kit, obj, adr_type);
  __ record_for_igvn(n);
  return n;
}

void ShenandoahBarrierSetC2::shenandoah_update_matrix(GraphKit* kit, Node* adr, Node* val) const {
  if (!UseShenandoahMatrix) {
    return;
  }

  assert(val != NULL, "checked before");
  if (adr == NULL) {
    return; // Nothing to do
  }
  assert(adr != NULL, "must not happen");
  if (val->bottom_type()->higher_equal(TypePtr::NULL_PTR)) {
    // Nothing to do.
    return;
  }

  ShenandoahConnectionMatrix* matrix = ShenandoahHeap::heap()->connection_matrix();

  enum { _set_path = 1, _already_set_path, _val_null_path, PATH_LIMIT };
  RegionNode* region = new RegionNode(PATH_LIMIT);
  Node* prev_mem = __ memory(Compile::AliasIdxRaw);
  Node* memphi    = PhiNode::make(region, prev_mem, Type::MEMORY, TypeRawPtr::BOTTOM);
  Node* null_ctrl = __ top();
  Node* not_null_val = __ null_check_oop(val, &null_ctrl);

  // Null path: nothing to do.
  region->init_req(_val_null_path, null_ctrl);
  memphi->init_req(_val_null_path, prev_mem);

  // Not null path. Update the matrix.

  // This uses a fast calculation for the matrix address. For a description,
  // see src/share/vm/gc/shenandoah/shenandoahConnectionMatrix.inline.hpp,
  // ShenandoahConnectionMatrix::compute_address(const void* from, const void* to).
  address heap_base = ShenandoahHeap::heap()->base();
  jint stride = matrix->stride_jint();
  jint rs = ShenandoahHeapRegion::region_size_bytes_shift_jint();

  guarantee(stride < ShenandoahHeapRegion::region_size_bytes_jint(), "sanity");
  guarantee(is_aligned(heap_base, ShenandoahHeapRegion::region_size_bytes()), "sanity");

  Node* magic_con = __ MakeConX((jlong) matrix->matrix_addr() - ((jlong) heap_base >> rs) * (stride + 1));

  // Compute addr part
  Node* adr_idx = __ gvn().transform(new CastP2XNode(__ control(), adr));
  adr_idx = __ gvn().transform(new URShiftXNode(adr_idx, __ intcon(rs)));

  // Compute new_val part
  Node* val_idx = __ gvn().transform(new CastP2XNode(__ control(), not_null_val));
  val_idx = __ gvn().transform(new URShiftXNode(val_idx, __ intcon(rs)));
  val_idx = __ gvn().transform(new MulXNode(val_idx, __ MakeConX(stride)));

  // Add everything up
  adr_idx = __ gvn().transform(new AddXNode(adr_idx, val_idx));
  adr_idx = __ gvn().transform(new CastX2PNode(adr_idx));
  Node* matrix_adr = __ gvn().transform(new AddPNode(__ top(), adr_idx, magic_con));

  // Load current value
  const TypePtr* adr_type = TypeRawPtr::BOTTOM;
  Node* current = __ gvn().transform(LoadNode::make(__ gvn(), __ control(), __ memory(Compile::AliasIdxRaw),
                                                    matrix_adr, adr_type, TypeInt::INT, T_BYTE, MemNode::unordered));

  // Check if already set
  Node* cmp_set = __ gvn().transform(new CmpINode(current, __ intcon(0)));
  Node* cmp_set_bool = __ gvn().transform(new BoolNode(cmp_set, BoolTest::eq));
  IfNode* cmp_iff = __ create_and_map_if(__ control(), cmp_set_bool, PROB_MIN, COUNT_UNKNOWN);

  Node* if_not_set = __ gvn().transform(new IfTrueNode(cmp_iff));
  Node* if_set = __ gvn().transform(new IfFalseNode(cmp_iff));

  // Already set, exit
  __ set_control(if_set);
  region->init_req(_already_set_path, __ control());
  memphi->init_req(_already_set_path, prev_mem);

  // Not set: do the store, and finish up
  __ set_control(if_not_set);
  Node* store = __ gvn().transform(StoreNode::make(__ gvn(), __ control(), __ memory(Compile::AliasIdxRaw),
                                                   matrix_adr, adr_type, __ intcon(1), T_BYTE, MemNode::unordered));
  region->init_req(_set_path, __ control());
  memphi->init_req(_set_path, store);

  // Merge control flows and memory.
  __ set_control(__ gvn().transform(region));
  __ record_for_igvn(region);
  __ set_memory(__ gvn().transform(memphi), Compile::AliasIdxRaw);
}

bool ShenandoahBarrierSetC2::satb_can_remove_pre_barrier(GraphKit* kit, PhaseTransform* phase, Node* adr,
                                                         BasicType bt, uint adr_idx) const {
  intptr_t offset = 0;
  Node* base = AddPNode::Ideal_base_and_offset(adr, phase, offset);
  AllocateNode* alloc = AllocateNode::Ideal_allocation(base, phase);

  if (offset == Type::OffsetBot) {
    return false; // cannot unalias unless there are precise offsets
  }

  if (alloc == NULL) {
    return false; // No allocation found
  }

  intptr_t size_in_bytes = type2aelembytes(bt);

  Node* mem = __ memory(adr_idx); // start searching here...

  for (int cnt = 0; cnt < 50; cnt++) {

    if (mem->is_Store()) {

      Node* st_adr = mem->in(MemNode::Address);
      intptr_t st_offset = 0;
      Node* st_base = AddPNode::Ideal_base_and_offset(st_adr, phase, st_offset);

      if (st_base == NULL) {
        break; // inscrutable pointer
      }

      // Break we have found a store with same base and offset as ours so break
      if (st_base == base && st_offset == offset) {
        break;
      }

      if (st_offset != offset && st_offset != Type::OffsetBot) {
        const int MAX_STORE = BytesPerLong;
        if (st_offset >= offset + size_in_bytes ||
            st_offset <= offset - MAX_STORE ||
            st_offset <= offset - mem->as_Store()->memory_size()) {
          // Success:  The offsets are provably independent.
          // (You may ask, why not just test st_offset != offset and be done?
          // The answer is that stores of different sizes can co-exist
          // in the same sequence of RawMem effects.  We sometimes initialize
          // a whole 'tile' of array elements with a single jint or jlong.)
          mem = mem->in(MemNode::Memory);
          continue; // advance through independent store memory
        }
      }

      if (st_base != base
          && MemNode::detect_ptr_independence(base, alloc, st_base,
                                              AllocateNode::Ideal_allocation(st_base, phase),
                                              phase)) {
        // Success:  The bases are provably independent.
        mem = mem->in(MemNode::Memory);
        continue; // advance through independent store memory
      }
    } else if (mem->is_Proj() && mem->in(0)->is_Initialize()) {

      InitializeNode* st_init = mem->in(0)->as_Initialize();
      AllocateNode* st_alloc = st_init->allocation();

      // Make sure that we are looking at the same allocation site.
      // The alloc variable is guaranteed to not be null here from earlier check.
      if (alloc == st_alloc) {
        // Check that the initialization is storing NULL so that no previous store
        // has been moved up and directly write a reference
        Node* captured_store = st_init->find_captured_store(offset,
                                                            type2aelembytes(T_OBJECT),
                                                            phase);
        if (captured_store == NULL || captured_store == st_init->zero_memory()) {
          return true;
        }
      }
    }

    // Unless there is an explicit 'continue', we must bail out here,
    // because 'mem' is an inscrutable memory state (e.g., a call).
    break;
  }

  return false;
}

#undef __
#define __ ideal.

void ShenandoahBarrierSetC2::satb_write_barrier_pre(GraphKit* kit,
                                                    bool do_load,
                                                    Node* obj,
                                                    Node* adr,
                                                    uint alias_idx,
                                                    Node* val,
                                                    const TypeOopPtr* val_type,
                                                    Node* pre_val,
                                                    BasicType bt) const {

  // Some sanity checks
  // Note: val is unused in this routine.

  if (do_load) {
    // We need to generate the load of the previous value
    assert(obj != NULL, "must have a base");
    assert(adr != NULL, "where are loading from?");
    assert(pre_val == NULL, "loaded already?");
    assert(val_type != NULL, "need a type");

    if (ReduceInitialCardMarks
        && satb_can_remove_pre_barrier(kit, &kit->gvn(), adr, bt, alias_idx)) {
      return;
    }

  } else {
    // In this case both val_type and alias_idx are unused.
    assert(pre_val != NULL, "must be loaded already");
    // Nothing to be done if pre_val is null.
    if (pre_val->bottom_type() == TypePtr::NULL_PTR) return;
    assert(pre_val->bottom_type()->basic_type() == T_OBJECT, "or we shouldn't be here");
  }
  assert(bt == T_OBJECT, "or we shouldn't be here");

  IdealKit ideal(kit, true);

  Node* tls = __ thread(); // ThreadLocalStorage

  Node* no_ctrl = NULL;
  Node* no_base = __ top();
  Node* zero  = __ ConI(0);
  Node* zeroX = __ ConX(0);

  float likely  = PROB_LIKELY(0.999);
  float unlikely  = PROB_UNLIKELY(0.999);

  BasicType active_type = in_bytes(SATBMarkQueue::byte_width_of_active()) == 4 ? T_INT : T_BYTE;
  assert(in_bytes(SATBMarkQueue::byte_width_of_active()) == 4 || in_bytes(SATBMarkQueue::byte_width_of_active()) == 1, "flag width");

  // Offsets into the thread
  const int marking_offset = in_bytes(ShenandoahThreadLocalData::satb_mark_queue_active_offset());
  const int index_offset   = in_bytes(ShenandoahThreadLocalData::satb_mark_queue_index_offset());
  const int buffer_offset  = in_bytes(ShenandoahThreadLocalData::satb_mark_queue_buffer_offset());

  // Now the actual pointers into the thread
  Node* marking_adr = __ AddP(no_base, tls, __ ConX(marking_offset));
  Node* buffer_adr  = __ AddP(no_base, tls, __ ConX(buffer_offset));
  Node* index_adr   = __ AddP(no_base, tls, __ ConX(index_offset));

  // Now some of the values
  Node* marking;
  Node* gc_state = __ AddP(no_base, tls, __ ConX(in_bytes(ShenandoahThreadLocalData::gc_state_offset())));
  Node* ld = __ load(__ ctrl(), gc_state, TypeInt::BYTE, T_BYTE, Compile::AliasIdxRaw);
  marking = __ AndI(ld, __ ConI(ShenandoahHeap::MARKING));
  assert(ShenandoahWriteBarrierNode::is_gc_state_load(ld), "Should match the shape");

  // if (!marking)
  __ if_then(marking, BoolTest::ne, zero, unlikely); {
    BasicType index_bt = TypeX_X->basic_type();
    assert(sizeof(size_t) == type2aelembytes(index_bt), "Loading G1 SATBMarkQueue::_index with wrong size.");
    Node* index   = __ load(__ ctrl(), index_adr, TypeX_X, index_bt, Compile::AliasIdxRaw);

    if (do_load) {
      // load original value
      // alias_idx correct??
      pre_val = __ load(__ ctrl(), adr, val_type, bt, alias_idx);
    }

    // if (pre_val != NULL)
    __ if_then(pre_val, BoolTest::ne, kit->null()); {
      Node* buffer  = __ load(__ ctrl(), buffer_adr, TypeRawPtr::NOTNULL, T_ADDRESS, Compile::AliasIdxRaw);

      // is the queue for this thread full?
      __ if_then(index, BoolTest::ne, zeroX, likely); {

        // decrement the index
        Node* next_index = kit->gvn().transform(new SubXNode(index, __ ConX(sizeof(intptr_t))));

        // Now get the buffer location we will log the previous value into and store it
        Node *log_addr = __ AddP(no_base, buffer, next_index);
        __ store(__ ctrl(), log_addr, pre_val, T_OBJECT, Compile::AliasIdxRaw, MemNode::unordered);
        // update the index
        __ store(__ ctrl(), index_adr, next_index, index_bt, Compile::AliasIdxRaw, MemNode::unordered);

      } __ else_(); {

        // logging buffer is full, call the runtime
        const TypeFunc *tf = ShenandoahBarrierSetC2::write_ref_field_pre_entry_Type();
        __ make_leaf_call(tf, CAST_FROM_FN_PTR(address, ShenandoahRuntime::write_ref_field_pre_entry), "shenandoah_wb_pre", pre_val, tls);
      } __ end_if();  // (!index)
    } __ end_if();  // (pre_val != NULL)
  } __ end_if();  // (!marking)

  // Final sync IdealKit and GraphKit.
  kit->final_sync(ideal);

  if (ShenandoahSATBBarrier && adr != NULL) {
    Node* c = kit->control();
    Node* call = c->in(1)->in(1)->in(1)->in(0);
    assert(call->is_shenandoah_wb_pre_call(), "shenandoah_wb_pre call expected");
    call->add_req(adr);
  }
}

void ShenandoahBarrierSetC2::shenandoah_write_barrier_pre(GraphKit* kit,
                                                          bool do_load,
                                                          Node* obj,
                                                          Node* adr,
                                                          uint alias_idx,
                                                          Node* val,
                                                          const TypeOopPtr* val_type,
                                                          Node* pre_val,
                                                          BasicType bt) const {

  IdealKit ideal(kit);

  // Some sanity checks
  // Note: val is unused in this routine.

  if (val != NULL) {
    shenandoah_update_matrix(kit, adr, val);
    ideal.sync_kit(kit);
  }

  kit->sync_kit(ideal);
  if (ShenandoahSATBBarrier) {
    satb_write_barrier_pre(kit, do_load, obj, adr, alias_idx, val, val_type, pre_val, bt);
  }
  ideal.sync_kit(kit);

  kit->final_sync(ideal);
}

Node* ShenandoahBarrierSetC2::shenandoah_enqueue_barrier(GraphKit* kit, Node* pre_val) const {
  return kit->gvn().transform(new ShenandoahEnqueueBarrierNode(pre_val));
}

// Helper that guards and inserts a pre-barrier.
void ShenandoahBarrierSetC2::insert_pre_barrier(GraphKit* kit, Node* base_oop, Node* offset,
                                                Node* pre_val, bool need_mem_bar) const {
  // We could be accessing the referent field of a reference object. If so, when G1
  // is enabled, we need to log the value in the referent field in an SATB buffer.
  // This routine performs some compile time filters and generates suitable
  // runtime filters that guard the pre-barrier code.
  // Also add memory barrier for non volatile load from the referent field
  // to prevent commoning of loads across safepoint.

  // Some compile time checks.

  // If offset is a constant, is it java_lang_ref_Reference::_reference_offset?
  const TypeX* otype = offset->find_intptr_t_type();
  if (otype != NULL && otype->is_con() &&
      otype->get_con() != java_lang_ref_Reference::referent_offset) {
    // Constant offset but not the reference_offset so just return
    return;
  }

  // We only need to generate the runtime guards for instances.
  const TypeOopPtr* btype = base_oop->bottom_type()->isa_oopptr();
  if (btype != NULL) {
    if (btype->isa_aryptr()) {
      // Array type so nothing to do
      return;
    }

    const TypeInstPtr* itype = btype->isa_instptr();
    if (itype != NULL) {
      // Can the klass of base_oop be statically determined to be
      // _not_ a sub-class of Reference and _not_ Object?
      ciKlass* klass = itype->klass();
      if ( klass->is_loaded() &&
          !klass->is_subtype_of(kit->env()->Reference_klass()) &&
          !kit->env()->Object_klass()->is_subtype_of(klass)) {
        return;
      }
    }
  }

  // The compile time filters did not reject base_oop/offset so
  // we need to generate the following runtime filters
  //
  // if (offset == java_lang_ref_Reference::_reference_offset) {
  //   if (instance_of(base, java.lang.ref.Reference)) {
  //     pre_barrier(_, pre_val, ...);
  //   }
  // }

  float likely   = PROB_LIKELY(  0.999);
  float unlikely = PROB_UNLIKELY(0.999);

  IdealKit ideal(kit);

  Node* referent_off = __ ConX(java_lang_ref_Reference::referent_offset);

  __ if_then(offset, BoolTest::eq, referent_off, unlikely); {
      // Update graphKit memory and control from IdealKit.
      kit->sync_kit(ideal);

      Node* ref_klass_con = kit->makecon(TypeKlassPtr::make(kit->env()->Reference_klass()));
      Node* is_instof = kit->gen_instanceof(base_oop, ref_klass_con);

      // Update IdealKit memory and control from graphKit.
      __ sync_kit(kit);

      Node* one = __ ConI(1);
      // is_instof == 0 if base_oop == NULL
      __ if_then(is_instof, BoolTest::eq, one, unlikely); {

        // Update graphKit from IdeakKit.
        kit->sync_kit(ideal);

        // Use the pre-barrier to record the value in the referent field
        satb_write_barrier_pre(kit, false /* do_load */,
                               NULL /* obj */, NULL /* adr */, max_juint /* alias_idx */, NULL /* val */, NULL /* val_type */,
                               pre_val /* pre_val */,
                               T_OBJECT);
        if (need_mem_bar) {
          // Add memory barrier to prevent commoning reads from this field
          // across safepoint since GC can change its value.
          kit->insert_mem_bar(Op_MemBarCPUOrder);
        }
        // Update IdealKit from graphKit.
        __ sync_kit(kit);

      } __ end_if(); // _ref_type != ref_none
  } __ end_if(); // offset == referent_offset

  // Final sync IdealKit and GraphKit.
  kit->final_sync(ideal);
}

#undef __

const TypeFunc* ShenandoahBarrierSetC2::write_ref_field_pre_entry_Type() {
  const Type **fields = TypeTuple::fields(2);
  fields[TypeFunc::Parms+0] = TypeInstPtr::NOTNULL; // original field value
  fields[TypeFunc::Parms+1] = TypeRawPtr::NOTNULL; // thread
  const TypeTuple *domain = TypeTuple::make(TypeFunc::Parms+2, fields);

  // create result type (range)
  fields = TypeTuple::fields(0);
  const TypeTuple *range = TypeTuple::make(TypeFunc::Parms+0, fields);

  return TypeFunc::make(domain, range);
}

const TypeFunc* ShenandoahBarrierSetC2::shenandoah_clone_barrier_Type() {
  const Type **fields = TypeTuple::fields(1);
  fields[TypeFunc::Parms+0] = TypeInstPtr::NOTNULL; // original field value
  const TypeTuple *domain = TypeTuple::make(TypeFunc::Parms+1, fields);

  // create result type (range)
  fields = TypeTuple::fields(0);
  const TypeTuple *range = TypeTuple::make(TypeFunc::Parms+0, fields);

  return TypeFunc::make(domain, range);
}

const TypeFunc* ShenandoahBarrierSetC2::shenandoah_write_barrier_Type() {
  const Type **fields = TypeTuple::fields(1);
  fields[TypeFunc::Parms+0] = TypeInstPtr::NOTNULL; // original field value
  const TypeTuple *domain = TypeTuple::make(TypeFunc::Parms+1, fields);

  // create result type (range)
  fields = TypeTuple::fields(1);
  fields[TypeFunc::Parms+0] = TypeInstPtr::NOTNULL;
  const TypeTuple *range = TypeTuple::make(TypeFunc::Parms+1, fields);

  return TypeFunc::make(domain, range);
}

void ShenandoahBarrierSetC2::resolve_address(C2Access& access) const {
  const TypePtr* adr_type = access.addr().type();

  if ((access.decorators() & IN_NATIVE) == 0 && (adr_type->isa_instptr() || adr_type->isa_aryptr())) {
    int off = adr_type->is_ptr()->offset();
    int base_off = adr_type->isa_instptr() ? instanceOopDesc::base_offset_in_bytes() :
      arrayOopDesc::base_offset_in_bytes(adr_type->is_aryptr()->elem()->array_element_basic_type());
    assert(off != Type::OffsetTop, "unexpected offset");
    if (off == Type::OffsetBot || off >= base_off) {
      DecoratorSet decorators = access.decorators();
      bool is_write = (decorators & C2_WRITE_ACCESS) != 0;
      GraphKit* kit = access.kit();
      Node* adr = access.addr().node();
      assert(adr->is_AddP(), "unexpected address shape");
      Node* base = adr->in(AddPNode::Base);

      if (is_write) {
        base = shenandoah_write_barrier(kit, base);
      } else {
        if (adr_type->isa_instptr()) {
          Compile* C = kit->C;
          ciField* field = C->alias_type(adr_type)->field();

          // Insert read barrier for Shenandoah.
          if (field != NULL &&
              ((ShenandoahOptimizeStaticFinals   && field->is_static()  && field->is_final()) ||
               (ShenandoahOptimizeInstanceFinals && !field->is_static() && field->is_final()) ||
               (ShenandoahOptimizeStableFinals   && field->is_stable()))) {
            // Skip the barrier for special fields
          } else {
            base = shenandoah_read_barrier(kit, base);
          }
        } else {
          base = shenandoah_read_barrier(kit, base);
        }
      }
      if (base != adr->in(AddPNode::Base)) {
        Node* address = adr->in(AddPNode::Address);

        if (address->is_AddP()) {
          assert(address->in(AddPNode::Base) == adr->in(AddPNode::Base), "unexpected address shape");
          assert(!address->in(AddPNode::Address)->is_AddP(), "unexpected address shape");
          assert(address->in(AddPNode::Address) == adr->in(AddPNode::Base), "unexpected address shape");
          address = address->clone();
          address->set_req(AddPNode::Base, base);
          address->set_req(AddPNode::Address, base);
          address = kit->gvn().transform(address);
        } else {
          assert(address == adr->in(AddPNode::Base), "unexpected address shape");
          address = base;
        }
        adr = adr->clone();
        adr->set_req(AddPNode::Base, base);
        adr->set_req(AddPNode::Address, address);
        adr = kit->gvn().transform(adr);
        access.addr().set_node(adr);
      }
    }
  }
}

Node* ShenandoahBarrierSetC2::store_at_resolved(C2Access& access, C2AccessValue& val) const {
  DecoratorSet decorators = access.decorators();
  GraphKit* kit = access.kit();

  const TypePtr* adr_type = access.addr().type();
  Node* adr = access.addr().node();

  bool anonymous = (decorators & ON_UNKNOWN_OOP_REF) != 0;
  bool on_heap = (decorators & IN_HEAP) != 0;

  if (!access.is_oop() || (!on_heap && !anonymous)) {
    return BarrierSetC2::store_at_resolved(access, val);
  }

  uint adr_idx = kit->C->get_alias_index(adr_type);
  assert(adr_idx != Compile::AliasIdxTop, "use other store_to_memory factory" );
  Node* value = val.node();
  value = shenandoah_storeval_barrier(kit, value);
  val.set_node(value);
  shenandoah_write_barrier_pre(kit, true /* do_load */, /*kit->control(),*/ access.base(), adr, adr_idx, val.node(),
              static_cast<const TypeOopPtr*>(val.type()), NULL /* pre_val */, access.type());
  return BarrierSetC2::store_at_resolved(access, val);
}


Node* ShenandoahBarrierSetC2::load_at_resolved(C2Access& access, const Type* val_type) const {
  DecoratorSet decorators = access.decorators();
  GraphKit* kit = access.kit();

  Node* adr = access.addr().node();
  Node* obj = access.base();

  bool mismatched = (decorators & C2_MISMATCHED) != 0;
  bool unknown = (decorators & ON_UNKNOWN_OOP_REF) != 0;
  bool on_heap = (decorators & IN_HEAP) != 0;
  bool on_weak = (decorators & ON_WEAK_OOP_REF) != 0;
  bool is_unordered = (decorators & MO_UNORDERED) != 0;
  bool need_cpu_mem_bar = !is_unordered || mismatched || !on_heap;

  Node* offset = adr->is_AddP() ? adr->in(AddPNode::Offset) : kit->top();
  Node* load = BarrierSetC2::load_at_resolved(access, val_type);

  // If we are reading the value of the referent field of a Reference
  // object (either by using Unsafe directly or through reflection)
  // then, if SATB is enabled, we need to record the referent in an
  // SATB log buffer using the pre-barrier mechanism.
  // Also we need to add memory barrier to prevent commoning reads
  // from this field across safepoint since GC can change its value.
  bool need_read_barrier = ShenandoahKeepAliveBarrier &&
    (on_heap && (on_weak || (unknown && offset != kit->top() && obj != kit->top())));

  if (!access.is_oop() || !need_read_barrier) {
    return load;
  }

  if (on_weak) {
    // Use the pre-barrier to record the value in the referent field
    satb_write_barrier_pre(kit, false /* do_load */,
                           NULL /* obj */, NULL /* adr */, max_juint /* alias_idx */, NULL /* val */, NULL /* val_type */,
                           load /* pre_val */, T_OBJECT);
    // Add memory barrier to prevent commoning reads from this field
    // across safepoint since GC can change its value.
    kit->insert_mem_bar(Op_MemBarCPUOrder);
  } else if (unknown) {
    // We do not require a mem bar inside pre_barrier if need_mem_bar
    // is set: the barriers would be emitted by us.
    insert_pre_barrier(kit, obj, offset, load, !need_cpu_mem_bar);
  }

  return load;
}

Node* ShenandoahBarrierSetC2::atomic_cmpxchg_val_at_resolved(C2AtomicAccess& access, Node* expected_val,
                                                   Node* val, const Type* value_type) const {
  GraphKit* kit = access.kit();
  if (access.is_oop()) {
    val = shenandoah_storeval_barrier(kit, val);
    shenandoah_write_barrier_pre(kit, false /* do_load */,
                                 NULL, NULL, max_juint, NULL, NULL,
                                 expected_val /* pre_val */, T_OBJECT);

  }
  return BarrierSetC2::atomic_cmpxchg_val_at_resolved(access, expected_val, val, value_type);
}

Node* ShenandoahBarrierSetC2::atomic_cmpxchg_bool_at_resolved(C2AtomicAccess& access, Node* expected_val,
                                                              Node* val, const Type* value_type) const {
  GraphKit* kit = access.kit();
  if (access.is_oop()) {
    val = shenandoah_storeval_barrier(kit, val);
    shenandoah_write_barrier_pre(kit, false /* do_load */,
                                 NULL, NULL, max_juint, NULL, NULL,
                                 expected_val /* pre_val */, T_OBJECT);
  }
  return BarrierSetC2::atomic_cmpxchg_bool_at_resolved(access, expected_val, val, value_type);
}

Node* ShenandoahBarrierSetC2::atomic_xchg_at_resolved(C2AtomicAccess& access, Node* val, const Type* value_type) const {
  GraphKit* kit = access.kit();
  if (access.is_oop()) {
    val = shenandoah_storeval_barrier(kit, val);
  }
  Node* result = BarrierSetC2::atomic_xchg_at_resolved(access, val, value_type);
  if (access.is_oop()) {
    shenandoah_write_barrier_pre(kit, false /* do_load */,
                                 NULL, NULL, max_juint, NULL, NULL,
                                 result /* pre_val */, T_OBJECT);
  }
  return result;
}

void ShenandoahBarrierSetC2::clone(GraphKit* kit, Node* src, Node* dst, Node* size, bool is_array) const {
  assert(!src->is_AddP(), "unexpected input");
  src = shenandoah_read_barrier(kit, src);
  BarrierSetC2::clone(kit, src, dst, size, is_array);
}

Node* ShenandoahBarrierSetC2::resolve_for_read(GraphKit* kit, Node* n) const {
  return shenandoah_read_barrier(kit, n);
}

Node* ShenandoahBarrierSetC2::resolve_for_write(GraphKit* kit, Node* n) const {
  return shenandoah_write_barrier(kit, n);
}

Node* ShenandoahBarrierSetC2::cmpoop_cmp(GraphKit* kit, Node* a, Node* b) const {
  if (ShenandoahAcmpBarrier && ShenandoahVerifyOptoBarriers || ShenandoahAcmpWBBarrier) {
    a = shenandoah_write_barrier(kit, a);
    b = shenandoah_write_barrier(kit, b);
  }
  return kit->gvn().transform(new CmpPNode(b, a));
}

/**
 * In Shenandoah, we need barriers on acmp (and similar instructions that compare two
 * oops) to avoid false negatives. If it compares a from-space and a to-space
 * copy of an object, a regular acmp would return false, even though both are
 * the same. The acmp barrier compares the two objects, and when they are
 * *not equal* it does a read-barrier on both, and compares them again. When it
 * failed because of different copies of the object, we know that the object
 * must already have been evacuated (and therefore doesn't require a write-barrier).
 */
void ShenandoahBarrierSetC2::cmpoop_if(GraphKit* kit, Node* tst, float true_prob, float cnt,
                                       Node*& taken_branch, Node*& untaken_branch,
                                       Node*& taken_memory, Node*& untaken_memory) const {
  IfNode* iff = kit->create_and_map_if(kit->control(), tst, true_prob, cnt);
  assert(iff->_prob > 0.0f,"Optimizer made bad probability in parser");
  taken_branch   = new IfTrueNode(iff);
  untaken_branch = new IfFalseNode(iff);

  taken_branch = kit->gvn().transform(taken_branch);
  untaken_branch = kit->gvn().transform(untaken_branch);

  assert(taken_memory == NULL && untaken_memory == NULL, "unexpected memory inputs");
  if (!ShenandoahAcmpBarrier || ShenandoahVerifyOptoBarriers || ShenandoahAcmpWBBarrier) {
    return;
  }
  if (taken_branch->is_top() || untaken_branch->is_top()) {
    // one of the branches is known to be untaken
    return;
  }
  assert(taken_branch->is_IfProj() && untaken_branch->is_IfProj(), "if projections only");
  assert(taken_branch->in(0) == untaken_branch->in(0), "should come from same if");
  BoolNode* bol = iff->in(1)->as_Bool();
  Node* cmp = bol->in(1);
  assert(cmp->Opcode() == Op_CmpP, "only oops comparisons");
  Node* a = cmp->in(1);
  Node* b = cmp->in(2);
  const Type* a_type = kit->gvn().type(a);
  const Type* b_type = kit->gvn().type(b);
  if (a_type->higher_equal(TypePtr::NULL_PTR) || b_type->higher_equal(TypePtr::NULL_PTR)) {
    // We know one arg is gonna be null. No need for barriers.
    return;
  }

  const TypePtr* a_adr_type = ShenandoahBarrierNode::brooks_pointer_type(a_type);
  const TypePtr* b_adr_type = ShenandoahBarrierNode::brooks_pointer_type(b_type);
  if ((! ShenandoahBarrierNode::needs_barrier(&kit->gvn(), NULL, a, kit->memory(a_adr_type), false)) &&
      (! ShenandoahBarrierNode::needs_barrier(&kit->gvn(), NULL, b, kit->memory(b_adr_type), false))) {
    // We know both args are in to-space already. No acmp barrier needed.
    return;
  }

  Node* equal_path = iff->proj_out(true);
  Node* not_equal_path = iff->proj_out(false);

  if (bol->_test._test == BoolTest::ne) {
    swap(equal_path, not_equal_path);
  }

  Node* init_equal_path = equal_path;
  Node* init_not_equal_path = not_equal_path;

  uint alias_a = kit->C->get_alias_index(a_adr_type);
  uint alias_b = kit->C->get_alias_index(b_adr_type);

  Node* equal_memory = NULL;
  Node* not_equal_memory = NULL;

  RegionNode* region = new RegionNode(3);
  region->init_req(1, equal_path);
  PhiNode* mem_phi = NULL;
  if (alias_a == alias_b) {
    mem_phi = PhiNode::make(region, kit->memory(alias_a), Type::MEMORY, kit->C->get_adr_type(alias_a));
  } else {
    Node* mem = kit->reset_memory();
    mem_phi = PhiNode::make(region, mem, Type::MEMORY, TypePtr::BOTTOM);
    kit->set_all_memory(mem);
  }

  kit->set_control(not_equal_path);

  Node* mb = NULL;
  if (alias_a == alias_b) {
    Node* mem = kit->reset_memory();
    mb = MemBarNode::make(kit->C, Op_MemBarAcquire, alias_a);
    mb->init_req(TypeFunc::Control, kit->control());
    mb->init_req(TypeFunc::Memory, mem);
    Node* membar = kit->gvn().transform(mb);
    kit->set_control(kit->gvn().transform(new ProjNode(membar, TypeFunc::Control)));
    Node* newmem = kit->gvn().transform(new ProjNode(membar, TypeFunc::Memory));
    kit->set_all_memory(mem);
    kit->set_memory(newmem, alias_a);
  } else {
    mb = kit->insert_mem_bar(Op_MemBarAcquire);
  }

  ShenandoahBarrierSetC2* bs = (ShenandoahBarrierSetC2*) BarrierSet::barrier_set()->barrier_set_c2();
  a = bs->shenandoah_read_barrier_acmp(kit, a);
  b = bs->shenandoah_read_barrier_acmp(kit, b);

  Node* cmp2 = kit->gvn().transform(new CmpPNode(a, b));
  Node* bol2 = bol->clone();
  bol2->set_req(1, cmp2);
  bol2 = kit->gvn().transform(bol2);
  Node* iff2 = iff->clone();
  iff2->set_req(0, kit->control());
  iff2->set_req(1, bol2);
  kit->gvn().set_type(iff2, kit->gvn().type(iff));
  Node* equal_path2 = equal_path->clone();
  equal_path2->set_req(0, iff2);
  equal_path2 = kit->gvn().transform(equal_path2);
  Node* not_equal_path2 = not_equal_path->clone();
  not_equal_path2->set_req(0, iff2);
  not_equal_path2 = kit->gvn().transform(not_equal_path2);

  region->init_req(2, equal_path2);
  not_equal_memory = kit->reset_memory();
  not_equal_path = not_equal_path2;

  kit->set_all_memory(not_equal_memory);

  if (alias_a == alias_b) {
    mem_phi->init_req(2, kit->memory(alias_a));
    kit->set_memory(mem_phi, alias_a);
  } else {
    mem_phi->init_req(2, kit->reset_memory());
  }

  kit->record_for_igvn(mem_phi);
  kit->gvn().set_type(mem_phi, Type::MEMORY);

  if (alias_a == alias_b) {
    equal_memory = kit->reset_memory();
  } else {
    equal_memory = mem_phi;
  }

  assert(kit->map()->memory() == NULL, "no live memory state");
  equal_path = kit->gvn().transform(region);

  if (taken_branch == init_equal_path) {
    assert(untaken_branch == init_not_equal_path, "inconsistent");
    taken_branch = equal_path;
    untaken_branch = not_equal_path;
    taken_memory = equal_memory;
    untaken_memory = not_equal_memory;
  } else {
    assert(taken_branch == init_not_equal_path, "inconsistent");
    assert(untaken_branch == init_equal_path, "inconsistent");
    taken_branch = not_equal_path;
    untaken_branch = equal_path;
    taken_memory = not_equal_memory;
    untaken_memory = equal_memory;
  }
}

void ShenandoahBarrierSetC2::resolve_for_cmpoop(GraphKit* kit, Node*& a, Node*& b) const {
  a = shenandoah_write_barrier(kit, a);
  b = shenandoah_write_barrier(kit, b);
}


// Support for GC barriers emitted during parsing
bool ShenandoahBarrierSetC2::is_gc_barrier_node(Node* node) const {
  if (node->Opcode() != Op_CallLeaf) {
    return false;
  }
  CallLeafNode *call = node->as_CallLeaf();
  if (call->_name == NULL) {
    return false;
  }

  return strcmp(call->_name, "shenandoah_clone_barrier") == 0 ||
         strcmp(call->_name, "shenandoah_cas_obj") == 0 ||
         strcmp(call->_name, "shenandoah_wb_pre") == 0;
}

Node* ShenandoahBarrierSetC2::step_over_gc_barrier(Node* c) const {
  // Currently not needed.
  return c;
}

Node* ShenandoahBarrierSetC2::peek_thru_gc_barrier(Node* v) const {
  return ShenandoahBarrierNode::skip_through_barrier(v);
}

bool ShenandoahBarrierSetC2::array_copy_requires_gc_barriers(bool tightly_coupled_alloc, BasicType type, bool is_clone, ArrayCopyPhase phase) const {
  bool is_oop = type == T_OBJECT || type == T_ARRAY;
  if (!is_oop) {
    return false;
  }

  if (tightly_coupled_alloc) {
    if (phase == Optimization) {
      return UseShenandoahMatrix;
    }
    return !is_clone;
  }
  if (phase == Optimization) {
    return !ShenandoahStoreValEnqueueBarrier;
  }
  return true;
}

// Support for macro expanded GC barriers
void ShenandoahBarrierSetC2::register_potential_barrier_node(Node* node) const {
  if (node->Opcode() == Op_ShenandoahWriteBarrier) {
    state()->add_shenandoah_barrier((ShenandoahWriteBarrierNode*) node);
  }
}

void ShenandoahBarrierSetC2::unregister_potential_barrier_node(Node* node) const {
  if (node->Opcode() == Op_ShenandoahWriteBarrier) {
    state()->remove_shenandoah_barrier((ShenandoahWriteBarrierNode*) node);
  }
}

void ShenandoahBarrierSetC2::eliminate_gc_barrier(PhaseMacroExpand* macro, Node* node) const {
  Compile::current()->shenandoah_eliminate_matrix_update(node, &macro->igvn());
}

void ShenandoahBarrierSetC2::enqueue_useful_gc_barrier(Unique_Node_List &worklist, Node* node) const {
}

void ShenandoahBarrierSetC2::eliminate_useless_gc_barriers(Unique_Node_List &useful) const {
  for (int i = state()->shenandoah_barriers_count()-1; i >= 0; i--) {
    ShenandoahWriteBarrierNode* n = state()->shenandoah_barrier(i);
    if (!useful.member(n)) {
      state()->remove_shenandoah_barrier(n);
    }
  }

}

void ShenandoahBarrierSetC2::add_users_to_worklist(Unique_Node_List* worklist) const {}

void* ShenandoahBarrierSetC2::create_barrier_state(Arena* comp_arena) const {
  return new(comp_arena) ShenandoahBarrierSetC2State(comp_arena);
}

ShenandoahBarrierSetC2State* ShenandoahBarrierSetC2::state() const {
  return reinterpret_cast<ShenandoahBarrierSetC2State*>(Compile::current()->barrier_set_state());
}

// If the BarrierSetC2 state has kept macro nodes in its compilation unit state to be
// expanded later, then now is the time to do so.
bool ShenandoahBarrierSetC2::expand_macro_nodes(PhaseMacroExpand* macro) const { return false; }
void ShenandoahBarrierSetC2::verify_gc_barriers(bool post_parse) const {
#ifdef ASSERT
  if (ShenandoahVerifyOptoBarriers && !post_parse) {
    ShenandoahBarrierNode::verify(Compile::current()->root());
  }
#endif
}

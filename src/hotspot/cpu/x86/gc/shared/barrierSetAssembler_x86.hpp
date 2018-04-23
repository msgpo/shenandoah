/*
 * Copyright (c) 2018, Oracle and/or its affiliates. All rights reserved.
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

#ifndef CPU_X86_GC_SHARED_BARRIERSETASSEMBLER_X86_HPP
#define CPU_X86_GC_SHARED_BARRIERSETASSEMBLER_X86_HPP

#include "asm/macroAssembler.hpp"
#include "memory/allocation.hpp"
#include "oops/access.hpp"

class InterpreterMacroAssembler;

class BarrierSetAssembler: public CHeapObj<mtGC> {
protected:
public:
  virtual void arraycopy_prologue(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
                                  Register src, Register dst, Register count) {}
  virtual void arraycopy_epilogue(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
                                  Register src, Register dst, Register count) {}

  virtual void load_at(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
                       Register dst, Address src, Register tmp1, Register tmp_thread);
  virtual void store_at(MacroAssembler* masm, DecoratorSet decorators, BasicType type,
                        Address dst, Register val, Register tmp1, Register tmp2);

  virtual void obj_equals(MacroAssembler* masm, DecoratorSet decorators, Register src1, Register src2);
  virtual void obj_equals_addr(MacroAssembler* masm, DecoratorSet decorators, Register src1, Address src2);

  virtual void resolve_for_read(MacroAssembler* masm, DecoratorSet decorators, Register obj);
  virtual void resolve_for_write(MacroAssembler* masm, DecoratorSet decorators, Register obj);

  virtual void barrier_stubs_init() {}
};

#endif // CPU_X86_GC_SHARED_BARRIERSETASSEMBLER_X86_HPP

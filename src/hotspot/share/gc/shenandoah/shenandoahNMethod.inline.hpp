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

#ifndef SHARE_GC_SHENANDOAH_SHENANDOAHNMETHOD_INLINE_HPP
#define SHARE_GC_SHENANDOAH_SHENANDOAHNMETHOD_INLINE_HPP

#include "gc/shenandoah/shenandoahNMethod.hpp"

template<bool CSET_FILTER>
void ShenandoahNMethodTable::parallel_blobs_do(CodeBlobClosure *f) {
  size_t stride = 256; // educated guess

  ShenandoahNMethod** const list = _array;

  size_t max = (size_t)_index;
  while (_claimed < max) {
    size_t cur = Atomic::add(stride, &_claimed) - stride;
    size_t start = cur;
    size_t end = MIN2(cur + stride, max);
    if (start >= max) break;

    for (size_t idx = start; idx < end; idx++) {
      ShenandoahNMethod* nmr = list[idx];
      assert(nmr != NULL, "Sanity");

      nmr->assert_alive_and_correct();

      if (CSET_FILTER && !nmr->has_cset_oops(_heap)) {
        continue;
      }

      f->do_code_blob(nmr->nm());
    }
  }
}

#endif // SHARE_GC_SHENANDOAH_SHENANDOAHNMETHOD_INLINE_HPP

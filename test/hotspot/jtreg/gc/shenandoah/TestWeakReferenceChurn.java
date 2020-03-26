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

/*
 * @test TestWeakReferenceChurn
 * @summary Check that -XX:+ShenandoahAggressiveReferenceDiscovery reclaims references
 *          even when accessed during marking
 * @key gc
 * @requires vm.gc.Shenandoah & !vm.graal.enabled
 *
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -Xmx1g -Xms1g
 *      -XX:+UseShenandoahGC -XX:ShenandoahGuaranteedGCInterval=1
 *      -XX:+ShenandoahAggressiveReferenceDiscovery
 *      TestWeakReferenceChurn
 */

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Random;

public class TestWeakReferenceChurn {

    public static final int NUM_REFS = 1000000;
    public static final int NUM_ITERS = 1000;
    public static class Payload {
        public int foo;
        public Payload() {
            foo = 0;
        }
    }

    private static ArrayList<WeakReference<Payload>> refs;
    private static Random random;

    public static void main(String[] args) {
        refs = new ArrayList<>(NUM_REFS);
        random = new Random();
        for (int i = 0; i < NUM_REFS; i++) {
            refs.add(new WeakReference(new Payload()));
        }
        int count = 0;
        for (int j = 0; j < NUM_ITERS; j++) {
            count = 0;
            for (int i = 0; i < NUM_REFS; i++) {
                if (accessRef(i)) {
                    count++;
                }
            }
            if (count == 0) {
                break;
            }
        }
        if (count != 0) {
            throw new RuntimeException("References not reclaimed");
        }
    }

    private static boolean accessRef(int idx) {
        Payload item = refs.get(idx).get();
        if (item != null) {
            item.foo = random.nextInt();
            return true;
        } else {
            return false;
        }
    }
}

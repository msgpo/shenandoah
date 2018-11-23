/*
 * Copyright (c) 2017, 2018, Red Hat, Inc. All rights reserved.
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
 * @test SieveObjects
 * @summary Acceptance tests: collector can deal with retained objects
 *
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=passive      -XX:+ShenandoahDegeneratedGC -XX:+ShenandoahVerify SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=passive      -XX:-ShenandoahDegeneratedGC -XX:+ShenandoahVerify SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=passive      -XX:+ShenandoahDegeneratedGC                       SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=passive      -XX:-ShenandoahDegeneratedGC                       SieveObjects
 *
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=aggressive   -XX:+ShenandoahOOMDuringEvacALot SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=aggressive   -XX:+ShenandoahAllocFailureALot  SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=aggressive                                    SieveObjects
 *
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=adaptive     -XX:+ShenandoahVerify SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=traversal    -XX:+ShenandoahVerify SieveObjects
 *
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=adaptive     SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=static       SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=compact      SieveObjects
 * @run main/othervm -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:ShenandoahGCHeuristics=traversal    SieveObjects
 *
 * @run main/othervm/timeout=240 -XX:+UnlockDiagnosticVMOptions -XX:+UnlockExperimentalVMOptions -XX:+UseShenandoahGC -Xmx1g -Xms1g -XX:-UseTLAB                -XX:+ShenandoahVerify SieveObjects
 */

import java.util.concurrent.ThreadLocalRandom;

public class SieveObjects {

  static final int COUNT = 100_000_000;
  static final int WINDOW = 1_000_000;
  static final int PAYLOAD = 100;

  static final MyObject[] arr = new MyObject[WINDOW];

  public static void main(String[] args) throws Exception {
    int rIdx = 0;
    for (int c = 0; c < COUNT; c++) {
      MyObject v = arr[rIdx];
      if (v != null) {
        if (v.x != rIdx) {
          throw new IllegalStateException("Illegal value at index " + rIdx + ": " + v.x);
        }
        if (ThreadLocalRandom.current().nextInt(1000) > 100) {
          arr[rIdx] = null;
        }
      } else {
        if (ThreadLocalRandom.current().nextInt(1000) > 500) {
          arr[rIdx] = new MyObject(rIdx);
        }
      }
      rIdx++;
      if (rIdx >= WINDOW) {
        rIdx = 0;
      }
    }
  }

  public static class MyObject {
    public int x;
    public byte[] payload;
    public MyObject(int x) {
      this.x = x;
      this.payload = new byte[PAYLOAD];
    }
  }

}

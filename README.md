# SPM-IK-FPGA

Hardware inverse kinematics for a **coaxial 3-RRR spherical parallel manipulator**,
implemented in VHDL-2008 on a Xilinx Artix-7 (Digilent Basys 3, XC7A35T).

Commanded platform orientation goes in over USB-UART; the three actuator angles
come back, together with live absolute-encoder readings and the tracking error.
The solve runs entirely in fabric in **114 clock cycles — 1.14 µs at 100 MHz —
with zero cycle-to-cycle variation**.

The IK formulation follows Tursynbek, Niyetkaliyev & Shintemirov (2019),
*Computation of Unique Kinematic Solutions of a Spherical Parallel Manipulator
with Coaxial Input Shafts*. The reference Python solver (`python/spm_kinematics.py`)
is the specification; the RTL is checked bit-for-bit against a fixed-point model of it.

---

## Results

| | |
|---|---|
| Solve latency | **114 clocks = 1.14 µs @ 100 MHz** |
| Latency jitter | **0 cycles** (p100 = p50 = 114) |
| Angular accuracy vs. float reference | 0.0001° mean, 0.0005° p99 |
| Wire resolution | 0.0055° (int16 BAM) |
| Dividers in the design | **0** |
| Target | XC7A35T-1CPG236C, 100 MHz |

**Verification, all bit-exact against the golden model:**

| Testbench | Vectors | Failures |
|---|---|---|
| `tb_math_units` — cordic_rot | 701 | 0 |
| `tb_math_units` — cordic_vec | 807 | 0 |
| `tb_math_units` — isqrt | 504 | 0 |
| `tb_spm_ik_core` | 615 | 0 |
| `tb_system` (through the real UART pins) | 23 round trips | 0 |

`tb_spm_ik_core` reports **max error 0 LSB** — not "close", identical.
`tb_system` drives `RsRx` with real 8N1 bit timing, real framing and real CRC-8,
and decodes the reply off `RsTx`. Nothing internal is probed.

Reproduce all of it with `make` (GHDL is the only dependency).

> The utilization / Fmax / power numbers are **not** filled in here yet — that
> requires a Vivado run, which has not been done. `make bitstream` produces them,
> and `fpga/tcl/build.tcl` errors out rather than shipping a bitstream with
> negative slack. Do not quote resource numbers until you have run it.

---

## The core idea

For each leg the problem reduces to solving

```
a·sin(ψ) + b·cos(ψ) = c
```

The reference Python does this the textbook way:

```python
phi   = atan2(b, a)
delta = asin(c / r)          # r = sqrt(a² + b²)
psi   = delta - phi          # branch 0
      = (pi - delta) - phi   # branch 1
```

That needs a **divider** and an **arcsine**, both awkward in fabric. This design
uses the identity

```
s    = sqrt(a² + b² − c²)
ζ    = atan2(c, +s)   ==  δ            (branch 0)
     = atan2(c, −s)   ==  π − δ        (branch 1)
ψ    = ζ − φ
```

`atan2(c, +s)` has positive cosine, so it lies in (−π/2, π/2) and its sine is
`c/r` — which *is* `asin(c/r)`. Same two assembly modes, exactly the same answer,
but the whole design contains **no divider anywhere**: two CORDIC vectoring
passes, one integer square root, and multipliers.

The reachability test falls out for free and is **exact**: the pose is
unreachable for a leg iff `a² + b² − c² < 0`, evaluated on the full 48-bit
product. Compare the float model's `abs(c) > r`, which needs an epsilon and can
disagree with itself at the boundary.

---

## Architecture

```
                   ┌──────────────┐
   USB-UART ──────▶│  uart_proto  │  framing, CRC-8, dispatch
   (FT2232)  ◀─────└──────┬───────┘
                          │ roll/pitch/yaw (BAM)
                   ┌──────▼────────────────────────────┐
                   │           spm_ik_core             │
                   │  ┌────────────┐                   │
                   │  │ rot_matrix │ 3∥ cordic_rot     │
                   │  │            │ + shared mult ×14 │
                   │  └─────┬──────┘                   │
                   │        │ R (3×3)                  │
                   │   ┌────┴────┬─────────┐           │
                   │   ▼         ▼         ▼           │
                   │ leg_0     leg_1     leg_2   ∥     │
                   │  (shared mult ×15,                │
                   │   isqrt ∥ cordic_vec,             │
                   │   then cordic_vec)                │
                   └───────────────┬───────────────────┘
                                   │ θ₁ θ₂ θ₃
   Pmod JA ───▶ enc_pwm ×3 ──▶ enc_bank ──▶ err = θ_cmd − θ_meas
```

**Legs run in parallel**, multipliers are time-multiplexed *within* each leg.
That split is a resource decision, not a stylistic one: instantiating all ~59
products in parallel overruns the 90 DSP48E1 slices on an XC7A35T, while the
three legs are structurally independent and worth the area to overlap.

Inside each leg, the **square root runs concurrently with the first CORDIC
vectoring pass** — they have no data dependency — which hides 24 of the sqrt's
26 cycles. Only the second CORDIC, which needs `s`, is serialised.

### Latency budget

| Stage | Clocks |
|---|---|
| 3 × cordic_rot (parallel) | 22 |
| R-matrix products (shared mult) | 16 |
| leg: v_b + a,b,c + squares | 16 |
| leg: isqrt ∥ cordic_vec(φ) | 26 |
| leg: cordic_vec(ζ) | 23 |
| wrap + handshake | 11 |
| **total** | **114** |

Constant by construction — **there is no data-dependent branch anywhere in the
schedule**. An unreachable pose takes exactly as long as a reachable one; it just
raises `unreach`. `spm_ik_core` carries a cycle counter that reports the measured
latency in every reply, and both `tb_spm_ik_core` and `tb_system` *assert* that it
never varies. If it ever does, something has acquired an input dependency and the
real-time guarantee is gone.

---

## Number formats

| Where | Format | Why |
|---|---|---|
| Internal datapath | **Q3.21**, 24-bit signed | π fits with headroom; one operand still lands in a DSP48E1's 25×18 |
| CORDIC internal | **Q3.25**, 28-bit (4 guard bits) | without guard bits the shift truncation, not the arctan table, sets accuracy |
| Wire + encoders | **int16 BAM**, ±32768 ↔ ±π | see below |

**Why BAM on the wire.** Two reasons, and the first is a trap I walked into:

1. Q3.15 does *not* fit in 16 bits. Sign + 3 integer + 15 fractional = 19.
2. BAM is range-closed under wraparound — 16-bit rollover *is* a mechanical
   revolution. So `θ_cmd − θ_enc` as a **plain two's-complement subtraction** is
   the wrapped shortest-path angular error, correct across the ±π seam, with no
   modulo, no branch, no special case. The encoder zero-offset is the same
   subtraction. Biasing a duty-cycle reading into BAM is a single MSB flip
   instead of a 16-bit subtractor.

`tb_system` explicitly checks the seam case.

**Rounding is truncation toward −∞** everywhere (plain arithmetic right shift).
Free in hardware, and Python's `>>` on negative integers does the same thing —
which is what lets the golden model be bit-exact rather than merely close.

---

## Single source of truth

Every constant in the RTL — the CORDIC arctan table, `1/K`, the π constants, the
BAM scale factors, and the mechanism geometry (α₁, α₂, β, η) — is **generated**
from `python/golden_fixed.py` into `rtl/pkg/spm_const_pkg.vhd` by
`python/gen_pkg.py`. Hand-editing the generated file is a bug. Changing the
mechanism geometry means editing one Python line and running `make pkg`.

---

## Debug log

Every one of these was found by simulating rather than reasoning. They are the
most useful part of this repository.

**1. `atan2` argument order.** `cordic_vec(x,y)` computes `atan2(y,x)`, so
`atan2(c,s)` is `cordic_vec(s,c)`, not `cordic_vec(c,s)`. Produced a clean π
error on a subset of poses.

**2. Negate-then-shift vs. shift-then-negate.** Arithmetic right shift truncates
toward −∞, so `−(x >> 4) ≠ (−x) >> 4` for negative `x`. The CORDIC quadrant
fold-back had them the wrong way round. Cost: exactly 1 LSB — which became
~0.05 rad (2.7°) after propagating through two atan2 passes. A one-LSB bug is not
a small bug in a feedback datapath.

**3. CORDIC accumulator overflow.** Input normalisation targeted bit `W−3`.
Vectoring grows the vector by the CORDIC gain `K = 1.6468`, so the worst case
reaches `K·√2·2^(W−2)` = 1.56e8 against a `2^(W−1)` = 1.34e8 limit — **16%
overflow**, silently wrapping. It only appeared for large inputs and produced a
handful of *repeated* plausible-looking wrong angles, which is what gave it away:
real arithmetic errors are scattered, wrapped ones cluster. Fix: normalise to
`W−4`, 1.7× margin. The headroom is now derived in a comment, not guessed.

**4. `atan2(0, 0)`.** With `x = y = 0` the sign test always takes the same branch
and `z` accumulates the *entire* arctan table — 1.7433 rad of confident garbage.
Callers guard the case, but a math block should not emit garbage for a legal
input.

**5. `req_v` pulsed in the same cycle as its payload.** The consuming FSM sampled
the *previous* frame's command and arguments. Classic valid/data timing error;
only surfaced on the 24th consecutive frame. Fixed by pulsing valid one cycle
after the payload registers are written.

**6. Receive was a state of the transmit FSM.** Any byte arriving while a reply
was still draining out of the transmitter was silently discarded — and the
dropped byte is the SOF, so the whole frame desyncs and the host waits forever
for a reply that never comes. Receive now runs in its own always-active process.

**Bonus, and arguably the most useful:** the original `tb_system` did
`wait until tx = '0'` to find a start bit. `wait until` waits for an *event*, so
when the transmitter began a byte in the same delta the previous stop bit ended,
the line was already low, the edge never came, and **the testbench hung instead
of failing**. A testbench that hangs converts "the reply never arrived" into "the
run never finished", which is strictly harder to diagnose. All waits are bounded
now and report on timeout.

---

## Protocol

**Request**, 9 bytes: `A5 | CMD | arg0(i16 LE) | arg1 | arg2 | CRC8`

| CMD | | |
|---|---|---|
| `0x01` | SOLVE | args = roll, pitch, yaw (BAM) |
| `0x02` | CONFIG | arg0[2:0] = assembly-mode branch per leg |
| `0x03` | ZERO | encoder home offsets |
| `0x04` | INJECT | virtual encoder values |
| `0x05` | PING | reply without solving |

**Response**, 23 bytes: `5A | STATUS | θ₁₋₃ | enc₁₋₃ | err₁₋₃ | cycles(u16) | CRC8`

`STATUS`: bits 2:0 per-leg unreachable, 5:3 per-leg encoder fault, 6 CRC error on
last request, 7 solve valid.

CRC-8/ATM (poly `0x07`). A corrupted request is flagged and **not acted on** —
`tb_system` verifies the commanded angle does not move.

---

## Encoders — read before wiring

⚠️ **Confirm your encoder's actual output mode.** The Lamprey2 is commonly used
in FRC with an **analog** absolute output. If that is the part you have, you need
`rtl/enc/enc_xadc.vhd`, **not** `enc_pwm.vhd`. Check the datasheet — do not take
this README's word for it, and do not take the RTL's.

Both front-ends drive an identical interface into `enc_bank.vhd`, so swapping
costs nothing above that line. There is also a **UART-injected virtual encoder**
(`sw[15] = 1`), which lets the entire command path, protocol, error arithmetic
and host tooling be brought up and regression-tested before a single encoder is
wired.

| File | Status |
|---|---|
| `enc_pwm.vhd` | duty-cycle absolute encoders. Fully simulatable. |
| `enc_xadc.vhd` | analog via XADC. **Scaling + fault logic written; the XADC primitive is NOT simulated and has NOT been run on hardware.** Use the Vivado XADC Wizard for the DRP side. |
| `enc_bank.vhd` | source mux, zero offset, fault aggregation. |

⚠️ **Basys 3 Pmod pins are 3.3V LVCMOS and are NOT 5V tolerant.** A 5V encoder
output wired straight to a Pmod pin will damage the FPGA. XADC auxiliary inputs
want roughly 0–1V, not 0–3.3V. Size any divider from the board schematic for
your revision.

Both front-ends assert `fault` on timeout or out-of-window period. This matters:
a disconnected encoder otherwise reads as a plausible constant angle, which a
control loop will happily and confidently act on.

---

## Board controls

| Control | |
|---|---|
| `btnC` | reset |
| `btnU` | local solve trigger — proves the datapath runs with no host attached |
| `sw[15]` | encoder source: 0 = PWM on JA, 1 = injected over UART |
| `sw[3]` | show raw BAM in hex instead of degrees |
| `sw[2:0]` | display select: which leg, θ vs. encoder |
| `led[2:0]` | per-leg unreachable |
| `led[5:3]` | per-leg encoder fault |
| `led[6]` | CRC error |
| `led[7]` | IK busy |
| `led[8]` | solve completed (stretched) |
| `led[9:10]` | UART rx / tx activity |
| `led[15]` | heartbeat — if this stops, the clock or reset is wrong |

---

## Layout

```
rtl/pkg/spm_const_pkg.vhd   GENERATED — do not hand-edit
rtl/pkg/spm_pkg.vhd         fixed-point types + arithmetic
rtl/math/cordic_rot.vhd     angle → sin/cos
rtl/math/cordic_vec.vhd     atan2, with input normalisation
rtl/math/isqrt.vhd          48-bit Q6.42 → 24-bit Q3.21
rtl/ik/rot_matrix.vhd       RPY → R
rtl/ik/leg_solver.vhd       one leg
rtl/ik/spm_ik_core.vhd      3 legs + latency counter
rtl/io/{uart_rx,uart_tx,crc8,uart_proto}.vhd
rtl/enc/{enc_pwm,enc_xadc,enc_bank}.vhd
rtl/top/{seven_seg,spm_ik_top}.vhd
constraints/basys3.xdc
fpga/tcl/build.tcl          non-project Vivado build
sim/tb/{tb_math_units,tb_spm_ik_core,tb_system}.vhd
python/spm_kinematics.py    the specification (float reference)
python/golden_fixed.py      bit-exact model of the RTL
python/gen_pkg.py           model → VHDL constants
python/gen_*_vectors.py     model → simulation stimulus
python/host.py              PC-side driver
```

`fpga/tcl/build.tcl` is non-project mode on purpose. A checked-in `.xpr` is a
binary blob encoding absolute paths and a Vivado version; it does not diff, does
not merge and does not reproduce on anyone else's machine.

---

## Honest limits

- **No Vivado run has been done.** No utilization, Fmax, power or on-hardware
  numbers. Everything above is simulation.
- `enc_xadc.vhd`'s XADC primitive interface is unverified.
- The worst observed fixed-point error (0.033°) occurs at `|c|/r ≈ 1.0004` — the
  workspace boundary, where `d(asin)/dx = 1/√(1−x²)` diverges. That is
  conditioning of the mechanism, not arithmetic. Boundary detection differs from
  the float model by ~1 LSB there, which is unavoidable and physically meaningless.
- No forward kinematics, no trajectory generation, no servo loop closure. The
  tracking error is computed and reported; nothing acts on it yet.
- Single solve at a time; no pipelining. Throughput is 1/114 clocks. That is a
  deliberate choice for a latency-bound control problem, not a limitation that
  was overlooked.

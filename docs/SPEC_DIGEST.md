# HyperBus Specification Digest — Single Source of Truth

**Normative source:** Infineon/Cypress **HyperBus Specification, doc 001-99253 Rev. \*H** (Feb 6, 2019).
Spec text file: `hyperbus_spec.txt`. Page citations are the spec's internal "Page N of 44" numbering; a few
line-range citations point into the plain-text extract. This digest is implementation-oriented: it is the
one document a controller implementer follows.

**Genericity boundary (read first).** 001-99253 is the *generic bus* spec. It normatively defines the
**protocol, signal roles, 48-bit CA encoding, ID0/ID1 low-nibble structure, and AC timing**. It treats the
**configuration-register *contents* as an *Example* (Table 5.3, §5.2)** and repeatedly defers register
*addresses, latency-code-to-frequency mapping, wrap-option support, reset defaults, and all of CR1* to each
device datasheet. Concrete addresses / reset values below are **W957D8NB / HyperRAM-family** values,
cross-checked against this project's device model `sim/hyperbus/w957d8nb_bfm.sv`, and are flagged
**[device, not generic spec]**. Everything unflagged cites 001-99253 directly.

---

## 1. Signal List and Roles
Source: **§2 Tables 2.1 (mandatory) / 2.2 (optional), p.7**; interface states **Table 7.1, p.25**; clocking **§1 p.3–4**.

| Signal | Direction | Role |
|---|---|---|
| **CS#** | Master → Slave (one per slave) | Chip Select, active Low. Transaction **starts on High→Low**, **ends on Low→High**. Idle (High) between transactions. Each slave has its own CS#. |
| **CK / CK#** | Master → Slave | Clock. **1.8 V I/O ⇒ differential CK/CK#**; **3.0 V I/O ⇒ single-ended CK only** (CK# unused). CA/data captured on CK edges / CK–CK# crossings. **Not required free-running**; may idle or stop mid-transaction (Active Clock Stop, optional). "Idle" = CK Low, CK# High. |
| **DQ[7:0]** | Bidirectional | 8-bit bus carrying Command, Address, and Data. **DDR: one byte per clock edge, two bytes per clock cycle.** Bit order fixed: CA/data bit 7 → DQ7 … bit 0 → DQ0 (Fig 3.2 note 5, p.11). |
| **RWDS** | Bidirectional | Read/Write Data Strobe — **four roles** (see §3, §5): (a) **during CA** = slave-driven latency indicator (High ⇒ 2× latency count, Low ⇒ 1×); (b) **during read data** = slave source-synchronous **read strobe**, edge-aligned to data; (c) **during latency-bearing write data** = master-driven **byte mask** (High ⇒ mask/skip, Low ⇒ write); (d) mid-burst the slave holds it Low to insert row/page-crossing latency. |
| **RST# (RESET#)** | Master → Slave, internal weak pull-up | Hardware reset. Low ⇒ slave self-inits to Standby, restores config-register defaults, exits Deep-Power-Down; **RWDS and DQ[7:0] go High-Z**. Floats High if unconnected. |
| *RSTO#, INT#* | Slave out, open-drain (optional) | POR/reset indicator; interrupt. Not part of the core transaction datapath. |

---

## 2. The 48-bit Command-Address (CA) Word
Source: **§3 p.8; §3.1 Fig 3.1 p.9; Table 3.1 p.9 (byte→edge); Table 3.2 p.10–11 (fields).**

Every transaction: **CS# Low with CK idle** → first **three clock cycles** transfer three 16-bit CA words
**CA0, CA1, CA2** = 48 bits = 6 bytes, DDR over the first six clock edges, **center-aligned** with CK.

**Byte → edge mapping (Table 3.1), MSB byte first:**
```
edge:  1          2          3          4          5          6
DQ:  CA[47:40]  CA[39:32]  CA[31:24]  CA[23:16]  CA[15:8]   CA[7:0]
     \___CA0 (word)___/     \___CA1 (word)___/   \___CA2 (word)___/
```
Within each byte, CA bit maps straight to DQ index (CA[47]→DQ7 … CA[40]→DQ0).

**Exact CA bit map (Table 3.2, p.10; spec lines 511–518):**

| CA bit(s) | Field | Encoding |
|---|---|---|
| **47** | **R/W#** | `1` = **Read**, `0` = **Write** |
| **46** | **Address Space (AS)** | `0` = **memory space**, `1` = **register space** (ID + config regs) |
| **45** | **Burst Type** | `0` = **wrapped**, `1` = **linear** |
| **44:16** | **Row & Upper Column Address** | System **word** address bits **A31–A3**. Unused upper Row bits = 0 (host). Row/Column split is device-dependent. |
| **15:3** | **Reserved** | Future column-address expansion. Don't-care today; **host must write 0** for forward-compat. |
| **2:0** | **Lower Column Address** | System **word** address bits **A2–A0** — starting word within a half-page. |

**Addressing notes (§3.1 notes p.10):** address is a **word** (16-bit) address. **Page = 16 words / 32 B**;
**half-page = 8 words / 16 B**. Upper column selects the half-page, lower column CA[2:0] selects the word
within it. Protocol max = 29 (Row+UpperCol) + 3 (LowerCol) = 32 word-address bits = 4G words = 8 GByte / 64 Gbit.
Read array access can begin **as soon as CA1 is captured** — specifically once CA[23:16] is on the bus
(Fig 3.4 note 4, p.13); CA2 then supplies the exact word.

`pack_ca()` in this project's `hyperbus_pkg.sv` builds this exactly: CA[47]=R/W#, [46]=AS, [45]=1 (linear),
[44:16]=addr[31:3], [15:3]=0, [2:0]=addr[2:0].

---

## 3. Initial Latency — Fixed vs Variable, RWDS-during-CA Selection, Latency Table
Source: **§3.2/§3.3 p.12–14; §5.2.3 Initial Latency + Table 5.3 p.19; §5.2.4 Fixed Latency p.20; spec lines 428–484.**

**Mechanism.** After CA the master keeps clocking a **latency count** of cycles before data moves.
tACC (initial access) is measured from **CA1 capture** to start of data. During the CA period the **slave
drives RWDS** to select single vs double latency:
- **RWDS Low during CA ⇒ 1× latency count** (no extra latency).
- **RWDS High during CA ⇒ 2× latency count** (one additional count inserted).

This lets the slave insert extra latency when an internal operation (HyperRAM refresh / row-boundary
crossing) is in progress at transaction start.

**Fixed vs Variable — CR0[3] (§5.2.4):**
- **CR0[3] = 1 → Fixed Initial Latency (POR default).** Slave drives RWDS to a **constant** value during CA
  every transaction ⇒ deterministic latency regardless of internal state. Some devices fix at 1×, some at 2×.
- **CR0[3] = 0 → Variable Initial Latency.** Slave drives RWDS High **only when extra latency is actually
  required**; otherwise Low (1×).

A **zero-latency Write** (§5) ignores RWDS-during-CA for latency purposes.

**Latency-count code — CR0[7:4] (Table 5.3, p.19).** Latency = clock cycles from CA1 capture to first data:

| Code | Clocks | Code | Clocks |
|---|---|---|---|
| 0000 | 5  | 1000 | 13 |
| 0001 | 6  | 1001 | 14 |
| 0010 | 7  | 1010 | 15 |
| 0011 | 8  | 1011 | 16 |
| 0100 | 9  | 1100 | Reserved |
| 0101 | 10 | 1101 | Reserved |
| 0110 | 11 | 1110 | **3** |
| 0111 | 12 | 1111 | **4** |

Range **3–16 clocks**; §5.2.3 states the value is frequency-dependent and the POR default is device-dependent
(picked to allow max-frequency operation until the host lowers it). **The two low codes (3, 4) sit at the top
of the field, out of numeric order** — a common off-by-one trap. Frequency-table reference values used by
reference cores (OpenHBMC `INITIAL_LATENCY`): 3/4/5/6/7 clocks for ≤83/100/133/166/200 MHz **[device guidance,
confirm against W957D8NB datasheet]**.

---

## 4. RWDS Dual Role and the DDR Data Phase
Source: **§1 p.4; §3 p.8; §3.2 p.12–13; §3.3 p.14; Table 3.3 p.11–12; Figs 3.2/3.3 p.11–12; spec lines 527–530.**

**DDR, 16-bit words.** Every transfer is a full 16-bit word: **byte A first, byte B second**.
- **Byte A = word[15:8]** on the **High-going CK edge / CK–CK# crossing** (write) or **following an RWDS rising
  edge** (read).
- **Byte B = word[7:0]** on the **Low-going CK edge** (write) or **following the RWDS falling edge** (read).

**Read data phase.** Slave drives DQ **and** RWDS simultaneously; **data is edge-aligned to RWDS transitions**
— RWDS is a source-synchronous read strobe, new data on every transition. The slave may **pause RWDS Low
between words** to insert inter-word latency at memory-array (row/page) boundaries; holding RWDS Low
**≥ 32 clocks** signals an error requiring the master to abort the read (device-dependent). (§3.2 p.13,
spec 527–530.)

**Write data phase (with latency).** **Data is center-aligned to CK** (byte A on CK rising, byte B on CK
falling; slave captures). RWDS is now **master-driven as a byte mask**: **RWDS High ⇒ byte masked (array
unchanged); RWDS Low ⇒ byte written.** Enables byte-aligned / unaligned merged writes within a burst.
Because the master owns RWDS during write data, **neither side can insert flow-control latency mid-write** —
the slave must accept a continuous burst (or the master respects the device max burst length / tCSM). (§3.3 p.14.)

**Endianness (Table 3.3).** *Memory space* may be little- or big-endian (order fixed at write time).
*Register space is **always big-endian*** : byte A = Word[15:8], byte B = Word[7:0].

**RWDS/DQ ownership per interface state (Table 7.1, p.25) — the turnaround handoffs:**

| Interface state | DQ[7:0] | RWDS |
|---|---|---|
| Command-Address | Master-Output-Valid | X (slave driving latency indicator) |
| Read Initial Access Latency | **High-Z (bus turn-around)** | L (slave) |
| Write Initial Access Latency | High-Z | **High-Z (RWDS turn-around)** — slave releases, master takes over; master must drive RWDS Low **before end of latency** as the data-mask preamble (Fig 3.6 note 5, p.15) |
| Read data | Slave-Output-Valid | Slave-Output-Valid |
| Write data (with latency) | Master-Output-Valid | Master-Output-Valid |

---

## 5. Register-Access Sequence
Source: **§3.1, §4, §5, §3.4 p.16; Table 3.2; Table 3.3 sheet 2 p.12.**

Every transaction: **CS# Low (CK idle: CK=Low, CK#=High)** → 6 CA bytes over 3 clocks (DDR, 2 B/clock) →
optional initial latency → data → **CS# High (with CK idle)**.

**Reading CR0 / CR1 / ID0 / ID1** — issue CA with **R/W#=1, AS=1**, register **word** address in CA[44:16]/[2:0].
Value returns after the normal initial-latency count, **big-endian** (byte A=[15:8] on RWDS-rising strobe,
byte B=[7:0] on RWDS-falling). Register space **aliases** throughout the address range (unused upper bits are
don't-care for HyperRAM, §5.1 p.17).

**Writing CR0 / CR1** — issue CA with **R/W#=0, AS=1**, register word address, then the 16-bit value:
byte A=[15:8] on CK rising, byte B=[7:0] on CK falling, center-aligned. **Register-space writes typically use
zero initial latency** (see §6): data immediately follows CA, master does **not** drive RWDS and there is **no
byte mask** (full-word writes only); the slave's RWDS during CA is ignored for latency.

Because AS (CA[46]) is known only after CA0 is captured, the slave drives RWDS during CA **before** it knows
read/write or memory/register — hence RWDS is always driven in CA even when it will be ignored (§3 p.8; §5.2.3 p.20).

---

## 6. Zero-Latency Writes, Turnaround (tRWR), and CS# Timing
Source: **§3.2/§3.3 p.12–14; §3.4 p.16; §5.2.3 p.20; §9.3 Table 9.4 p.36; Table 7.1 note 2 p.25.**

**Writes without initial latency (§3.4).** No RWDS turn-around. Slave still drives RWDS during CA (it does not
yet know R/W#/space) but its state **does not affect** the zero latency. Write data immediately follows CA.
The slave may keep RWDS Low or go High-Z; **the master must NOT drive RWDS**, RWDS is **not** a mask, and
**all bytes are written (full-word writes only).** Generally used for register-space writes; whether required
per space (memory/register) is device-dependent, so a compliant master must be **configurable for zero-latency
writes per space**.

**Read-Write Recovery, tRWR (§3.2; Table 9.4).** Minimum time from end of the prior transaction (prior CS#
rising edge) to the point **CA1 is captured** on the next. The master must delay CS# Low so CA1 completes only
after tRWR is satisfied.

**CS# High between transactions, tCSHI (Table 9.4; §7).** CS# must be High ≥ tCSHI between transactions and
before the first access after power-up (§8.5).

**tCSM — HyperRAM CS# Maximum Low time (Table 9.4).** Upper bound on how long CS# may stay Low in one
transaction (guarantees distributed self-refresh can run): **4.0 µs (Industrial) / 1.00 µs (Industrial-Plus)**,
constant across 200/166/133/100 MHz. Master must terminate/re-issue before tCSM expires. (Reference cores derive
`MAX_BURST_COUNT = tCSM / tCK` and chop long bursts accordingly.)

See §9 for the full timing table (tCSS, tCSH, tDSZ, tOZ, etc.).

---

## 7. Burst Modes
Source: **§1 p.5; §3.1; §5.2.5–§5.2.6 p.20; Table 5.4 p.21.**

Selected **per transaction** by **CA[45]**: `0` = wrapped, `1` = linear. Wrapped/linear may be freely
intermixed transaction-to-transaction. Whether a wrap is *legacy* or *hybrid* is set globally by **CR0[2]**.

- **Linear (CA[45]=1):** start at CA address, increment every clock sequentially across page/row boundaries
  until CS# High. Reading past the last array address ⇒ undefined data (§3.2 p.13).
- **Legacy wrapped (CA[45]=0, CR0[2]=1):** start at CA word, run to the aligned top of the configured
  16/32/64/128 B group, wrap to the group base, continue back to the start, and keep wrapping within the group
  indefinitely.
- **Hybrid wrapped (CA[45]=0, CR0[2]=0):** wrap the group **once**, then switch to **linear** beginning at the
  start of the **next half-page** beyond the wrap group (critical-word-first line fill, then prefetch).

**Wrap boundary from CR0[1:0]:** `00`=128 B, `01`=64 B, `10`=16 B, `11`=32 B (default). In **words** the
boundary = bytes/2 = 64/32/8/16 words. The address **wraps by clearing the low log2(words) bits** (group base)
after hitting the aligned top.

**Wrap examples (Table 5.4, p.21), word addresses:**

| Mode | Group | Start | Sequence |
|---|---|---|---|
| Wrap-16 | 8 words | 02 | `02 03 04 05 06 07 00 01` → repeat (legacy) / then linear `08 09 0A…` (hybrid) |
| Wrap-32 | 16 words | 0A | `0A 0B 0C 0D 0E 0F 00 01 02 03 04 05 06 07 08 09 …` |
| Wrap-64 | 32 words | 03 | `03 … 1F 00 01 02` → continue |
| Wrap-128 | 64 words | 03 | `03 … 3F 00 01 02` → continue; hybrid then linear `40 41 42 …` |

**Mid-burst latency:** the slave may insert extra latency when crossing Row (device-dependent) or Page
boundaries by holding RWDS Low; ≥ 32 clocks Low signals an error requiring abort (§3.2 p.13). Robust
controllers **count returned words gated on the RWDS strobe** rather than free-running a latency clock, so
these gaps are handled transparently. This project's BFM models this via `ROW_BYTES`/`ROW_PENALTY`.

---

## 8. CR0 / CR1 / ID Register Maps

### 8.1 CR0 — Configuration Register 0 (Table 5.3, §5.2, p.18–19)
16-bit volatile register; loads defaults on power-up / hardware reset; writable in standby. Register words are
big-endian (byte A = [15:8]).

| Bit(s) | Field | Encoding | Reset / default |
|---|---|---|---|
| **15** | Deep Power-Down Enable | `0` = enter Deep-Power-Down immediately; `1` = normal | `1` (normal) |
| **14:12** | Drive Strength | `000`–`111` device-dependent impedance; `000` = midpoint | `000` |
| **11:8** | Reserved | write as `1` for future compatibility | `1` (all) |
| **7:4** | Initial Latency Count | see §3 latency-code table | device-dependent |
| **3** | Fixed Latency Enable | `0` = Variable Initial Latency; `1` = Fixed Initial Latency | `1` (fixed) |
| **2** | Hybrid Burst Enable | `0` = wrapped bursts use **hybrid**; `1` = **legacy** wrap | `1` (legacy) |
| **1:0** | Burst Length (wrap boundary) | `00`=128 B, `01`=64 B, `10`=16 B, `11`=32 B | `11` (32 B) |

**Polarity/order traps:** bit 2 is inverted (`1` = legacy/hybrid-*disabled*); burst-length codes are
non-monotonic (`10` = 16 B sits between 64 B and 32 B); latency codes `1110`/`1111` = 3/4 clocks at the top.

**Per-field behavior (§5.2.1–5.2.6):**
- **DPD (CR0[15], §5.2.1):** write `0` → Deep-Power-Down immediately (lowest current). Exit via write setting
  `1`, POR, or hardware reset; return to standby takes **tDPDOUT** (device-dependent). Support optional.
- **Drive Strength (CR0[14:12], §5.2.2):** DQ[7:0] output impedance; per-code values device-dependent.
- **Initial Latency (CR0[7:4], §5.2.3):** applies to memory read/write and register **read**. RWDS-High during
  CA inserts the second latency count.
- **Fixed Latency (CR0[3], §5.2.4):** `1` (default) = deterministic; `0` = variable. Optional/device-dependent.
- **Wrapped Burst (CR0[1:0], §5.2.5)** and **Hybrid Burst (CR0[2], §5.2.6):** see §7.

### 8.2 CR1 — **[device, not generic spec]**
001-99253 does **not** define CR1; §5.2.7 (p.21) states "Any additional identification or configuration
registers are device dependent." For the W957D8NB, CR1 exists at register-space word address `0x0000_0801`,
reset `0x0000` (per `w957d8nb_bfm.sv`). In the Winbond/Cypress HyperRAM family CR1 typically carries
distributed-refresh interval / partial-array-refresh / hybrid-sleep / master-clock-type fields — **confirm the
exact bit layout against the W957D8NB datasheet.** Do not hard-code CR1 fields from the generic spec (it has none).

### 8.3 ID Registers (§5.1, p.17–18)
Read-only, in Register Space (CA[46]=1), starting at word address 0. Implemented register space **aliases**
throughout because unused upper address bits are don't-care.

**ID0 — Word Address 0 (Table 5.1, p.17):**

| Bits | Function | Values |
|---|---|---|
| 15:4 | Device-dependent | per datasheet |
| 3:0 | **Manufacturer** | `0000`=Reserved, **`0001`=Cypress**, `0010`–`1111`=Reserved |

**ID1 — Word Address 1 (Table 5.2, p.18):**

| Bits | Function | Values |
|---|---|---|
| 15:4 | Device-dependent | per datasheet |
| 3:0 | **Device Type** | **`0000`=HyperRAM**, `0001`–`1101`=Reserved, **`1110`=HyperFlash**, `1111`=Reserved |

HyperFlash reads ID via a legacy Autoselect (ASO) sequence and *ignores* CA[46]; HyperRAM and other
peripherals **must** implement Register Space and expose ID at word 0 unconditionally.

### 8.4 Register addresses & reset values **[device / project BFM]**
Concrete values used by this project's HyperRAM model (`sim/hyperbus/w957d8nb_bfm.sv`; **not** normative in
001-99253):

| Register | Word addr | Reset value (BFM) |
|---|---|---|
| ID0 | `0x0000_0000` | `0x0C81` (mfr nibble `0001` = Cypress/Winbond-compatible) |
| ID1 | `0x0000_0001` | `0x0000` (device-type `0000` = HyperRAM) |
| CR0 | `0x0000_0800` | `0x0008` (bit 3 = 1 → fixed latency) |
| CR1 | `0x0000_0801` | `0x0000` |

---

## 9. Reset / Power-Up
Source: **§8.5–§8.7 p.29–31; Table 8.3 p.30; Table 2.2 p.7.**

**Power-On Reset (§8.5).** VCC and VCCQ ramp **simultaneously** with VCCQ ≤ VCC. On reaching VCC(min) the
device runs internal POR for **tVCS**, during which CS# must be High and transactions are prohibited. HyperRAM
is CS#-sensitive during init: **CS# must be High through tVCS**, and High for **tCSHI** before the first access.
Recommendation: pull CS# and RESET# up to VCCQ so both are High during tVCS.

**Hardware Reset (§8.7).** RESET# Low returns the device to standby, **restores all config registers to
defaults**, forces exit from Deep-Power-Down, and drives **RWDS + DQ[7:0] to High-Z**. Bus transactions
disallowed while RESET# Low. Internal weak pull-up (floats High if unconnected).

**Power-Down (§8.6).** Below VLKO, HyperRAM loses config/array data; for clean re-init VCC must drop below VRST
for ≥ **tPD**; above VLKO the part stays initialized. Values device-dependent.

**Power-On / Hardware-Reset AC parameters (Table 8.3, p.30):**

| Parameter | Description | HyperRAM | HyperFlash | Unit |
|---|---|---|---|---|
| tVCS | VCC/VCCQ ≥ min → first access | ≤ 150 | ≤ 300 | µs |
| tRPH | RESET# Low → CS# Low | ≥ 400 ns | ≥ 30 µs | — |
| tRP  | RESET# pulse width | ≥ 200 | ≥ 200 | ns |
| tRH  | RESET# High → CS# Low | ≥ 200 | ≥ 150 | ns |
| tCSHI | CS# High between operations | ≥ 10 | ≥ 10 | ns |
| VCC (1.8 V I/O) | supply | 1.70–2.00 | — | V |
| VCC (3.0 V I/O) | supply | 2.70–3.60 | — | V |

Controllers should implement a **POR init delay (~150–200 µs)** then a **CR0/CR1 write sequence** before
serving traffic (both reference cores do this: MJoergen `C_INIT_DELAY`, OpenHBMC `MEM_POWER_UP_DELAY`).

---

## 10. AC Timing Parameter Table
Source: **§9.3 Tables 9.2/9.3/9.4/9.5 p.33–39.** Clock rates **200 / 166 / 133 / 100 MHz** (50 MHz only for
HyperFlash word-program writes). Jitter ±5%. All I/O DDR (two transfers/CK), referenced to VCCQ/2 or CK–CK#
crossing.

**Clock (Table 9.2, p.33):**

| Symbol | Parameter | 200 MHz | 166 | 133 | 100 | Unit |
|---|---|---|---|---|---|---|
| tCK | Clock period (min) | 5.00 | 6.00 | 7.50 | 10.00 | ns |
| tCKHP | Half-period / duty (0.45–0.55 tCK) | 2.30–2.70 | 2.70–3.30 | 3.38–4.13 | 4.50–5.50 | ns |

**Clock DC levels (Table 9.3, p.33):** VID(DC) ≥ VCCQ×0.4; VIX = VCCQ×0.4…0.6.

**Read timing — common HyperFlash/HyperRAM (Table 9.4, p.36):**

| Symbol | Parameter | 200 MHz | 166 | 133 | 100 MHz | Unit |
|---|---|---|---|---|---|---|
| tCSHI | CS# High between transactions (min) | 5.00 | 6.00 | 7.50 | 10.00 | ns |
| tRWR | HyperRAM Read-Write Recovery (min) | 35.00 | 36.00 | 37.50 | 40.00 | ns |
| tCSS | CS# setup to next CK rising (min) | 2.00 | 3.00 | 3.00 | 3.00 | ns |
| tDSV | Data Strobe Valid (max) | 5.00 | — | — | 12.00 | ns |
| tIS | Input setup (min) | 0.40 | — | — | 1.00 | ns |
| tIH | Input hold (min) | 0.40 | — | — | 1.00 | ns |
| tACC | Read initial access, HyperRAM (max) | 35.00 | — | — | 40.00 | ns |
| tACC | Read initial access, HyperFlash (max) | 80.00 | — | — | 96.00 | ns |
| tDQLZ | Clock → DQ Low-Z (min) | 0 | — | — | 0 | ns |
| tCKD | CK transition → DQ valid (min/max) | 1.20 / 5.00 | — | — | 1.00 / 5.50 | ns |
| tCKDI | CK transition → DQ invalid (min/max) | 0.40 / 4.20 | — | — | 0.40 / 4.30 | ns |
| tDV | Data Valid window (min) | 1.45 | — | — | 3.30 | ns |
| tCKDS | CK transition → RWDS valid (min/max) | 1.20 / 5.00 | — | — | 1.00 / 5.50 | ns |
| tDSS | RWDS→DQ valid skew (max) | +0.40 | — | — | +0.80 | ns |
| tDSH | RWDS→DQ invalid skew (max) | +0.40 | — | — | +0.80 | ns |
| tCSH | CS# hold after CK falling (min) | 0 | — | — | 0 | ns |
| tDSZ | CS# inactive → RWDS High-Z (max) | 5.00 | 6.00 | 6.00 | 6.00 | ns |
| tOZ | CS# inactive → DQ High-Z (max) | 5.00 | 6.00 | 6.00 | 6.00 | ns |
| tCSM | HyperRAM CS# max Low, Industrial (max) | 4.00 | 4.00 | 4.00 | 4.00 | µs |
| tCSM | HyperRAM CS# max Low, Ind.-Plus (max) | 1.00 | 1.00 | 1.00 | 1.00 | µs |

`tDV(min) = min( tCKHP_min − tCKD_max + tCKDI_max , tCKHP_min − tCKD_min + tCKDI_min )`.
**tCSH note (Table 9.4 note 1):** despite tCSH(min)=0, the master must hold CS# Low **one or more extra clock
periods** so the last word stays valid long enough to cover tCKD/tCKDS and RWDS phase-shifting for capture.

**Write timing — common (Table 9.5, p.39):** same tCSHI, tRWR, tCSS, tDSV, tIS, tIH, tCSH, tDSZ, tCSM as read, plus:

| Symbol | Parameter | Value | Unit |
|---|---|---|---|
| tDMV | Data Mask Valid (RWDS setup to end of initial latency) | 0 (min), all freqs | ns |
| tACC | HyperRAM memory-space write initial access (max) | 35.00 (200 MHz) … 40.00 (100 MHz) | ns |

**Other named timings referenced elsewhere:** **tDPDOUT** (DPD exit → standby, §5.2.1/§7.1.3),
**tVCS/tRPH/tRP/tRH/tPD** (reset/power, Table 8.3), **tCMS** (Fig 9.4/9.5 notes, CS#-related),
**tPOR_CK / PORTime** (HyperFlash RSTO# extension, Fig 8.4). All device-dependent unless tabulated above.

---

## 11. Load-Bearing Constants — RTL Quick Reference
- **CA** = 48 bits / 3 words / 6 DQ edges, **MSB byte first, center-aligned**. CS# falls with CK idle (CK Low, CK# High).
- **CA[47]** = R/W# (1=read); **CA[46]** = AS (1=register); **CA[45]** = Burst Type (1=linear);
  **CA[44:16]** = Row+UpperCol (A31:A3); **CA[15:3]** = reserved (=0); **CA[2:0]** = LowerCol (A2:A0).
- **Latency select:** RWDS-during-CA **Low = 1×, High = 2×**; count = **CR0[7:4]** (3–16 clocks, codes
  `1110`/`1111` = 3/4); **fixed vs variable = CR0[3]** (1 = fixed, default).
- **Read:** RWDS = edge-aligned source-synchronous read strobe (slave-driven). **Write (with latency):** RWDS =
  byte mask, master-driven, **High = mask**; write data center-aligned to CK.
- **Byte A = word[15:8]** on CK rising / RWDS-rising; **Byte B = word[7:0]** on CK falling / RWDS-falling.
  Register space always big-endian.
- **Zero-latency writes** (typical CR/register writes): master must **not** drive RWDS, **no masking**,
  full-word writes only; data follows CA immediately.
- **Burst:** wrap boundary from **CR0[1:0]** (128/64/16/32 B), legacy vs hybrid from **CR0[2]**
  (1=legacy, 0=hybrid). Address wraps by clearing low log2(words) bits.
- **Read completion:** count returned words gated on the RWDS strobe (survives row/page latency gaps); guard
  with underrun + timeout. Chop bursts to **MAX_BURST_COUNT = tCSM / tCK** so CS# never exceeds tCSM.
- **Timing (HyperRAM, Table 9.4/9.5):** tRWR 35/36/37.5/40 ns; tCSHI 5/6/7.5/10 ns; tCSS 2/3/3/3 ns;
  tCSH 0 ns (+ extra clocks per note 1); tACC ≤ 35…40 ns; **tCSM 4 µs (Ind) / 1 µs (Ind-Plus)** @ 200/166/133/100 MHz.
- **CR1 layout and latency-code↔frequency mapping are device-dependent** — pull from the **W957D8NB datasheet**,
  not this generic spec.

---

*Spec file: `hyperbus_spec.txt` (page numbers = spec's "Page N of 44"). Device register addresses/reset values
cross-checked against `/home/tcovert/projects/agilex_3_ai_benchmarks/sim/hyperbus/w957d8nb_bfm.sv`.*

# Instrumented FPGA Dataset v1 — `infra-hyperram`

> **Frozen snapshot (captured 2026-07-08).** The measurements below describe the state at capture
> time and are intentionally NOT updated with later progress — they are provenance for
> `fpga_dataset_v1.db`. In particular, the "PNR BLOCKED / no bitstream / hardware verification
> BLOCKED" rows were subsequently RESOLVED on the `ddio-200` branch (Fitter 24403/24404 fixed,
> bitstreams produced, silicon-verified up to 344.7/337.3 MB/s at 175 MHz with `ERR_COUNT=0`) —
> see `fpga/axc3000/README.md` for the current hardware status.

Machine-readable log of an **AI-led FPGA IP generation** run: the clean-room
HyperBus/HyperRAM controller IP in this repo (controller + AXI4/Avalon front-ends
+ PHY + bandwidth test), built by an agentic LLM (Claude Opus 4.8) under the Claude
Code harness with multi-agent Workflow orchestration. Captured to quantify
effort/effectiveness of agent-generated RTL.

**Database:** [`fpga_dataset_v1.db`](fpga_dataset_v1.db) (SQLite). Flat exports in [`csv/`](csv/).

## Tables
Five per the collection spec — `tasks`, `defects`, `wip_snapshots`, `perception`,
`agent_interactions` — plus four supplementary logs the spec's v1 outline also asked
for: `qor_log`, `verification_log`, `tool_env`, `design_meta`.

## Headline numbers (all measured)
| Metric | Value |
|---|---|
| Total agent task-effort | **331 min (~5.5 h)** across 28 tasks |
| RTL generated | ~3492 lines SystemVerilog across 17 modules |
| Agent invocations (model calls) | 1320 |
| Defects (all origins) | 7, total rework **47 min** |
| Sim verification | 5/5 Verilator TBs PASS; formal: none |
| Hardware (AXC3000) | SYNTH PASS, **PNR BLOCKED** (IOPLL two-phase-clock periphery placement); no bitstream |
| Perceived speedup | **~64x** (2 months full-time ~320 h -> ~5 h) |

## Effort by stage
| stage | task-effort (min) | tasks |
|---|---|---|
| RTL | 120 | 8 |
| TB | 70 | 4 |
| SPEC | 31 | 6 |
| DEBUG | 31 | 1 |
| REVIEW | 28 | 5 |
| DOC | 26 | 2 |
| ARCH | 16 | 1 |
| PNR | 9 | 1 |

## Defects by origin / detection
| origin | detected_at | count | rework_min |
|---|---|---|---|
| rtl-logic | sim | 2 | 22 |
| constraint-timing | pnr | 1 | 9 |
| rtl-interface | sim | 1 | 6 |
| integration | pnr | 1 | 5 |
| tool-flow | pnr | 1 | 4 |
| tool-flow | sim | 1 | 1 |

All RTL defects were caught in **sim** (Verilator), none escaped to synthesis/PNR.
The only PNR-stage items are the two integration fix-ups and the one **unresolved**
physical blocker (`constraint-timing`: the Agilex IOPLL cannot route both clock
phases to the I/O DDIO region — needs the board package bank-map or hardened DDR-I/O).

## Methodology / proxy definitions (§1.2 scrubbing applied)
- **Provenance:** `tasks`/`agent_interactions` derived from the per-agent workflow
  transcripts (`agent-*.jsonl`): `wallclock_min` = last-minus-first message timestamp;
  `invocations` = assistant turns; `context_tokens` = max(input+cache) over turns;
  `iterations` = count of build/sim/compile tool-runs. `defects` from the verify-agents'
  reports; `qor_log` from the Quartus synth/fit logs; `verification_log` from the
  Verilator run; `design_meta` computed from the committed RTL. **Content is scrubbed**
  — only quantitative proxies + file/label pointers are stored; `prompt_ref` points to
  the LOCAL workflow script archive (not committed).
- **`prompt_complexity`** (1–5): proxy = task-prompt size buckets (<=400/800/1400/2200/more tokens).
- **`accepted_without_edit`** (0/1): 0 iff that stage's artifact was later corrected by a
  verify/fix agent (measured), else 1.
- **`origin`='agent'`, `agent_tool`='claude'** throughout — this run had **no human hand-RTL**;
  the `hand`/`hybrid` enum values are present for future comparison rows.

## Honest caveats
- **`wallclock_min` is per-task effort**, and many tasks ran **concurrently** (parallel
  agents within a phase), so the 331-min sum overstates elapsed wall-time — it is the
  right measure of *work performed*, not calendar time.
- **`perception`** is a single respondent's whole-project estimate (exceeds the weekly
  survey's ±500% band); logged verbatim, not normalized.
- **`wip_snapshots`** is one honest end-of-session snapshot (not a weekly series).
- **Hardware verification is BLOCKED**, logged as such — the read/write bandwidth was
  proven in *simulation* (cycle-accurate), not yet on silicon.

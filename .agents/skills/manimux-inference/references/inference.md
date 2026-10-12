# Inference implementation contract

For supported schedules, parameters, constraints and YAML examples, use the
[public inference guide](../../../../guideline.html#algorithms).
The [model sampling matrix](inference-matrix.md) records backend capabilities;
the running server's advertised sampling modes remain authoritative.

## Ownership and entry points

| Responsibility | Implementation |
| --- | --- |
| Algorithm registry and strategy factory | `manimux/runtime/inference.py` |
| Request admission, refill, rate cap and serial completion | `manimux/runtime/scheduling.py` |
| Config normalization and compatibility validation | `manimux/runtime/__init__.py` |
| Canonical action clock, trimming and handoff | `manimux/runtime/timeline.py` |
| ACT query cadence and opt-in fusion | `manimux/runtime/temporal_ensemble.py` |
| BID backward candidate selection | `manimux/runtime/bid.py` |
| Shared candidate decoding and timestamp interpolation | `manimux/runtime/candidates.py` |
| AAC selection and adaptive execution | `manimux/runtime/aac.py` |
| AutoHorizon selected-prefix execution | `manimux/runtime/autohorizon.py` |
| RTC and PAINT request conditions and acceptance | `manimux/runtime/rtc/strategy.py`, `manimux/runtime/paint.py` |

`serial` and `async` share `DefaultChunkStrategy`. Main removed the old `manimux`
selector. Do not reintroduce a second request scheduler inside an algorithm.
Adapters preserve request sampling fields and own conversion to canonical absolute
joint targets; model frameworks own sampling, normalization and native output.

## Optional algorithm composition

The factory wraps a supported base strategy with `EnsembleStrategy`. Candidate
selection and preparation share the final timeline handoff timestamp. The inline
`decode_strategy_action` hook is also used during warmup; ordinary decoding remains
unchanged. Candidate paths require an adapter declaring `supports_candidate_decoding`
and complete, untrimmed absolute-joint rows. See the
[runtime integration contract](../../manimux-development/references/runtime-config.md).

- Fixed BID retains its full selected, unfused prediction only after acceptance.
  Serial scoring shifts it by the executed prefix, matching the supplied reference.
  Async scoring interpolates overlapping future timestamps without extrapolation or
  expired leading rows. K counts future rows; optional AAC length instead caps source rows.
- ACT retains the original exponential weighting. Composed paths interpolate at exact
  action times and discard rejected history. RTC/PAINT receive the accepted fused plan
  for subsequent conditioning. BID's own reference remains unfused.
- Serial ACT retains its explicit `act_temporal_ensemble` strategy identity. It fuses
  full predictions before cropping K rows and holds the last row for one interval.
  Its reference origin is the actual commit time, including inference gaps.
- Async AAC/AutoHorizon and BID + AAC retain the selected source prefix; latency
  consumes that prefix instead of extending its end. An entirely expired prefix is
  rejected. AAC's async backward selector uses aligned accepted candidate history;
  plain serial AAC retains its original client selection path.
- RTC + ACT and PAINT streaming/composition sample conditions on the observation's
  exact time grid. Missing required prefixes fall back to unconditioned sampling;
  replies based on replaced plans are rejected. Existing standalone paths remain intact.

Pause, Home and rollout/reset transitions clear strategy and ensemble history.
Prepare alone does not run warmup: the current GUI requires Activate Warmup.
Start drains warmup and waits for acknowledged backend reset before formal requests.
Warmup previews never seed accepted-plan history.

## Validation and provenance

Use synthetic timestamps to check fractional-grid overlap, latency trimming, fixed
versus adaptive prefixes, stale-plan rejection and lifecycle resets. Exercise the
actual wire path with fake servers and validate configurations through `load_config`.
The [reproduction records](reproductions/README.md) separate reference formulas,
local adaptations and real-model/robot evidence. Runtime compatibility and offline
checks alone do not establish hardware performance.

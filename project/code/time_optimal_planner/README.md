# Time-Optimal Planner V2

This directory is a parallel implementation.  It does not replace or modify
`Algorithm.c`, `Game_logic.c`, `path.c`, `Control.c`, or `Map_Path_Data.*`.

## Competition model

- Playable grid: 10x14, 200 mm per cell.
- Car footprint: one full square cell; arbitrary straight empty-car shortcuts
  use continuous swept-square collision checks.
- Translation: 500 mm/s average in every direction, including diagonal motion
  and pushing.
- Pushes are axis-aligned.  The car heading does not need to match the push.
- A 90-degree in-place rotation costs 1000 ms.
- Near/far recognition both cost 200 ms after the car is stopped and facing the
  object.  Valid center distances are 1, 2, or 3 cells.  Other targets block the
  sight line.
- A bomb is pushed into an internal wall cell, clears the internal 3x3 wall
  area, never affects objects, cannot chain, and adds a 500 ms wait.
- At most five boxes/targets and ten bombs.
- IDs may repeat.  Any equal-ID box/target assignment is legal.  If exactly one
  active box and target remain unknown, the pair is inferred.
- All internal walls disappear after the final delivery.  The finish cells are
  `(4,0)` and `(5,0)`, with final heading right.

## Modules

- `TopPlanner.*`: bounded anytime weighted A* over push macro-actions.  Box
  matching, bomb motion/explosion, dynamic walls, return, repeated IDs and
  interleaved delivery/recognition decisions share one time-cost model.
- `TopGrid.*`: 8-neighbor Dijkstra, no corner cutting, swept-square visibility,
  and time-first/fewest-points walk compression.
- `TopPath.*`: self-contained raw path, compact execution path, 32-bit event
  protocol, segments, durations, and dynamic effect cells.  Consecutive
  collinear pushes are emitted as one execution segment.
- `TopVerify.*`: independent segment replay.  It rechecks collision, push
  geometry, matching, explosions, mandatory waits, recognition geometry,
  completion, duration totals and the exported end state.
- `TopControlV2.*`: rolling plan/execute/identify/replan state machine.  It is
  intentionally independent from the old monolithic `Control.c`.
- `TopLegacyAdapter.*`: imports unchanged legacy globals and exports compatible
  `Position` geometry/event bits.  Rich rotate/identify/wait semantics still
  need `TopControlV2` segment execution.

## Objective and bounded search

The primary cost is predicted execution time.  The returned total is:

```text
planning_ms + translation_ms + rotation_ms + recognition_ms + bomb_wait_ms
```

Default search settings are a 1000 ms deadline, 4096-node pool, 4000 expansion
cap and a strongly goal-directed weight.  The first verified solution is kept;
remaining budget searches for alternatives.  Timeout returns a verified
solution when one exists, otherwise an explicit failure.

Unknown IDs make the future conditional on camera output.  The planner returns
one executable partial plan, then replans after recognition.  If a known pair
can be delivered faster than the next useful recognition, a delivery may be
returned first.  This enables recognition and pushing to interleave without
pretending that an unseen ID is known.

## Integration outline

1. Keep one `top_control_v2_t` in static storage; do not put it on a task stack.
2. Build a `top_problem_t` from the camera map.  Use explicit `id_known` flags
   because numeric ID zero is valid.
3. Supply a monotonic millisecond callback in `top_config_t`.  The measured
   planning time is included in `predicted_total_ms`.
4. Call `top_control_v2_plan()` while the motor target is held at zero.
5. Execute `top_segment_t` sequentially:
   - `MOVE`/`PUSH_*`: remap the referenced execution points and pass them to
     `path_follow`.
   - `ROTATE`: call the yaw rotation primitive.
   - `WAIT`: use a non-blocking 500 ms state.
   - `IDENTIFY`: request the correct camera model and submit the ID.
6. After the last segment, `top_control_v2_segment_completed()` either finishes
   or requests the next planning cycle.

Do not feed stationary rotate/identify/wait points directly into old
`path_follow`; those are control events, not translational waypoints.

## Verification

Run all host tests from the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File tools\time_optimal_planner_tests\run_tests.ps1
```

Coverage includes exact small cases, repeated IDs, last-pair inference,
interleaved delivery, mandatory bomb use, new control flow, all six nonduplicate
legacy preset maps, 200 deterministic randomized maps, and deliberately damaged
output paths.

The parallel Keil project is `project/mdk/rt1064_time_optimal.uvprojx`.  It
compiles every new module with ARM Compiler 6 while leaving the original project
file unchanged.

Current static planner workspace is about 271 KiB with the 4096-node pool.  On
ARM Compiler 6 it is explicitly placed in the project's 512 KiB cached OCRAM
region instead of DTCM.  One `top_result_t` is about 41 KiB on the host ABI.
Confirm the final ARM map after the new control is selected by `main`, because
unused modules are removed by the linker in the compile-only parallel project.

## Remaining board calibration

The motion model intentionally uses the agreed 0.5 m/s average and has no
separate acceleration term.  After board/vehicle logging is available, compare
predicted and measured segment time, then adjust speed or add a direction-change
penalty without changing search legality.

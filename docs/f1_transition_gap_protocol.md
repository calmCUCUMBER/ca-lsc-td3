# Archived development diagnosis: transition gap

> Status: development/diagnostic only. This document is retained for
> traceability, but the transition gap is not a paper contribution and the
> TRANS=10/13 experiments are not part of formal F1. Formal experiments use
> the single frozen low-level configuration in `config/nominal_low_level.env`.

This protocol investigates the repeated non-convergence at
`Va=10 m/s, lambda=0.6` without changing the frozen schema-v9 flight
protocol or its pass thresholds.

## Current evidence

The five schema-v9 smoke runs give zero F1 passes and only two complete
measurement windows.  After lambda reaches its target, the first measurement
window does not begin for 9.75--16.95 s; one run never achieves the required
two-second joint Va/lambda dwell.  Across the complete controlled phases,
airspeed crosses its target 10.6 times per run on average.  This is an
underdamped coupled response, not a one-sample evaluator failure.

The diagnostic reconstruction finds no repeated pusher-throttle or lift
collective saturation under the old thresholds.  The earlier statement that
the elevator did not saturate is withdrawn: the old evaluator checked
`abs(servo)>=0.95`, while the GZ standard_vtol elevator joint clips commands
at +/-0.53 rad.  Archived schema-10/11 logs do not contain direct joint
position and must be interpreted with this limitation.  The pusher maximum remains about
0.33--0.36 against the configured 0.45 limit.  Three of the four runs with a
measurement segment spend more than half of that segment below the modeled
weight-support requirement.  The repeated sequence is:

1. lift-rotor collective is unloaded;
2. altitude falls and the experiment-only PX4 height-to-pitch loop pitches up;
3. wing support recovers but drag rises and Va falls;
4. the pusher PI reacts after the Va error appears;
5. Va and height overshoot into the next cycle.

This supports the interpretation `controller-limited transition gap`.  It
does not yet prove a purely aerodynamic infeasible point.

## Correct Gazebo LiftDrag convention

The SDF field `a0` is an angle-of-attack offset added by Gazebo:

```text
alpha_plugin = alpha_geometric + a0
CL = cla * alpha_plugin
```

It is not a zero-lift angle to subtract.  The old offline proxy used the wrong
sign and returned eta_L approximately zero even at stable
`Va=18 m/s, lambda=1.0`.  The corrected proxy now gives approximately 0.50 at
`Va=10, lambda=0.3`, 0.84 at `Va=14, lambda=0.6`, and 1.00 at
`Va=18, lambda=1.0`.  The proxy excludes main-wing control deflection and is
still model-based rather than measured aerodynamic force.

## Diagnostic telemetry extension

Telemetry schema 10 adds diagnostics only; dwell, fixed three-second window,
90% occupancy, 0.6 m/s error guard, and hard aborts remain schema-v9 protocol
rules.

New channels are:

- actual PX4 roll/pitch/yaw attitude setpoints;
- filtered Va and filtered pusher error;
- pusher PI integral and unsaturated throttle request;
- attitude-setpoint message freshness.

These variables separate pitch-demand lag from pusher PI bandwidth and true
actuator saturation.

The schema-10 chain was verified with one non-formal smoke run.  All six new
diagnostic channels were finite for 61/61 measurement samples, and attitude
setpoint age stayed between 0.9 and 11.7 ms.  In that run:

- Va occupancy was 40.98% and maximum Va error was 0.636 m/s;
- lambda occupancy remained 100%;
- pusher unsaturated demand was only 0.259--0.316 versus the 0.45 limit;
- pitch tracking RMSE was 0.0667 rad;
- modeled vertical support averaged 46.56 N versus 49.28 N weight and was
  below weight for 77.05% of the measurement window.

This smoke is instrumentation evidence only and is not one of the formal ten
repeats.

## Ten-repeat experiment

Rebuild because the PX4 DDS publication set and ROS recorder changed:

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

make -C PX4-Autopilot px4_sitl_default \
  BUILD_DIR=build_ca_make -j"$(nproc)"

source /opt/ros/humble/setup.bash
colcon --log-base ros2_ws/log_ca build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build_ca \
  --install-base ros2_ws/install_ca \
  --symlink-install \
  --packages-select ca_lsc_transition
source ros2_ws/install_ca/setup.bash
```

Collect ten data-quality-valid independent restarts, retaining up to five
protocol-invalid retries:

```bash
F1_VALID_TARGET=10 F1_MAX_ATTEMPTS=15 \
./scripts/run_phase0_75_va_hold_repeats.sh \
  data/evaluation/f1_gap_va10_lambda_0p6_v10_10x \
  10.0 0.6 220 f1
```

The repeat runner may return a nonzero status when valid runs are physical
failures; that is expected for an F1 boundary experiment and does not discard
the data.  Generate the aligned diagnostic products afterward:

```bash
PYTHONPATH=src python -m ca_lsc_td3.evaluation.f1_transition_gap \
  data/evaluation/f1_gap_va10_lambda_0p6_v10_10x
```

Do not tune the pusher PI or height-to-pitch gains during these ten repeats.
After the instrumented evidence is reviewed, any controller change must be a
predeclared A/B experiment and the final F1 grid must use one frozen controller
configuration.

## Authority-handover A/B experiment

The formal schema-10 A arm collected 10 valid repeats (plus one excluded
pre-measurement transient): 0/10 passed, only 3/10 completed the fixed window,
and 7/10 ended in settle timeout.  No valid run produced a hard physical
abort.  The completed windows had only 23.3--29.5% Va-band occupancy.

The runner used `VT_ARSP_BLEND=8 m/s` and selected
`VT_ARSP_TRANS=min(Va, 13 m/s)`, so the A arm used
`VT_ARSP_TRANS=10 m/s`.  Around the requested 9.7--10.3 m/s target band, the
stock PX4 airspeed blend therefore leaves approximately 0--0.15 MC pitch
weight while the external `lambda=0.6` command retains 40% lift-rotor
collective.  This A/B tests whether that uncoordinated pitch-authority handover
causes the repeated limit cycle.

Telemetry schema 11 records `VT_ARSP_BLEND`, `VT_ARSP_TRANS`, and an explicitly
named `mc_pitch_weight_proxy`.  The proxy reconstructs PX4's airspeed schedule;
it is not a new controller input and does not change the schema-v9 pass rules.

The B arm changes exactly one PX4 parameter:

```text
A: VT_ARSP_BLEND=8, VT_ARSP_TRANS=10  (already complete, 10 valid runs)
B: VT_ARSP_BLEND=8, VT_ARSP_TRANS=13  (collect 5 valid runs)
```

All unloading-pitch gains, pusher PI settings, lambda command, thresholds, and
the 20 s transition horizon remain frozen.  Run B with:

```bash
./scripts/run_f1_va10_lambda0p6_authority_b.sh \
  data/evaluation/f1_gap_va10_lambda_0p6_trans13_v11_5x \
  220
```

Predeclared interpretation:

- 4/5 or 5/5 B passes, with no repeated hard abort, strongly confirms an
  early pitch-authority handover as the principal A-arm failure mechanism;
- 3/5 is boundary evidence and requires five more valid repeats;
- 0/5--2/5 does not confirm the hypothesis; inspect pitch-loop tuning next
  without relabeling the A-arm result;
- changing the timeout or pass thresholds is not permitted in this A/B.

The primary outcomes are valid pass count and fixed-window completion count.
Secondary outcomes are Va occupancy/error, pitch tracking RMSE, MC pitch-weight
proxy, actuator saturation, altitude error, and modeled support shortfall.

### Completed B-arm result (2026-09-01)

The B arm required six independent starts to obtain five valid repeats.  One
start was excluded for a pre-measurement altitude transient.  All five valid
runs completed the fixed three-second window and four passed F1:

```text
A (transition=10): 0/10 pass, 3/10 measurement complete
B (transition=13): 4/5 pass, 5/5 measurement complete
```

For the four passing B runs, Va-band occupancy was 100%, mean maximum absolute
Va error was 0.195 m/s, mean altitude RMSE was 0.445 m, mean pitch-tracking
RMSE was 0.00393 rad, and mean MC pitch-weight proxy was 0.604.  No valid B run
had a hard physical abort or pusher, collective, or servo saturation.

The single valid B failure completed its window but had 27.9% Va occupancy,
1.139 m/s maximum Va error, and 2.473 m altitude RMSE.  It is retained as
physical evidence.  Consequently, the predeclared A/B criterion strongly
supports early pitch-authority handover as the principal A-arm failure
mechanism, while the B configuration itself remains `boundary_feasible`
rather than `nominal_feasible`.  It must not yet be silently adopted as the
final frozen F1 controller.

### Predeclared independent confirmation

Before freezing the B configuration, collect a new five-valid-run batch in a
separate directory with identical settings and no further tuning.  Since the
discovery batch passed 4/5, the confirmation batch must pass 5/5 for the two
batches to reach the predeclared 9/10 freeze threshold.  No hard physical
abort is allowed.  Protocol-invalid startup transients are excluded and
replaced exactly as in the discovery batch.

```bash
./scripts/run_f1_va10_lambda0p6_authority_confirm.sh \
  data/evaluation/f1_gap_va10_lambda_0p6_trans13_v11_confirm_5x \
  220
```

If the combined result is at least 9/10 with zero hard physical aborts, freeze
`VT_ARSP_BLEND=8` and `VT_ARSP_TRANS=13` for the next F1 grid.  Otherwise, do
not tune thresholds or discard failures; proceed to an eta-C-aware attitude
handover or a separately predeclared controller redesign.

### Confirmation outcome and freeze decision (2026-09-01)

The independent confirmation completed five valid repeats without needing a
protocol retry and passed 4/5.  The two trans-13 batches therefore combine to:

```text
10/10 fixed measurement windows complete
 8/10 F1 passes
 0/10 hard physical aborts
```

The predeclared 9/10 threshold was not met, so the static trans-13 controller
is **not frozen** and a new full F1 grid is **not authorized** by this result.
The confirmation failure had 50% Va-band occupancy and 0.448 m/s maximum Va
error.  Together, the two trans-13 failures averaged 38.9% occupancy,
0.793 m/s maximum Va error, 1.676 m altitude RMSE, and 0.690 MC pitch-weight
proxy. Neither failure involved a hard abort. The archived “no actuator
saturation” statement applies only to the old +/-0.95 PX4-channel threshold;
it is not evidence that the +/-0.53 rad GZ elevator joint remained off-limit.

Across the eight passes, mean Va occupancy was 99.6%, maximum Va error was
0.199 m/s, altitude RMSE was 0.569 m, pitch-tracking RMSE was 0.00429 rad, and
MC pitch-weight proxy was 0.602.  Static retention of MC pitch authority thus
removes the dominant A-arm failure mode but does not provide the repeatability
required for a final controller.  The later architecture decision superseded
that branch: schema 13 synchronizes MC/FW attitude weights to the executed
transition-allocation lambda.  No separate eta-C-aware PX4 handover is used.

## Archived schema-12 platform diagnosis

PX4 now publishes the actual MC/FW pitch weights plus the
existing virtual MC/FW normalized torque demands through DDS. The ROS
recorder logs these values with independent freshness ages and records an
elevator joint-limit command proxy using the SDF limits +/-0.53 rad.

```bash
./scripts/run_f1_control_authority_sweep.sh \
  data/evaluation/f1_control_authority_schema12 \
  220
```

The archived development script can run three arms
(`VT_ARSP_TRANS=10/13/15`, `VT_ARSP_BLEND=8`) with five valid
independent restarts at `Va=10 m/s, lambda=0.6`. Interpret the joint trends
in actual weights, normalized virtual demands, elevator joint-limit command
fraction, altitude RMSE and pitch-rate RMSE. Do not select the final
controller from pass count alone. The torque demands remain dimensionless
PX4 controller outputs, so this experiment validates the structural coupling
but does not complete F4 or produce paper-level eta_C. It predates the
lambda-synchronized allocator and is retained only to document why the
architecture changed.

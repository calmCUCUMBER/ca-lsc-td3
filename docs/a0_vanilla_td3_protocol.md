# A0 Vanilla MLP-TD3 protocol

Status: initial smoke stage after Physics Prior Only was frozen.

This stage validates the Vanilla TD3 learning harness before A1/A2 are added.
It must not be used as evidence that the final ROS/PX4 transition policy has
been learned.

## Frozen A0 contract

A0 observation is 10-D and deliberately excludes capability features:

```text
[Va, alpha, gamma, e_h, v_z, theta, q, lambda_exec, P_L, P_P]
```

The actor is:

```text
10 -> FC64 -> FC64 -> tanh(1)
```

The action maps to:

```text
lambda_d = (a + 1) / 2
```

The critics are standard TD3 twin critics:

```text
[o, a] -> FC128 -> FC128 -> Q
```

No `eta_L`, no `eta_C`, no physics prior, no residual action, no CfC, no safety
critics, and no Lagrangian are present in A0.

## Current smoke environment

`ToyA0TransitionEnv` is a cheap deterministic contract environment, not the
final SITL training environment.  It now includes an oracle-reachable transition
task so a learning smoke can distinguish software plumbing from a mathematically
impossible MDP.

The toy pusher is intentionally strong enough to hold approximately
`Va=15 m/s` even when `lambda≈1`, matching the frozen architecture where pusher
airspeed control is independent from lift unloading.

## Training harness requirements

- raw SI telemetry remains the external environment contract;
- training uses a running observation normalizer, frozen for evaluation;
- `learning_starts` is separate from `batch_size`;
- replay stores `terminated` and `truncated` separately;
- Bellman bootstrap masking uses `terminated`, not time-limit truncation;
- reward keeps `energy_reward_weight=0` until Power Qualification is solved;
- hard failure receives a terminal penalty;
- evaluation telemetry records `state`, `action`, `reward`, and `next state`
  explicitly.

## Smoke command

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

./scripts/run_a0_td3_smoke.sh \
  data/training/a0_vanilla_td3_smoke_v2 \
  20 \
  20260906
```

Passing this smoke means the A0 software and local learning loop are healthy.
The next formal step is to connect the same A0 contract to the ROS/PX4 training
environment; it is not a reason to start A1/A2/CfC yet.

## ROS/PX4 environment-contract smoke

Status: DONE/FROZEN after `data/training/a0_ros_contract_smoke_v1` was
reanalyzed with the corrected 10 Hz measurement-window reconstruction.

Before ROS/PX4 training starts, run scripted actions rather than a learned
policy.  The runner checks four fixed target conditions:

```text
lambda_d = 0.0 at Va=12
lambda_d = 0.3 at Va=12
lambda_d = 0.6 at Va=12
lambda_d = 0.5 at Va=14  # known-safe scripted condition
```

The analyzer reconstructs the frozen A0 10-D state from telemetry, applies the
A0 action mapping `lambda_d=(a+1)/2`, recomputes the debug reward with
`energy_reward_weight=0`, and emits explicit `terminated/truncated` contract
bits.  This is still not policy training.

Only rows satisfying all three conditions are used for A0 step reconstruction:

```text
command_state == TRANSITION_FW
va_hold_measurement_active == 1
lambda_external_active == 1
```

The approximately 20 Hz telemetry stream is then resampled at the A0 control
rate, `dt=0.1 s`.  A three-second measurement window should reconstruct roughly
29--30 A0 transitions.

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
source ros2_ws/install_ca/setup.bash

./scripts/run_a0_ros_contract_smoke.sh \
  data/training/a0_ros_contract_smoke_v1 \
  220
```

If this passes, the next step is a fresh randomly initialized A0 ROS/PX4
training smoke.  Do not load the toy checkpoint into PX4.

## A0 ROS/PX4 online training smoke

Status: the x4 online runtime, retry contract, and schema-32 task-success
semantics are qualified. The clean `a0_ros_td3_x4_nominal_50ep_v2` run has
completed; its next use is the frozen x1 deterministic evaluation specified
below, not another training or platform-tuning run.

This runner launches one SITL transition per episode.  During the transition,
the trainer receives `/ca_lsc/a0_observation`, selects an A0 action online at
10 Hz simulation time, publishes `/ca_lsc/a0_rl_action`, stores real
transitions in replay, updates the unchanged `TD3Agent`, and saves a checkpoint.

The transition node uses:

```text
schedule_mode = a0_rl_external
```

No capability features, physics prior, residual action, hard capability shield,
safety critic, Lagrangian, CfC, or toy checkpoint are used.

The frozen task/infrastructure split is:

```text
task success:       command node confirms TRANSITION_FW -> HOLD_FW
task failures:      forward_transition_timeout, airspeed/altitude/vertical
                    runaway, pre-measurement transient, PX4 failsafe
infrastructure:     action timeout, missing terminal, launch failure,
                    runner wall-clock timeout
```

The Va/lambda/altitude/vertical-speed dwell is only `handover_ready`: it
releases the external PX4 test hold and is available as a diagnostic. It is
not an RL terminal and never receives the success reward. After release,
nonterminal A0 observations are suppressed until PX4 reports either HOLD_FW or
a command error, so action-free handover samples cannot enter replay.
The A0 trainer metrics also latch the terminal command state, handover-ready
flag, handover dwell, and actual fixed-wing confirmation. Any episode reported
as `success` must carry `terminal_actual_fw_confirmed=true`; otherwise the smoke
fails automatically.

All task dwell and safety-envelope timers use ROS simulation time. Wall time is
reserved for process watchdogs, DDS delivery grace, and runtime measurements;
therefore x1 and x4 expose the same task MDP.

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

source /opt/ros/humble/setup.bash
source ros2_ws/install_ca/setup.bash

./scripts/run_a0_ros_training_smoke.sh \
  data/training/a0_ros_td3_training_smoke_v2 \
  3 \
  180 \
  20260907
```

Passing this smoke means online action ownership, replay, finite losses,
network updates, checkpointing, and terminal semantics are integrated with
PX4/Gazebo.  It is still not a formal A0 performance result.

## Simulation-speed qualification

Status: x4 QUALIFIED after
`data/training/a0_sim_speed_qualification_x4_v6`.

The speed qualification is intentionally not a TD3 performance experiment.  It
uses a fixed action and disables learning updates so that the result isolates
ROS/Gazebo/PX4 timing and observability.  Gazebo is run at the requested
`sim_speed_factor`, while the physics integration step remains frozen.  The
trainer and recorder use simulation time from `/clock` for observation/action
scheduling, and record both simulation-time and wall-time step intervals.

Validated x4 metrics:

```text
mean RTF during RL-active window: 4.005
mean observation dt:              0.0500 s sim-time  (~20.0 Hz)
mean RL step dt:                  0.1030 s sim-time  (~9.71 Hz)
max stale fraction:               0.0
min action/step ratio:            1.0
learning enabled:                 false
fixed action:                     0.0  # lambda≈0.5
qualification pass:               true
```

The two qualification episodes terminated with `forward_transition_timeout`
under the fixed action.  That terminal reason is acceptable for this stage
because the gate is checking scheduler integrity, terminal propagation,
fresh actions, and replay observability at x4 rather than policy quality.

Reproduce the x4 qualification with a fresh directory:

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

./scripts/run_a0_sim_speed_qualification.sh \
  data/training/a0_sim_speed_qualification_x4_v6 \
  2 \
  180 \
  20260907 \
  "4"
```

The three-episode learning-enabled pilot at
`data/training/a0_ros_td3_x4_learning_pilot_v6` passed its short smoke, but it
was not sufficient to freeze long-runtime infrastructure.  The subsequent
50-episode diagnostic run found six action timeouts, six pre-RL invalid
attempts, and one missing terminal.

Validated learning-enabled x4 metrics:

```text
episodes:             3
pass failures:        none
terminal observed:    3/3
action timeouts:      0
mean observation dt:  0.0500 s sim-time
mean RL step dt:      ~0.1000 s sim-time
RTF range:            3.86--4.02
replay size:          705
critic updates:       450
actor updates:        225
losses:               finite after learning starts
```

The 50-episode result is retained as
`data/training/a0_ros_td3_x4_nominal_50ep_v1` and is classified DIAGNOSTIC
FAIL.  Its checkpoint must not be resumed or used as a paper result.

The repaired runner now:

- resets all simulation-clock deadlines at every SITL episode and on clock
  rollback;
- serves actions from a read-only actor snapshot while a background learner
  performs gradient updates;
- treats runtime/action transport failures as Bellman truncations without the
  task failure penalty;
- counts only attempts that reached RL_ACTIVE as training episodes and writes
  every launch to `attempt_metrics.csv`;
- repeats early-abort terminal messages briefly before the recorder exits.

Run the next clean-runtime gate with a fresh directory:

```bash
./scripts/run_a0_ros_training_smoke.sh \
  data/training/a0_ros_td3_x4_runtime_clean_10valid_v1 \
  10 \
  180 \
  20260907 \
  4 \
  --max-attempts 15
```

Acceptance requires ten valid RL_ACTIVE episodes, zero infrastructure-invalid
attempts, zero action timeout, ten observed terminals, zero infrastructure
failure penalties, no learner backlog/failure, zero RL-active stale fraction,
and simulation-time observation/action periods near 0.05/0.10 s.  Do not tune
TD3, reward, network size, or exploration before this gate passes.

Post-run audit note (schema 30): the original summary for
`a0_ros_td3_x4_runtime_clean_10valid_v1` reported seven action timeouts and
only eight valid episodes. The raw telemetry proves those labels were false:
the first terminal samples carried six vertical-speed runaways and one
altitude runaway, all with a fresh action. During the recorder's DDS terminal
delivery grace period the trainer correctly stopped publishing, action age
then became stale, and a repeated payload overwrote the physical terminal
reason. Under first-terminal-wins semantics the first ten attempts are ten
valid RL_ACTIVE episodes (five forward-transition timeouts and five
vertical-speed runaways), with 10/10 terminals, zero genuine action timeout,
zero RL-active stale fraction, and mean observation/action periods of
0.0501/0.1000 s simulation time. The raw run therefore satisfies the runtime
gate after reclassification; its original summary is retained unchanged as
an audit artifact.

Schema 31 prevents recurrence at both ends of the contract: the trainer
latches the first terminal payload, while the recorder cannot replace an
already-latched physical abort with `a0_rl_action_timeout` during terminal
delivery grace. A task failure after RL_ACTIVE is a valid training episode
and must not trigger a replacement SITL launch.

A two-episode post-patch integration smoke at
`data/training/a0_ros_td3_x4_runtime_patch_learning_smoke_v1` passed: 2/2
valid attempts, 0 stale/action timeout, 2/2 terminals, 196/196 queued learner
updates completed, no infrastructure penalty, and checkpoint reload remained
exact.  This verifies the patched path but does not replace the 10-valid gate.

## A0 deterministic nominal evaluation

Status: this is the next paper-line experiment after the schema-32 A0 x4
50-episode training run. Evaluation uses x1, the frozen 50 m nominal setup,
ten cold SITL restarts, and the final A0 policy plus its matching observation
normalizer.

The evaluator enforces all of the following:

- deterministic Actor inference (`explore=False`);
- no replay insertion;
- no critic or Actor update;
- no normalizer update;
- no output checkpoint;
- identical policy tensor digest and normalizer state before and after the run;
- success only when PX4 reports actual `HOLD_FW`.

The preregistered decision bands for ten valid episodes are 0--2 successes:
continue A0 to 100 episodes; 3--5: continue A0 to 100 episodes; 6--8: candidate
A0 nominal baseline subject to the safety/trajectory audit; 9--10: freeze A0
nominal and start A1. Success rate is an experimental outcome, not a protocol
pass criterion.

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

./scripts/run_a0_deterministic_evaluation.sh \
  data/evaluation/a0_deterministic_nominal_eval_v1 \
  data/training/a0_ros_td3_x4_nominal_50ep_v2/checkpoints/a0_ros_td3_smoke.pt \
  data/training/a0_ros_td3_x4_nominal_50ep_v2/checkpoints/a0_ros_normalizer.json \
  10 \
  180 \
  20260908 \
  15
```

## A0 success-discovery exploration pilot

Status: the legacy IID exploration protocol is rejected after the fresh
100-valid-episode run and its frozen ten-restart evaluation both achieved zero
actual fixed-wing handovers.  This is an exploration-protocol diagnosis, not a
change to TD3, reward, PX4 allocation, or task termination.

The pilot changes exactly two coupled discovery settings:

```text
learning_starts:       256 -> 2000 replay transitions
exploration time scale: 0.1 s IID -> 5.0 s piecewise constant (simulation time)
```

The 5 s hold is derived from the frozen positive unloading rate and handover
dwell: raising `lambda_exec` from zero to 0.95 at 0.25/s needs approximately
3.8 s, followed by the 1.0 s handover dwell.  Physics time step, RL period,
Gaussian exploration magnitude, reward, network sizes, and TD3 update rules
remain unchanged.  Warm-up holds a uniformly sampled action; after warm-up the
Actor remains state-dependent while its Gaussian perturbation is held for the
same simulation-time interval.

This ten-valid-episode pilot tests discovery only.  It records, per episode,
the exploration segment count, warm-up action count, maximum continuous time
with `lambda_exec >= 0.9`, handover metadata, and actual `HOLD_FW` success.
Passing success discovery requires at least one authoritative success with
`terminal_actual_fw_confirmed=true`.  Do not interpret ten episodes as policy
convergence and do not change the reward during this pilot.

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1
set -o pipefail

mkdir -p data/training/a0_exploration_persistent_10ep_v1

./scripts/run_a0_exploration_pilot.sh \
  data/training/a0_exploration_persistent_10ep_v1 \
  10 \
  180 \
  20260909 \
  4 \
  15 \
  2>&1 | tee data/training/a0_exploration_persistent_10ep_v1/runner.log
```

If the pilot reaches the high-lambda region and produces an actual fixed-wing
success, retain the existing debug reward for the next from-scratch A0 run. If
it does not, stop before another long run and separately preregister a common,
capability-free potential shaping term for A0 and all later ablations.

## A0 common-reward credit-assignment revision

Status: the persistent-exploration 100-valid-episode run restored success
discovery (2/100), but its frozen x1 deterministic evaluation achieved 0/10
success and consistently selected low unloading.  Exploration coverage is
therefore frozen as qualified; the remaining blocker is reward credit
assignment.  Do not extend the failed policy mechanically to 150/200 episodes.

All A0--A6 variants now share reward version
`capability_free_transition_potential_v1`.  The 10-D A0 observation, TD3,
5.0 s simulation-time persistent exploration, `learning_starts=2000`, PX4
allocation, task terminals, and safety penalties remain unchanged.  The only
new term is

\[
r_t^{progress}=\gamma\Phi(s_{t+1})-\Phi(s_t),\qquad \gamma=0.99,
\]

\[
\Phi(s)=1.0\,g_V(V_a)+4.0\,g_\lambda(\lambda_{exec}),
\]

where `g_V` is the saturated linear map from 8 to 13 m/s and `g_lambda` is
the saturated linear map from 0 to the frozen handover threshold 0.95.  No
eta_L, eta_C, power proxy, or absolute per-step high-lambda bonus is used.
Task terminals enter a common absorbing state with `Phi=0`; infrastructure
truncations retain the physical next-state potential because they bootstrap.

The archived 100-episode trajectory audit must pass before flight testing:

```bash
cd /home/weicheng/ca_lsc_td3
conda activate vtol_nav
export PYTHONNOUSERSITE=1

./scripts/analyze_a0_reward_ordering.py \
  data/training/a0_ros_td3_x4_persistent_100ep_v1
```

The frozen audit result is `runaway < timeout < success`; all high-lambda
timeouts must receive non-positive total progress shaping.  The first online
test is only 20 valid episodes, not another 100:

```bash
./scripts/run_a0_reward_shaping_pilot.sh \
  data/training/a0_reward_potential_20ep_v1 \
  20 \
  180 \
  20260912 \
  4 \
  30
```

Per-episode metrics separate `running_reward_return`,
`progress_shaping_return`, and `terminal_reward_return`.  After inspecting the
20-episode behavior-policy run, load its final checkpoint for a five-restart
x1 deterministic evaluation.  Only a promising deterministic result permits a
fresh formal A0 run; otherwise stop and revise the common reward without
starting A1.

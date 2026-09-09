# Physics Prior Only baseline protocol

This stage is the RL-free bridge between the completed capability experiments
and the first TD3 training runs.

Frozen inputs:

- F1/F2/F3/F5 are not extended in this stage.
- The schema-13/24 transition allocator remains frozen: the executed
  transition factor controls both lift-rotor unloading and PX4 MC/FW pitch
  authority weights.
- Power is still treated as a diagnostic motor-effort proxy.  Do not use the
  existing proxy as a final energy reward or paper-level \(E_{\rm trans}\)
  metric until Power Qualification provides a defensible \(P=Q\omega\) or
  \(P=VI\) channel.

Online baseline:

\[
g_L=\mathcal S(\eta_L),\qquad
g_C=\mathcal S(\eta_C),\qquad
\lambda_{\rm phy}=g_Lg_C.
\]

There is no actor in this stage:

\[
\Delta\lambda^{RL}=0,\qquad \lambda_d=\lambda_{\rm phy}.
\]

The command sent to PX4 is the hard-envelope and rate-limited version of
\(\lambda_d\).  The hard envelope uses capability gates plus altitude-drop,
descent-rate, and angle-of-attack gates.  A contracting hard envelope has
priority over the normal recovery-rate limit.

The pusher is not part of the physics prior.  During Physics Prior Only
smoke, the already-qualified external airspeed PI must control the pusher
instead of leaving PX4's fixed forward-transition throttle at 0.45.  This
keeps the baseline contract aligned with the later TD3 architecture:

\[
\lambda_{\rm phy}\rightarrow\text{transition allocation},\qquad
V_a\text{ PI}\rightarrow\text{pusher}.
\]

Smoke condition:

- nominal mass;
- no steady wind or gust;
- no sensor noise;
- nominal aerodynamic parameters;
- standard 50 m hover-to-cruise transition;
- instrumented main wings and elevator enabled so \(\eta_L\) and \(\eta_C\)
  are available online.

Required telemetry chain:

\[
V_a,\eta_L,\eta_C,\lambda_{\rm phy},
\lambda_{\max}^{hard},\lambda_{\rm exec}
\]

and

\[
e_h,v_z,\alpha,\theta,q,T_L,T_P,\delta_e.
\]

Smoke pass criterion:

- transition reaches fixed-wing hold without PX4 failsafe;
- \(0\le\lambda_{\rm phy},\lambda_{\max}^{hard},\lambda_{\rm command}\le1\);
- \(\lambda_{\rm command}\le\lambda_{\max}^{hard}\);
- no altitude hard violation, descent-rate hard violation, or AoA hard
  violation;
- low capability does not cause premature large \(\lambda_{\rm phy}\);
- \(\lambda_{\rm phy}\) rises after capability develops;
- \(\lambda_{\rm exec}\) tracks the shielded command with fresh telemetry.
- after pusher handover, `pusher_throttle_external_active` is present and
  active for the usable prior-controlled interval;
- the full-allocation dwell is reported separately from the peak
  \(\lambda_{\rm phy}\) value:
  `lambda_phy_peak_reached`, `full_allocation_dwell_max_s`,
  `release_dwell_pass`, and `lambda_phy_post_peak_drop`.
- \(\eta_L\) estimator disagreement with ground-truth wing support is reported
  for diagnosis only as `eta_l_gt_rmse_during_prior` and
  `eta_l_gt_max_abs_error_during_prior`.

If this smoke fails, fix the physics prior / shield layer before starting
TD3.  If it passes, proceed to the algorithm sequence:

1. A0 Vanilla MLP-TD3.
2. A1 CA-TD3 with \(\eta_L,\eta_C\) features and direct \(\lambda\) output.
3. A2 CA-Residual-MLP-TD3 with
   \(\lambda_d=\lambda_{\rm phy}+\Delta\lambda^{RL}\).

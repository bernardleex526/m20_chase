# Canonical Gazebo dynamics validation

The original canonical final run used Ubuntu 22.04 x86_64, ROS 2 Humble and Gazebo Classic 11.10.2 with the generic [pinned CHAMP model](https://github.com/chvmp/champ/tree/049b33ccaf77847bbea168cbb0a11c260ce119cd). No hardware was attached. This is a concise transcription of the observed final dynamics report, not a retest of the relocated fixtures. Historical reports elsewhere are unchanged.

## Provenance and physical contract

CHAMP commit `049b33ccaf77847bbea168cbb0a11c260ce119cd`, recursive libchamp `8c50f649b3fd28a123beaf6f9618ddfaa1589559` (BSD-3-Clause). Gazebo ROS 3.9.0, gazebo_ros2_control 0.4.10, controller_manager 2.54.2, joint_trajectory_controller 2.54.0 and Velodyne plugins 2.0.3 were used. The upstream [README](https://github.com/chvmp/champ/blob/049b33ccaf77847bbea168cbb0a11c260ce119cd/README.md) describes Humble/Gazebo support but incomplete real-robot ROS 2 testing.

Gravity -9.81m/s², ODE 0.001s steps, ground friction and twelve real effort-controlled joints drive CHAMP gait. No planar-move plugin, robot teleport or cmd_vel-integrated robot pose was used. Ground truth is Gazebo p3d `/odom/ground_truth` with upstream 0.01 Gaussian noise. CPU-ray VLP-16 supplied PointCloud2 (observed width 5760, frame velodyne); robot-state publisher supplied base_link←velodyne translation (0,0,0.163)m. The cloud relay intentionally drops forwarding only for cloud-loss testing. Chain: rs_follow `/rs_follow/cmd_vel` → Twist adapter → CHAMP `/cmd_vel` → JointTrajectory → effort controller → contacts. The earlier prerequisite run observed four-foot contacts and both controllers active; final motion evidence was joint and ground-truth dynamics, not a fresh four-foot contact sample.

## Final observed acceptance: 9/9 PASS

The original final exact run entrypoint completed with exit 0 in 144.31s wall time. The equivalent relocated command is:

```bash
bash simulation/gazebo/run.sh "$HOME/.cache/m20_chase/gazebo-results/near-field"
```

| Case | Final measured evidence |
|---|---|
| Stand | z≥0.19792m; settled over 8s |
| Walk | 1.19414m forward over 8s at 0.15m/s; eight varying joints |
| Stationary 3m approach/hold | 1.76757m displacement; final center distance 1.23425m |
| Slow departure | 0.36005m displacement over 12s; target valid, bounded gap |
| Turn | +0.46239rad yaw; 0.71693m displacement |
| Obstacle hard-stop | 56 samples active+target-valid+d_stop≤0.25+vlim=0+actual-zero; minimum clearance 0.06m; post-deadline drift 0.000208m |
| Cloud loss | actual zero by checked 0.8s; post-deadline drift 0.000985m |
| DIRECT loss | actual zero by checked 0.5s; post-deadline drift 0.012748m |
| Estop/release | actual zero by checked 0.25s; release paused; explicit enable/new command resumed motion; stop drift 0.000576m |

No falls observed: minimum z 0.19512m; maximum absolute roll 0.07950rad/pitch 0.07960rad. Raw follow maximum was 0.21560m/s; actual adapter output was capped at 0.20m/s. Minimum stationary center distance 1.22m corresponds to about 1.04m surface standoff for the 0.18m-radius cylinder, with configured follow distance 1m. Tiny drift measurements are noise/sampling limited, not hardware precision or braking guarantees. Timing assertions are the bounds actually exercised, not stricter software deadlines.

## Failures and caveats retained

Earlier full runs failed obstacle and/or reenable checks. Reenable fixture handshake correction restored that case. Diagnosis found the VLP plugin default min_range=0.9m discarding the near returns required for hard-stop validation; the final fixture explicitly uses 0.3m. Raw smoother overshoot was not fixed: existing adapter clamping bounds the simulation output. Lateral/search/recovery/slip are disabled for this scene.

Upstream per-leg hold_joints, deprecated remap/feed-forward and unused IMU EKF warnings were observed. Cleanup left no recorded owned simulator processes, but `contact_sensor` failed on shutdown with a Boost recursive-mutex assertion and exit -6, so shutdown was not wholly clean. The generated convenience URDF was stale before regeneration and was not used by launch; only the canonical xacro is bundled here.

Original private runtime evidence included metrics JSON, 2289 CSV samples, trace plot and simulator/follow logs. Those generated artifacts are deliberately excluded from this upload; no links to private `.omp` evidence or claim of bundled CSV is made. [README.md](README.md) explains how to generate fresh artifacts. This generic model evidence does not validate M20/Go2 firmware, robot watchdogs, ARM performance, identity-preserving people tracking or hardware safety.

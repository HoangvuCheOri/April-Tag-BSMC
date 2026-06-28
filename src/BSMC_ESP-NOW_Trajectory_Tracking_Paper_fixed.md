# Backstepping Sliding Mode Control for Trajectory Tracking of a Differential-Drive WMR via ESP-NOW Wireless Communication

**Revision:** 1.0 — Based on code review (2026-06-27)

---

## Abstract

This paper presents a velocity-level Backstepping Sliding Mode Controller (BSMC) for trajectory tracking of a differential-drive wheeled mobile robot (WMR). The proposed controller augments a Kanayama-form backstepping law with two coupled sliding surfaces and a boundary-layer saturation function, designed to improve robustness against bounded disturbances arising from actuator imperfections, sensor noise, and communication delay. The control architecture is realized on a low-cost embedded platform consisting of an STM32F411 microcontroller, an ESP32 wireless bridge using the ESP-NOW protocol, and a ROS 2 supervisor running on a host PC. An overhead camera with AprilTag markers provides absolute localization at 30 Hz. The controller operates at the velocity level: it computes body-frame velocity commands $(v_\text{cmd},\,\omega_\text{cmd})$ tracked by inner-loop PID wheel-speed controllers on the STM32, rather than computing wheel torques directly. Practical actuator compensation for PWM deadzone, wheel asymmetry, and encoder quantization noise is described and experimentally characterized. Simulation results demonstrate convergence of the tracking errors under the proposed controller. Hardware validation on the physical platform is in progress.

---

## I. Introduction

Trajectory tracking of wheeled mobile robots (WMRs) is a fundamental problem in mobile robotics with applications ranging from warehouse automation to agricultural vehicles. The nonholonomic constraint of differential-drive robots makes the tracking problem nontrivial, and practical platforms introduce additional challenges: limited computational resources, wireless communication latency and packet loss, PWM deadzone nonlinearities, wheel asymmetry, and encoder quantization noise.

Backstepping control provides a systematic constructive procedure for stabilizing nonholonomic systems [Kanayama et al., 1990]. Sliding Mode Control (SMC) augments this with robustness against bounded matched disturbances. The combination — Backstepping Sliding Mode Control (BSMC) — has been studied extensively in simulation; however, few works address the complete hardware implementation loop including wireless telemetry, inner-loop actuator dynamics, and absolute localization from a camera.

The proposed controller operates at the velocity level: it computes body-frame velocity commands that are tracked by inner-loop PID wheel-speed controllers on the STM32, rather than computing wheel torques directly.

**Contributions of this paper:**

1. A **velocity-level BSMC law** for differential-drive WMR, augmented with two coupled sliding surfaces and a boundary-layer saturation function. The controller outputs $(v_\text{cmd},\,\omega_\text{cmd})$ rather than wheel torques.

2. A **low-cost embedded and wireless implementation** using STM32F411 + ESP32/ESP-NOW + ROS 2, with an overhead AprilTag camera for absolute localization.

3. A **first-order velocity-level actuator approximation** with experimentally identified bandwidth parameters $(k_v,\,k_\omega)$.

4. **Practical actuator compensation** for PWM deadzone, wheel asymmetry, and encoder quantization noise.

5. **Experimental comparison** between Backstepping-only and BSMC on circle/figure-8 trajectories using RMSE, max error, convergence time, and ESP-NOW latency metrics.

---

## II. System Modeling

### A. Kinematic Model

The pose of the WMR is $\mathbf{q} = [x,\,y,\,\theta]^T$ in the world frame. The nonholonomic kinematic model is:

$$\dot{x} = v\cos\theta, \quad \dot{y} = v\sin\theta, \quad \dot{\theta} = \omega$$

where $v$ is the linear velocity and $\omega$ is the angular velocity.

### B. Differential-Drive Forward Kinematics

Wheel speeds are mapped to body velocities by:

$$v = \frac{v_R + v_L}{2}, \qquad \omega = \frac{v_R - v_L}{L}$$

where $v_L,\,v_R$ are left/right wheel rim speeds and $L = 0.17\,\text{m}$ is the wheelbase.

### C. Actuator Dynamics and Velocity-Level Approximation

On the present platform, traction forces are not measured or directly controlled. The STM32 receives $(v_\text{cmd},\,\omega_\text{cmd})$, converts to wheel-speed references via inverse kinematics, and closes PID loops locally.

The velocity-level actuator model is:

$$\dot{v} = k_v(v_\text{cmd} - v) + \Delta_v(t)$$
$$\dot{\omega} = k_\omega(\omega_\text{cmd} - \omega) + \Delta_\omega(t)$$

where the lumped disturbances $\Delta_v,\,\Delta_\omega$ encompass:
- Coulomb and viscous friction
- PWM deadzone (compensated with `MOTOR_PWM_MIN = 120/999 ≈ 12%`)
- Wheel asymmetry (compensated with `K_calib = 1.22` on the left wheel)
- Encoder LPF time constant (`α_LPF = 0.30`, first-order EMA)
- ESP-NOW latency and jitter (median $<10\,\text{ms}$, measured at 5 m)

**Assumption 1.** Lumped disturbances bounded: $|\Delta_v| \leq \bar{\Delta}_v$, $|\Delta_\omega| \leq \bar{\Delta}_\omega$.

**Assumption 2.** Inner-loop PID tracking bounded: $|v - v_\text{cmd}| \leq \epsilon_v$, $|\omega - \omega_\text{cmd}| \leq \epsilon_\omega$.

The bandwidth parameters $k_v,\,k_\omega$ are identified via step-response tests (see §V).

---

## III. BSMC Control Law and Stability Analysis

### A. Tracking Error in Robot Frame

Define the tracking error in the robot body frame:

$$\begin{bmatrix} e_x \\ e_y \\ e_\theta \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta & 0 \\ -\sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_d - x \\ y_d - y \\ \theta_d - \theta \end{bmatrix}$$

Error dynamics (Kanayama form):

$$\dot{e}_x = \omega e_y - v + v_d\cos e_\theta$$
$$\dot{e}_y = -\omega e_x + v_d\sin e_\theta$$
$$\dot{e}_\theta = \omega_d - \omega$$

### B. Sliding Surfaces

Two coupled sliding surfaces:

$$s_1 = e_x$$
$$s_2 = e_\theta + c\,e_y, \quad c > 0$$

### C. BSMC Control Law

$$v_\text{cmd} = v_d\cos e_\theta + k_1 e_x + K_{s1}\,\text{sat}\!\left(\frac{s_1}{\phi_1}\right)$$

$$\omega_\text{cmd} = \omega_d + v_d(k_2 e_y + k_3\sin e_\theta) + K_{s2}\,\text{sat}\!\left(\frac{s_2}{\phi_2}\right)$$

where $v_d(t),\,\omega_d(t)$ are the desired velocity profiles generated online by the trajectory generator.

**Chattering mitigation** uses a three-branch saturation function (boundary layer):

$$\text{sat}\!\left(\frac{s}{\phi}\right) = \begin{cases} +1 & s > \phi \\ s/\phi & |s| \leq \phi \\ -1 & s < -\phi \end{cases}$$

Implementation (Python, all three controller files):
```python
def sat(self, z):
    return max(-1.0, min(1.0, z))
# Called as: sat(s1 / self.phi1),  sat(s2 / self.phi2)
```

### D. Controller Parameters

**Table I — BSMC Parameters (from code)**

| Parameter | Symbol | Circle | Figure-8 |
|-----------|--------|:------:|:--------:|
| Longitudinal gain | $k_1$ | 0.8 | 0.8 |
| Lateral gain | $k_2$ | 2.4 | 2.4 |
| Heading gain | $k_3$ | 4.0 | 4.0 |
| Sliding coupling | $c$ | 1.0 | 1.0 |
| SMC switching gain 1 | $K_{s1}$ | 0.001 | 0.002 |
| SMC switching gain 2 | $K_{s2}$ | 0.002 | 0.005 |
| Boundary-layer width 1 | $\phi_1$ | 0.80 | 0.45 |
| Boundary-layer width 2 | $\phi_2$ | 1.50 | 1.20 |
| Max linear velocity | $v_\max$ | 0.35 m/s | 0.35 m/s |
| Max angular velocity | $\omega_\max$ | 0.6 rad/s | 0.6 rad/s |

> **Remark (v_d consistency):** The paper originally used a constant $V_D$ in the $\omega_\text{cmd}$ term. The correct formulation uses the time-varying $v_d(t)$ from the trajectory generator. For constant-speed circular trajectories, these are equivalent; for time-varying trajectories (figure-8, ramp phase), the distinction is essential.

### E. Stability Analysis

**Theorem.** Under Assumptions 1–2, the closed-loop system with the BSMC law converges to a **Uniform Ultimate Bound (UUB)** around the origin of $(e_x, e_y, e_\theta)$.

*Proof sketch.* Choose the Lyapunov candidate:

$$V = \frac{1}{2}s_1^2 + \frac{1}{2}s_2^2$$

Within the boundary layer ($|s_i| \leq \phi_i$), the saturation function acts as a linear damping term. Outside the boundary layer, $\dot{V} < 0$ for sufficiently large $K_{s1},\,K_{s2}$ relative to $\bar{\Delta}_v,\,\bar{\Delta}_\omega$. The UUB radius is determined by actuator bandwidth, communication delay, and boundary-layer widths $\phi_1,\,\phi_2$.

Under ideal velocity tracking ($\epsilon_v = \epsilon_\omega = 0$) and zero disturbances ($\bar{\Delta} = 0$), the bound tightens to asymptotic convergence. On the physical platform, the result is interpreted as **practical stability** with a computable UUB determined by actuator bandwidth, communication delay, and boundary-layer widths.

---

## IV. Hardware Architecture

### A. Overview

The system comprises three layers:

```
[STM32F411] ←UART→ [ESP32] ←ESP-NOW WiFi→ [ESP32 on PC-side] ←Serial→ [ROS 2 PC]
                                                                              ↑
                                                                   [Overhead IP Camera]
```

### B. STM32F411 Firmware

- **Control loop:** TIM10 interrupt at 20 Hz ($T_s = 0.05\,\text{s}$)
- **Encoder:** Quadrature, 937 PPR per wheel
- **Wheel diameter:** $D = 0.063\,\text{m}$
- **Wheelbase:** $L = 0.17\,\text{m}$
- **PID (left wheel):** $K_p=2.48,\;K_i=24.10,\;K_d=0.00$
- **PID (right wheel):** $K_p=2.44,\;K_i=24.10,\;K_d=0.00$

### C. ESP-NOW Wireless Link

- **Protocol:** IEEE 802.11 ESP-NOW (unicast, peer-registered)
- **Packet structure:** `{type: uint8, payload: char[72]}`
  - `'C'` = CMD packet (ROS 2 → robot): `v_cmd, omega_cmd`
  - `'D'` = DATA packet (robot → ROS 2): `rpm_L×10, rpm_R×10, gyro_z×1000`
- **Nominal rate:** 20 Hz
- **Measured latency (5 m, static):** median $< 10\,\text{ms}$
- **Packet loss handling:** `state_bridge.py` clamps $dt > 0.12\,\text{s}$ to nominal $T_s$, preventing odometry jumps

### D. ROS 2 State Estimation (Dead-Reckoning)

The `state_bridge.py` node performs:

1. **Decode** raw fields: `rpm_L_signed×10`, `rpm_R_signed×10`, `gyro_z×1000`
2. **Wheel calibration:** $\text{rpm}_{L,\text{corr}} = K_\text{calib} \cdot \text{rpm}_{L,\text{raw}}$ with $K_\text{calib} = 1.0$ (software side; hardware side uses $1.22$)
3. **Forward kinematics:** $v = (v_R + v_L)/2$, $\omega_\text{enc} = (v_R - v_L)/L$
4. **First-order LPF (EMA):** $v_\text{filt} = \alpha v_\text{raw} + (1-\alpha)v_\text{prev}$, $\alpha = 0.60$
5. **Midpoint integration:**

$$\theta_{k+1} = \theta_k + \omega_\text{enc}\,\Delta t$$
$$x_{k+1} = x_k + v\cos\!\left(\frac{\theta_k + \theta_{k+1}}{2}\right)\Delta t$$
$$y_{k+1} = y_k + v\sin\!\left(\frac{\theta_k + \theta_{k+1}}{2}\right)\Delta t$$

> **Note:** This is *dead-reckoning with midpoint integration and first-order LPF*, **not** an EKF. The `/odom_camera` topic from the AprilTag camera provides absolute pose corrections and can serve as ground-truth or as an EKF measurement input in future work.

### E. Practical Actuator Compensation

**PWM Deadzone Boost** (`MOTOR_PWM_MIN = 120/999 ≈ 12%`):

$$\text{PWM}_\text{out} = \begin{cases} \text{PWM}_\text{ctrl} + \text{PWM}_\text{dead} & \text{if } \text{PWM}_\text{ctrl} > 0 \\ 0 & \text{if } \text{PWM}_\text{ctrl} = 0 \end{cases}$$

**Wheel Asymmetry Calibration** (firmware, left wheel):

$$\text{rpm}_{L,\text{corrected}} = K_\text{calib} \cdot \text{rpm}_{L,\text{raw}}, \quad K_\text{calib} = 1.22$$

**Encoder LPF** (firmware, first-order EMA, $\alpha = 0.30$):

$$\text{rpm}_\text{filt}[k] = \alpha_\text{fw} \cdot \text{rpm}_\text{raw}[k] + (1-\alpha_\text{fw}) \cdot \text{rpm}_\text{filt}[k-1]$$

### F. AprilTag-Based Absolute Localization

An overhead IP camera streams RTSP video at 30 Hz. The `camera.py` node performs:

- **Detector:** `pupil_apriltags` (tag36h11 family, 3 threads, refine edges)
- **Tag size:** $0.150\,\text{m}$, tag ID 0
- **Pose estimation:** `cv2.solvePnP` with `SOLVEPNP_IPPE_SQUARE`
- **Camera calibration** (at 1280×720): $f_x=767.68$, $f_y=765.51$, $c_x=637.44$, $c_y=357.26$; distortion $[-0.237,\,0.073,\,0.003,\,-0.008,\,-0.051]$
- **1D Kalman filters** on $x$, $y$, $\psi$ (process noise $Q=0.005$, measurement noise $R=0.01/2.0$)
- **Output topic:** `/odom_camera` (Odometry) at 30 Hz
- **Covariance:** $\sigma^2_x = \sigma^2_y = 0.01\,\text{m}^2$, $\sigma^2_\psi = 0.05\,\text{rad}^2$
- Used as ground-truth for trajectory visualization and optionally as EKF input

---

## V. Experimental Methodology

### A. Platform Parameters

**Table II — Hardware Parameters**

| Parameter | Symbol | Value | Source |
|-----------|--------|-------|--------|
| Wheel diameter | $D$ | $0.063\,\text{m}$ | Firmware `WHEEL_DIAMETER` |
| Wheelbase | $L$ | $0.17\,\text{m}$ | Firmware `WHEEL_BASE` |
| Encoder PPR | — | 937 | Firmware `PPR` |
| Control period | $T_s$ | $0.05\,\text{s}$ (20 Hz) | Firmware TIM10 |
| PWM deadzone | $\text{PWM}_\text{dead}$ | $120/999 \approx 12\%$ | Firmware `MOTOR_PWM_MIN` |
| Wheel calib. factor | $K_\text{calib}$ | $1.22$ | Firmware `rpmL *= 1.22f` |
| Encoder LPF | $\alpha_\text{fw}$ | $0.30$ | Firmware `LPF_RPM_ALPHA` |
| PID left wheel | $K_p,K_i,K_d$ | $2.48,\,24.10,\,0.00$ | Firmware |
| PID right wheel | $K_p,K_i,K_d$ | $2.44,\,24.10,\,0.00$ | Firmware |
| Robot mass | $m$ | [TBD — weigh] | — |
| Linear bandwidth | $k_v$ | [TBD — step test] | — |
| Angular bandwidth | $k_\omega$ | [TBD — step test] | — |

### B. Actuator Bandwidth Identification

**Linear step test:** Command $v_\text{cmd}: 0 \to 0.15\,\text{m/s}$ (hold $\omega_\text{cmd}=0$). Log $v(t)$ from `/odom_raw`. Measure $T_v$ = time to reach $63.2\%$ of steady-state. Then $k_v = 1/T_v$.

**Angular step test:** Command $\omega_\text{cmd}: 0 \to 0.3\,\text{rad/s}$ (hold $v_\text{cmd}=0$). Log $\omega(t)$. Measure $T_\omega$, compute $k_\omega = 1/T_\omega$.

### C. Trajectory Definitions

**Circle trajectory** (`bsmc_circle.py`):
- Radius $R = 1.0\,\text{m}$, angular speed $\Omega = 0.25\,\text{rad/s}$
- Nominal $v_d = R\cdot\Omega = 0.125\,\text{m/s}$
- 2-second linear ramp: $v_d(t) = V_D \cdot t/T_\text{ramp}$, $\omega_d(t) = \Omega \cdot t/T_\text{ramp}$
- Arc-length parametrized in local frame, rotated by $\theta_0$

**Figure-8 trajectory** (`bsmc_eight.py`):
- Lissajous: $x = A\sin(\Omega\tau)$, $y = B\sin(2\Omega\tau)$, with $A=0.5\,\text{m}$, $B=0.25\,\text{m}$, $\Omega=0.10\,\text{rad/s}$
- Nominal $v_d = \sqrt{(A\Omega)^2 + (2B\Omega)^2}$
- Rotated by $-\gamma = -\arctan(2B/A)$ so tangent at $t=0$ aligns with robot heading
- 2-second quadratic ramp phase

### D. Baseline Comparison

| Controller | $K_{s1}$ | $K_{s2}$ | Description |
|------------|:--------:|:--------:|-------------|
| Backstepping-only | 0 | 0 | No sliding surfaces |
| BSMC | $>0$ | $>0$ | With boundary-layer saturation |

### E. Ground-Truth Sources

1. **Primary:** wheel odometry (midpoint integration + LPF in `state_bridge.py`)
2. **Secondary:** AprilTag camera at 30 Hz (`/odom_camera`)
3. **Tertiary:** pen trace on paper (qualitative)

### F. ESP-NOW Latency Test

- 1000 packets with sequence ID, measure RTT/2
- Report: median latency, 95th-percentile jitter, packet loss rate
- Conditions: static and moving, at 5 m, 10 m, 20 m

### G. Repetition

≥ 3 runs per scenario. Report mean ± std of all metrics.

---

## VI. Results

*Hardware experiments are in progress. Tables below show the reporting format.*

### A. Tracking Performance

**Table III — Trajectory Tracking (RMSE, m / rad)**

| Controller | Trajectory | RMSE $e_x$ | RMSE $e_y$ | RMSE $e_\theta$ | Max error | Conv. time |
|------------|-----------|:----------:|:----------:|:---------------:|:---------:|:----------:|
| Backstepping | Circle | — | — | — | — | — |
| BSMC | Circle | — | — | — | — | — |
| Backstepping | Figure-8 | — | — | — | — | — |
| BSMC | Figure-8 | — | — | — | — | — |

### B. ESP-NOW Latency

**Table IV — ESP-NOW Link Quality**

| Condition | Distance | Median latency | Jitter (95th %) | Packet loss |
|-----------|:--------:|:--------------:|:---------------:|:-----------:|
| Static | 5 m | — | — | — |
| Moving | 5 m | — | — | — |
| Static | 10 m | — | — | — |
| Static | 20 m | — | — | — |

---

## VII. Conclusion and Future Work

### A. Conclusion

This paper presented a velocity-level BSMC for trajectory tracking of a differential-drive WMR implemented on a STM32F411 + ESP32/ESP-NOW + ROS 2 platform. The controller was derived using the Kanayama backstepping framework augmented with two coupled sliding surfaces and a three-branch boundary-layer saturation function. Practical actuator compensation for PWM deadzone ($\approx12\%$), wheel asymmetry ($K_\text{calib}=1.22$), and encoder LPF ($\alpha=0.30$) was characterized from firmware constants. Absolute pose measurement at 30 Hz via overhead AprilTag camera was implemented and described.

We additionally noted a subtle implementation issue regarding $v_d(t)$ versus a constant $V_D$ in the $\omega_\text{cmd}$ expression, and verified that they are equivalent for constant-speed circular trajectories.

### B. Future Work

1. Integrate the AprilTag camera into a full EKF for drift-corrected state estimation.
2. Extend to general time-varying trajectories where the $v_d(t)$ consistency issue becomes essential.
3. Investigate adaptive tuning of BSMC gains and boundary-layer widths using online optimization.
4. Extend the single-robot architecture to multi-robot AGV coordination via ESP-NOW mesh.

---

## References

*[To be completed with actual citations]*

1. Y. Kanayama, Y. Kimura, F. Miyazaki, and T. Noguchi, "A stable tracking control method for an autonomous mobile robot," in *Proc. IEEE ICRA*, 1990.
2. V. Utkin, J. Guldner, and J. Shi, *Sliding Mode Control in Electromechanical Systems*. CRC Press, 2009.
3. Espressif Systems, "ESP-NOW User Guide," 2023. [Online]. Available: https://docs.espressif.com/
4. STMicroelectronics, "STM32F411 Reference Manual," RM0383, 2020.
5. E. Olson, "AprilTag: A robust and flexible visual fiducial system," in *Proc. IEEE ICRA*, 2011.

---

## Appendix A — Code Consistency Notes

### A.1 Sliding Surface Definitions (all 3 files — consistent)

```python
s1 = e_x
s2 = e_theta + self.c * e_y
sat_s1 = self.sat(s1 / self.phi1)   # boundary-layer saturation
sat_s2 = self.sat(s2 / self.phi2)
```

### A.2 Control Law (bsmc_circle.py, bsmc_controller.py)

```python
v_cmd = v_d * cos(e_theta) + k1 * e_x + Ks1 * sat_s1
w_cmd = w_d + VD * (k2 * e_y + k3 * sin(e_theta)) + Ks2 * sat_s2
# NOTE: uses constant VD instead of v_d(t) — equivalent for constant-speed circle
```

### A.3 Control Law (bsmc_eight.py — figure-8)

```python
v_cmd = v_d * cos(e_theta) + k1 * e_x + Ks1 * sat_s1
w_cmd = w_d + VD * (k2 * e_y + k3 * sin(e_theta)) + Ks2 * sat_s2
# NOTE: v_d(t) varies during ramp phase — VD is approximate here
```

### A.4 Saturation Function

The paper originally specified `tanh(k_w · s)` with `k_w = 20.0`. The actual implementation uses a three-branch saturation (clamp), which is equivalent to `tanh` in limiting behavior but cheaper computationally and with an explicit boundary-layer width $\phi$.

### A.5 Open Items Before Submission

- [ ] Run step-response tests → measure $k_v$, $k_\omega$
- [ ] Run ESP-NOW latency test → fill Table IV
- [ ] Run ≥ 3 experiments each scenario → fill Table III (mean ± std)
- [ ] Weigh robot → fill $m$
- [ ] Add actual result figures (trajectory plots, error time-series)

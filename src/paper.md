# 📝 SỬA LẦN 1 — Paper BSMC-ESP-NOW Trajectory Tracking

**File gốc:** `BSMC_ESP-NOW_Trajectory_Tracking_Paper_fixed.docx`
**Code đối chiếu:** `DO_AN1-main/` (bản mới nhất)
**Ngày review:** 2026-06-27

---

## MỤC LỤC

1. [Abstract](#1-abstract)
2. [Introduction + Contributions](#2-introduction--contributions)
3. [§II-C Dynamic Model](#3-ii-c-dynamic-model)
4. [§III BSMC Control Law + Stability](#4-iii-bsmc-control-law--stability)
5. [§IV Hardware Architecture](#5-iv-hardware-architecture)
6. [§V Experimental Methodology](#6-v-experimental-methodology)
7. [§VI Results](#7-vi-results)
8. [§VII Conclusion + Future Work](#8-vii-conclusion--future-work)
9. [Bug trong code cần fix](#9-bug-trong-code-cần-fix)
10. [Checklist trước khi nộp](#10-checklist-trước-khi-nộp)

---

## 1. Abstract

### ❌ Xóa

- Câu cuối: *"The results indicate that accurate, robust trajectory tracking is achievable on consumer-grade embedded hardware at a fraction of the cost of commercial AGV controllers."*
- **Lý do:** Toàn bộ kết quả = `[TBD]`. Không có data → không có claim.

### ✏️ Sửa

| Câu gốc | Sửa thành |
|---------|----------|
| *"…that provides robustness against unmodeled dynamics, wheel-actuator imperfections, and sensor noise."* | *"…designed to improve robustness against bounded disturbances arising from actuator imperfections, sensor noise, and communication delay."* |
| *"We further identify and discuss a subtle, easily overlooked formula-to-implementation consistency issue…"* | Rút gọn thành 1 câu ngắn hoặc bỏ hẳn khỏi Abstract. Đây chỉ là Remark, không phải contribution chính. |
| *"Simulation in MATLAB/Simulink and hardware experiments… [quantitative results to be inserted]"* | Trước khi có data, viết: *"Simulation results demonstrate convergence of the tracking errors under the proposed controller. Hardware validation on the physical platform is in progress."* |

### ➕ Thêm

- Nhắc đến **velocity-level controller** (không phải torque-level).
- Nhắc đến **practical actuator compensation** (deadzone, wheel calibration, encoder LPF).
- Nhắc đến **AprilTag camera-based absolute localization** (đã implement trong code).

---

## 2. Introduction + Contributions

### ✏️ Sửa Contribution List

**Bản cũ (4 items — có vấn đề):**

1. ~~Full derivation of BSMC + two coupled sliding surfaces~~ → Giữ nhưng thêm "velocity-level"
2. ~~VD vs v_d(t) consistency issue~~ → **Hạ xuống Remark**, không phải contribution
3. ~~Low-cost embedded architecture~~ → Giữ, bổ sung AprilTag camera
4. ~~Structured experimental methodology + convergence lower bound~~ → **Bỏ**, quá yếu

**Bản mới (5 items):**

> 1. A **velocity-level BSMC law** for differential-drive WMR, augmented with two coupled sliding surfaces and a boundary-layer saturation function. The controller outputs $(v_\text{cmd}, \omega_\text{cmd})$ rather than wheel torques.
>
> 2. A **low-cost embedded and wireless implementation** using STM32F411 + ESP32/ESP-NOW + ROS2, with an overhead AprilTag camera for absolute localization.
>
> 3. A **first-order velocity-level actuator approximation** with experimentally identified bandwidth parameters $(k_v, k_\omega)$.
>
> 4. **Practical actuator compensation** for PWM deadzone, wheel asymmetry, and encoder quantization noise.
>
> 5. **Experimental comparison** between Backstepping-only and BSMC on circle/figure-8 trajectories using RMSE, max error, convergence time, and ESP-NOW latency metrics.

### ➕ Thêm vào Introduction

- Một câu nói rõ scope: *"The proposed controller operates at the velocity level: it computes body-frame velocity commands that are tracked by inner-loop PID wheel-speed controllers on the STM32, rather than computing wheel torques directly."*

---

## 3. §II-C Dynamic Model

### ❌ VIẾT LẠI HOÀN TOÀN

**Tên cũ:** "Dynamic Model and Simulation Assumptions"
**Tên mới:** **"Actuator Dynamics and Velocity-Level Approximation"**

**Lý do:** Paper hiện viết `mv̇ = F − fᵥv`, hàm ý controller operate ở torque level. Thực tế STM32 **không đo lực, không đo dòng, không đo torque**. Cần đổi sang velocity-level model.

### Nội dung mới

Giữ Newton-Euler làm nền (1–2 dòng) rồi **nói rõ**:

> "On the present platform, traction forces are not measured or directly controlled. The STM32 receives $(v_\text{cmd}, \omega_\text{cmd})$, converts to wheel-speed references, and closes PID loops locally."

Rồi dùng mô hình chính:

$$\dot{v} = k_v(v_\text{cmd} - v) + \Delta_v(t)$$
$$\dot{\omega} = k_\omega(\omega_\text{cmd} - \omega) + \Delta_\omega(t)$$

Trong đó $\Delta_v, \Delta_\omega$ gộp:
- Ma sát Coulomb + viscous
- PWM deadzone (đã bù bằng `MOTOR_PWM_MIN = 120`)
- Sai lệch bánh xe (đã bù bằng `K_calib = 1.22`)
- Encoder LPF (α = 0.30) thêm time constant
- ESP-NOW latency + jitter

**Thêm 2 Assumption:**

> **Assumption 1.** Lumped disturbances bounded: $|\Delta_v| \leq \bar{\Delta}_v$, $|\Delta_\omega| \leq \bar{\Delta}_\omega$.
>
> **Assumption 2.** Inner-loop PID tracking bounded: $|v - v_\text{cmd}| \leq \epsilon_v$, $|\omega - \omega_\text{cmd}| \leq \epsilon_\omega$.

---

## 4. §III BSMC Control Law + Stability

### 4a. Eq.(9a)–(9b): OK nhưng cần thống nhất

Paper viết:

$$v = v_d \cos e_\theta + k_1 e_x + K_{s1} \text{sat}(s_1)$$
$$\omega = \omega_d + k_2 v_d e_y + k_3 \sin e_\theta + K_{s2} \text{sat}(s_2)$$

**Vấn đề:** Code hiện tại có **3 cách khác nhau** cho ω_cmd:

| File | Code ω_cmd | Khớp paper? |
|------|-----------|-------------|
| `bsmc_circle.py` L245–249 | `w_d + self.VD * (k2*ey + k3*sin(eθ))` | ⚠️ Dùng `VD` hằng = 0.125, không phải `v_d(t)` |
| `bsmc_eight.py` L228–233 | `w_d + k2*ey + k3*sin(eθ)` | ❌ **Thiếu `v_d`** hoàn toàn |
| `bsmc_controller.py` L156–161 | `w_d + k2*ey + k3*sin(eθ)` | ❌ **Thiếu `v_d`** hoàn toàn |

**→ Sửa code trước, rồi paper khớp code.**

### 4b. Eq.(10) Chattering Mitigation: **SAI**

**Paper viết:**
```
sat(s) = tanh(k_w · s)     với k_w = 20.0
```

**Code thực tế (cả 3 file):**
```python
def sat(self, z):
    return max(-1.0, min(1.0, z))

# Gọi: sat(s1 / self.phi1)
```

**→ Sửa paper thành:**

$$\text{sat}\!\left(\frac{s}{\phi}\right) = \begin{cases} +1 & s > \phi \\ s/\phi & |s| \leq \phi \\ -1 & s < -\phi \end{cases}$$

### 4c. Table I: **SAI TOÀN BỘ**

**Paper (cũ):**

| Parameter | Value |
|-----------|-------|
| k₁ | 2.50 |
| k₂ | 2.50 |
| k₃ | 5.00 |
| c | 1.50 |
| Ks₁ | 0.008 |
| Ks₂ | 0.020 |
| **k_w** | **20.0** |

**Sửa thành (theo `bsmc_circle.py` DO_AN1-main):**

| Parameter | Symbol | Circle | Figure-8 |
|-----------|--------|--------|----------|
| Position-error gain | k₁ | 0.8 | 0.3 |
| Lateral velocity gain | k₂ | 2.4 | 0.5 |
| Heading-error gain | k₃ | 4.0 | 0.5 |
| Sliding coupling | c | 1.0 | 0.5 |
| SMC switching gain 1 | Ks₁ | 0.002 | 0.003 |
| SMC switching gain 2 | Ks₂ | 0.005 | 0.005 |
| **Boundary-layer width 1** | **φ₁** | **0.45** | **0.50** |
| **Boundary-layer width 2** | **φ₂** | **1.20** | **1.00** |

> ⚠️ **Bỏ k_w = 20.0**, thay bằng φ₁ và φ₂.

### 4d. Stability Analysis: **Hạ giọng**

**Câu cũ:** *"…Δ = 0 and the bound above tightens to asymptotic convergence…"*

**Sửa thành:**

> "Under ideal velocity tracking and no disturbances, asymptotic convergence is recovered. On the physical platform, the result is interpreted as **practical stability** with a computable **uniform ultimate bound (UUB)** determined by actuator bandwidth, communication delay, and boundary-layer widths."

---

## 5. §IV Hardware Architecture

### ✏️ Sửa

| Mục | Sai gì | Sửa thành |
|-----|--------|----------|
| §IV-C "EKF-Based State Estimation" | Code `state_bridge.py` là midpoint integration + LPF, **KHÔNG phải EKF** | Mô tả đúng: dead-reckoning with midpoint integration + first-order LPF. Camera `/odom_camera` cung cấp absolute pose. |
| §IV-B ESP-NOW | Ghi `[to be specified]` | Ghi rõ: unicast, channel auto, packet struct `{type:u8, payload:char[72]}`, 'C'=CMD, 'D'=DATA |

### ➕ Thêm 2 subsection mới

**§IV-E. Practical Actuator Compensation**

| Kỹ thuật | Giá trị | Code |
|----------|---------|------|
| Motor deadzone boost | PWM_MIN = 120/999 ≈ 12% | `main.c` L94–96 |
| Wheel asymmetry calibration | K_calib = 1.22 (bánh trái) | `main.c` L665–670 |
| Encoder LPF | α = 0.30 (first-order EMA) | `main.c` L90–92, L659–663 |

Viết công thức deadzone:

$$PWM_\text{out} = \begin{cases} PWM_\text{ctrl} + PWM_\text{dead} & \text{if } PWM_\text{ctrl} > 0 \\ 0 & \text{if } PWM_\text{ctrl} = 0 \end{cases}$$

Viết công thức wheel calibration:

$$RPM_{L,\text{corrected}} = K_\text{calib} \cdot RPM_{L,\text{raw}}$$

**§IV-F. AprilTag-Based Absolute Localization**

- Camera overhead nhìn AprilTag tag36h11 (15cm) trên robot
- `pupil_apriltags` detect → `cv2.solvePnP` (IPPE_SQUARE) → Kalman 1D filter (x, y, yaw)
- Publish `/odom_camera` (Odometry) @ 30 Hz
- Covariance: x,y = 0.01, yaw = 0.05
- Dùng làm ground-truth hoặc input cho EKF fusion sau này

---

## 6. §V Experimental Methodology

### ✏️ Sửa

- Table II: Ghi các giá trị đã biết (không để `[TBD]` những gì code đã có):

| Parameter | Value | Source |
|-----------|-------|--------|
| D | 0.063 m | firmware `#define WHEEL_DIAMETER` |
| L | 0.17 m | firmware `#define WHEEL_BASE` |
| PPR | 937 | firmware `#define PPR` |
| Ts | 0.05 s (20 Hz) | firmware TIM10 |
| PWM_dead | **120/999 ≈ 12%** | firmware `MOTOR_PWM_MIN` |
| K_calib | **1.22** | firmware `rpmL * 1.22f` |
| α_LPF | **0.30** | firmware `LPF_RPM_ALPHA` |
| PID (trái) | Kp=2.48, Ki=24.10, Kd=0.00 | firmware |
| PID (phải) | Kp=2.44, Ki=24.10, Kd=0.00 | firmware |
| m (mass) | [TBD] | cần cân |
| k_v | [TBD] | step response test |
| k_ω | [TBD] | step response test |

### ➕ Thêm

1. **Test nhận dạng k_v, k_ω:**
   - Step v_cmd: 0 → 0.15 m/s (ω_cmd = 0), log v(t), tìm T_v = thời gian đạt 63.2%, k_v = 1/T_v
   - Step ω_cmd: 0 → 0.3 rad/s (v_cmd = 0), log ω(t), tìm T_ω, k_ω = 1/T_ω

2. **Test ESP-NOW:**
   - 1000 packets, sequence ID, tính RTT/2, median latency, 95th percentile jitter, packet loss
   - Đo ở 5m, 10m, 20m, cả static và moving

3. **Velocity ramp:**
   - Paper cần ghi rõ: 2s linear ramp: $v_d(t) = V_D \cdot t / T_\text{ramp}$ trong [0, 2s]

4. **Baseline comparison:**
   - Backstepping-only: Ks1 = Ks2 = 0
   - BSMC: Ks1, Ks2 > 0

5. **Ground-truth options:**
   - Primary: wheel odometry (midpoint + LPF)
   - Secondary: AprilTag camera (30 Hz absolute pose)
   - Tertiary: pen trace (qualitative)

6. **Repetition:** ≥ 3 runs mỗi scenario, báo cáo mean ± std

---

## 7. §VI Results

### ✏️ Sửa

- Không claim mạnh khi data = `[TBD]`
- Chuẩn bị sẵn template bảng:

**Bảng tracking performance:**

| Controller | Trajectory | RMSE ex | RMSE ey | RMSE eθ | Max error | Convergence time |
|------------|-----------|---------|---------|---------|-----------|-----------------|
| Backstepping | Circle | — | — | — | — | — |
| BSMC | Circle | — | — | — | — | — |
| Backstepping | Figure-8 | — | — | — | — | — |
| BSMC | Figure-8 | — | — | — | — | — |

**Bảng ESP-NOW:**

| Condition | Distance | Median latency | Jitter (95%) | Packet loss |
|-----------|----------|---------------|-------------|-------------|
| Static | 5m | — | — | — |
| Moving | 5m | — | — | — |

---

## 8. §VII Conclusion + Future Work

### ❌ Xóa khỏi Future Work

- *"…leader-follower multi-AGV formation control scheme based on a nonlinear-disturbance-observer adaptive super-twisting sliding mode controller…"*
- *"…radial-basis-function-network-based method recently proposed for UAV formations in [9]…"*
- *"…distributed AGV formation scheme combining radial-basis-function sliding mode control with a global vision system…"*

**Lý do:** Quá xa, không liên quan trực tiếp đến hệ thống hiện tại.

### ✏️ Sửa Future Work thành (gần hơn):

> 1. Integrate the AprilTag camera into a full EKF for drift-corrected state estimation.
> 2. Extend to general time-varying trajectories (figure-8, S-curves), where the $v_d(t)$ consistency issue becomes essential.
> 3. Investigate adaptive tuning of BSMC gains and boundary-layer widths using online optimization.
> 4. Extend the single-robot architecture to multi-robot AGV coordination.

### ✏️ Sửa Conclusion

- Chỉ claim đúng theo data đã có.
- Hạ câu VD vs v_d(t): *"We additionally noted a subtle implementation issue regarding v_d(t) versus constant V_D and verified that it does not affect the constant-speed trajectories tested."*

---

## 9. Bug trong code cần fix

| # | File | Dòng | Bug | Cách sửa |
|---|------|------|-----|----------|
| 1 | `bsmc_circle.py` | L247 | `self.VD * (k2*ey + k3*sin)` dùng **VD hằng số** thay vì v_d biến | Đổi `self.VD` → `v_d` (biến local từ trajectory generator) |
| 2 | `bsmc_eight.py` | L228–233 | `w_d + k2*ey + k3*sin(eθ)` **thiếu v_d** nhân trước | Thêm `v_d *` → `w_d + v_d*(k2*ey + k3*sin(eθ))` |
| 3 | `bsmc_controller.py` | L156–161 | Tương tự `bsmc_eight.py`, thiếu `v_d` | Thêm `v_d *` |

> ⚠️ **FIX CODE TRƯỚC, RỒI PAPER KHỚP CODE.**

---

## 10. Checklist trước khi nộp

- [ ] Sửa eq.(10): `tanh(k_w·s)` → `sat(s/φ)` 3 nhánh
- [ ] Sửa Table I: bỏ `k_w`, thêm `φ₁, φ₂`, cập nhật gain đúng code
- [ ] Viết lại §II-C: velocity-level model
- [ ] Sửa "EKF" → dead-reckoning + LPF (hoặc mô tả đúng)
- [ ] Thêm subsection AprilTag camera
- [ ] Thêm subsection Practical Actuator Compensation
- [ ] Ghi ESP-NOW packet format
- [ ] Ghi velocity ramp 2s
- [ ] Ghi encoder LPF α=0.30
- [ ] Ghi PWM_dead = 120 (không để [TBD])
- [ ] Ghi K_calib = 1.22 (không để [TBD])
- [ ] Fix ω_cmd trong code (3 file)
- [ ] Chốt 1 bộ gain → ghi vào Table I
- [ ] Sửa contribution list (5 items mới)
- [ ] Xóa overclaim Abstract
- [ ] Hạ VD vs v_d(t) từ contribution → Remark
- [ ] Hạ "asymptotic convergence" → "UUB / practical stability"
- [ ] Xóa future work UAV/RBF/NDO
- [ ] Sửa future work gần hơn
- [ ] Chạy step-response → lấy k_v, k_ω thực tế
- [ ] Chạy ESP-NOW latency test
- [ ] Chạy ≥ 3 runs thực nghiệm → RMSE mean ± std
# Introduction to AMCL

**Adaptive Monte Carlo Localization (AMCL)** is a **probabilistic localization algorithm** widely used in mobile robotics. Its goal is to estimate a robot’s pose (position and orientation) in a known map, using data from its sensors (e.g., laser scanner) and odometry.

Instead of assuming a single "best guess" for the robot’s position, AMCL represents the robot’s belief as a **set of weighted hypotheses**, called *particles*. Each particle corresponds to a possible robot pose, and the algorithm continuously updates these particles as the robot moves and senses the environment.

## How it works

1. **Prediction (motion update):**
   When the robot moves, each particle is updated according to the odometry information. This step models the uncertainty of motion (e.g., wheel slippage or noise).

2. **Correction (measurement update):**
   The particles are compared against sensor observations (e.g., laser scans). Particles that are consistent with the sensor data get higher weights, while inconsistent ones get lower weights.

3. **Resampling:**
   Particles with low weights are discarded, and particles with high weights are duplicated. Over time, this concentrates the particle set around the true robot pose.

4. **Adaptivity (in AMCL):**
   Unlike standard Monte Carlo Localization (MCL), AMCL adjusts the **number of particles dynamically**. If the robot is uncertain (e.g., after starting or getting lost), AMCL increases the number of particles. If the robot is confident, it reduces the number, saving computational resources.

---

## Key Parameters in AMCL

The behavior of AMCL depends strongly on its configuration parameters.
They can be grouped into three main categories, reflecting the stages of the localization process: **motion prediction**, **sensor correction**, and **filter adaptivity/resampling**.
The following parameters are those used in the **Nav2 AMCL implementation**, but very similar sets exist in other AMCL implementations.

---

### 1. Odometry / Motion Model (Prediction Step)

The motion model controls how the **particles are spread** when the robot moves.
This dispersion accounts for uncertainty in odometry (slippage, sensor error, imperfect control).
Therefore this parameters ensure that the particles evolve realistically given the robot’s kinematics and noise in its movement.

- **Why geometry matters:**
  Robots can be **holonomic** (able to move in any direction, e.g. omnidirectional robots with mecanum wheels) or **non-holonomic** (constrained to certain motions, e.g. differential-drive robots that cannot move sideways).
  The motion model must reflect these constraints, otherwise the particle set would diverge from what is physically possible for the robot.

- **In Nav2 AMCL:**
  - `robot_model_type` selects whether the robot is modeled as differential drive or omnidirectional.
  - `alpha1`–`alpha5` tune how uncertainty (noise) is applied to translations and rotations. They define how wide the “cloud” of particles spreads during motion.

---

### 2. Laser / Sensor Model (Correction Step)

The sensor model controls how the **particles are weighted** when comparing simulated sensor readings to the actual laser scan.
Particles that “see” the world in a way consistent with the real scan gain higher weights, while inconsistent ones lose weight.

- **Big picture:**
  The key trade-off here is between **trusting the map** (exact matches) vs. **accounting for noise and dynamic environments** (unexpected or missing obstacles).
  If the sensor model is too strict, the robot may get lost when reality doesn’t match the map perfectly. If it’s too loose, localization becomes vague.

- **In Nav2 AMCL:**
  - `laser_model_type` chooses the mathematical model (beam model or likelihood field).
  - Parameters like `z_hit`, `z_rand`, `z_max`, and `z_short` define how much weight is given to “good matches,” random noise, max-range readings, or unexpected short readings.
  - `sigma_hit` and `lambda_short` tune how tolerant the filter is to noise.

---

### 3. Overall Filter / Adaptivity (Resampling Step)

The filter parameters determine **how many particles** are maintained, **when to resample**, and how the algorithm adapts its complexity to the robot’s level of certainty.

- **Big picture:**
  - With **too few particles**, the filter is fast but may fail in ambiguous environments.
  - With **too many particles**, localization is robust but computationally expensive.
  - Adaptivity allows the filter to use many particles when uncertain, and fewer when confident.

- **In Nav2 AMCL:**
  - `min_particles` and `max_particles` bound the adaptive range.
  - `update_min_d` and `update_min_a` control how often updates happen (based on robot movement).
  - `resample_interval` defines how often low-weight particles are discarded and high-weight ones duplicated.
  - `pf_err` and `pf_z` relate to statistical thresholds that adjust particle count automatically.

---
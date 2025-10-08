# Introduction to AMCL

**Adaptive Monte Carlo Localization (AMCL)** is a **probabilistic localization algorithm** widely used in mobile robotics. Its goal is to estimate a robot’s pose (position and orientation) in a known map, using data from its sensors (e.g., laser scanner) and odometry.

Instead of assuming a single "best guess" for the robot’s position, AMCL represents the robot’s belief as a **set of weighted hypotheses**, called *particles*. Each particle corresponds to a possible robot pose, and the algorithm continuously updates these particles as the robot moves and senses the environment.

## How it Works

1. **Prediction (motion update):**
  When the robot moves, each particle is updated according to the odometry information. This step models the uncertainty of motion (e.g., wheel slippage or noise).

2. **Correction (measurement update):**
  The particles are compared against sensor observations (e.g., laser scans). Particles that are consistent with the sensor data get higher weights, while inconsistent ones get lower weights.

3. **Resampling:**
  Particles with low weights are discarded, and particles with high weights are duplicated. Over time, this concentrates the particle set around the true robot pose.

4. **Adaptivity (in AMCL):**
  Unlike standard Monte Carlo Localization (MCL), AMCL adjusts the **number of particles dynamically**. If the robot is uncertain (e.g., after starting or getting lost), AMCL increases the number of particles. If the robot is confident, it reduces the number, saving computational resources.

## Key Parameters in AMCL

The behavior of AMCL depends strongly on its configuration parameters.
They can be grouped into three main categories, reflecting the stages of the localization process: **motion prediction**, **sensor correction**, and **filter adaptivity/resampling**.
The following parameters are those used in the **Nav2 AMCL implementation**, but very similar sets exist in other AMCL implementations.

### 1. Odometry / Motion Model (Prediction Step)

The motion model controls how the **particles are spread** when the robot moves.
This dispersion accounts for uncertainty in odometry (slippage, sensor error, imperfect control).
Therefore, these parameters ensure that the particles evolve realistically given the robot’s kinematics and noise in its movement.

- **Why geometry matters:**
  Robots can be **holonomic** (able to move in any direction, e.g., omnidirectional robots with mecanum wheels) or **non-holonomic** (constrained to certain motions, e.g., differential-drive robots that cannot move sideways).
  The motion model must reflect these constraints, otherwise the particle set would diverge from what is physically possible for the robot.

- **In Nav2 AMCL:**
  - `robot_model_type` selects whether the robot is modeled as differential drive or omnidirectional.
  - `alpha1`–`alpha5` tune how uncertainty (noise) is applied to translations and rotations.
    They define how wide the “cloud” of particles spreads during motion.

### 2. Laser / Sensor Model (Correction Step)

The sensor model controls how the **particles are weighted** when comparing simulated sensor readings to the actual laser scan.
Particles that “see” the world in a way consistent with the real scan gain higher weights, while inconsistent ones lose weight.

- **Big picture:**
  The key trade-off here is between **trusting the map** (exact matches) vs. **accounting for noise and dynamic environments** (unexpected or missing obstacles).
  If the sensor model is too strict, the robot may get lost when reality doesn’t match the map perfectly.
  If it’s too loose, localization becomes vague.

- **In Nav2 AMCL:**
  - `laser_model_type` chooses the mathematical model (`beam`, `likelihood_field`, or `likelihood_field_prob`).
  - Parameters such as `z_hit`, `z_rand`, `z_max`, and `z_short` define how much weight is given to “good matches,” random noise, max-range readings, or unexpected short readings.
  - `sigma_hit` and `lambda_short` tune how tolerant the filter is to noise.
  - The optional **beam skipping** parameters (`do_beamskip`, `beam_skip_distance`, etc.) make AMCL more robust to dynamic obstacles.

For detailed explanations and mathematical formulations of these models, see [Sensor Models in AMCL](#sensor-models-in-amcl).

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
  - `pf_err` and `pf_z` relate to statistical thresholds that adjust the particle count automatically.









## Sensor Models in AMCL

The **sensor model** defines how likely a particle’s predicted laser measurements are, given the map.
In Nav2 AMCL, three sensor models are available:

| Model Type | Description | Typical Use |
|-------------|--------------|--------------|
| `beam` | Evaluates each laser beam individually using a mixture model. Accurate but computationally heavier. | Small maps or research scenarios. |
| `likelihood_field` | Uses a precomputed distance field for efficiency. Ignores max-range readings. | Most common for real robots. |
| `likelihood_field_prob` | Likelihood field + probabilistic beam skipping (ignores beams inconsistent with the map). | Dynamic or cluttered environments with moving obstacles. |

### 1. Beam Model (`beam`)

The **beam model** calculates the probability of each laser measurement $ z $ given the particle pose $ x $ and map $ m $.

$$
p(z | x, m) = p_{\text{hit}} + p_{\text{short}} + p_{\text{max}} + p_{\text{rand}}
$$

| Term | Description | Formula |
|------|--------------|----------|
| $p_{\text{hit}}$ | Gaussian noise around expected distance | $z_{\text{hit}} \cdot \exp\left(-\frac{(z - z_{\text{expected}})^2}{2\sigma_{\text{hit}}^2}\right)$ |
| $p_{\text{short}}$ | Unexpected short readings $(z < 0)$ | $z_{\text{short}} \cdot \lambda_{\text{short}} \exp(-\lambda_{\text{short}} z)$ |
| $p_{\text{max}}$ | Sensor reports max range $(z = z_{\text{max}})$ | $z_{\text{max}}$ |
| $p_{\text{rand}}$ | Random uniform noise $(z < z_{\text{max}})$ | $z_{\text{rand}} / z_{\text{max}}$ |

> **Note:** In `nav2_amcl`, beam probabilities are combined using an ad-hoc scheme $ p \mathrel{+}= p_z^3 $ instead of strict multiplication.


### 2. Likelihood Field Model (`likelihood_field`)

The **likelihood field model** uses a precomputed **distance field** for each map cell (distance to nearest obstacle), ignoring max-range readings for efficiency.

$$
p(z | x, m) = z_{\text{hit}} \cdot \exp\left(-\frac{d^2}{2\sigma_{\text{hit}}^2}\right) + z_{\text{rand}} \cdot \frac{1}{z_{\text{max}}}
$$

Where $ d $ is the distance from the laser endpoint to the nearest obstacle.

> **Note:** Beams are combined using the same $ p_z^3 $ ad-hoc method in the beam model.


### 3. Likelihood Field Probabilistic Model (`likelihood_field_prob`)

The **likelihood field probabilistic model** is an extension of **likelihood field model** with **beam skipping** for robustness in dynamic environments. This method skips beams that do not match the majority of particles. Additionally, if too many beams are skipped, all beams are integrated to avoid filter divergence.

$$
p(z | x, m) =
\begin{cases}
z_{\text{hit}} \cdot \exp\left(-\frac{d^2}{2\sigma_{\text{hit}}^2}\right) + z_{\text{rand}} / z_{\text{max}}, & \text{if beam not skipped} \\
\text{ignored}, & \text{otherwise}
\end{cases}
$$

Controlled by these parameters:
  - `beam_skip_distance`: distance threshold for a beam to be considered consistent
  - `beam_skip_threshold`: fraction of particles required to agree
  - `beam_skip_error_threshold`: maximum fraction of skipped beams before integrating them

### Practical Notes

- Lower $\sigma_{\text{hit}}$ (`sigma_hit`): AMCL is more confident but less tolerant to noise.
- Higher $z_{\text{rand}}$ (`z_rand`): more tolerant to outliers, but may cause pose drift.
- Use `likelihood_field_prob` with `do_beamskip: true` for dynamic environments.
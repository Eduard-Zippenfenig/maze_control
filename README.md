### Associated Research Paper

**Assessing the Emotional Response and Performance of Users When Controlling the UR5e Robotic Arm Using Hand Gesture in a Maze Task**

This gesture control system was developed and evaluated as part of the above study, which investigates how psychological stress affects user performance and emotional state during gesture-based human–robot interaction.

---

### Low-pass filter (LPF)

The gesture velocity signal is smoothed using a first-order low-pass filter to reduce jitter caused by hand tremor, tracking noise, and abrupt movements.

The filter is defined as:

`v_filtered = alpha * v_raw + (1 - alpha) * v_previous`

Where:
- `v_raw` is the raw velocity computed from hand displacement  
- `v_filtered` is the filtered velocity  
- `alpha` is the smoothing coefficient (`0 < alpha ≤ 1`)  
- `v_previous` is the filtered velocity from the previous timestep  

A higher `alpha` results in a more responsive but noisier signal, while a lower `alpha` produces smoother but more delayed motion.

---

### Hand position computation

MediaPipe provides normalized hand landmark coordinates in the range `[0, 1]`.

A stable hand reference point is computed as the midpoint between:
- `WRIST`
- `MIDDLE_FINGER_MCP`

The midpoint is calculated as:

`cx = (wrist.x + middle_mcp.x) / 2`  
`cy = (wrist.y + middle_mcp.y) / 2`

---

### Normalization and deadzone

The hand position is normalized relative to the image center `(0.5, 0.5)`:

`dx = cx - 0.5`  
`dy = cy - 0.5`

A deadzone is applied such that:
- if `|dx| < deadzone`, then `dx = 0`
- if `|dy| < deadzone`, then `dy = 0`

This prevents unintended robot motion caused by small involuntary hand movements.

---

### Velocity mapping

The normalized offsets are mapped to planar linear velocities:

`vx = dx * (max_speed / 0.5)`  
`vy = -dy * (max_speed / 0.5)`

The negative sign on `vy` ensures intuitive mapping where upward hand motion results in forward robot motion.

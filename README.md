### Research Context and Purpose

This work was conducted as part of the study  
**Assessing the Emotional Response and Performance of Users When Controlling the UR5e Robotic Arm Using Hand Gesture in a Maze Task**.

The research was designed to investigate how **psychological stress** affects user performance, emotional state, and perceived workload when interacting with a robot through **gesture-based control**.

Gesture control is often considered intuitive and natural, but it is also highly sensitive to variations in human motor behavior. Under stress, users tend to make faster, less precise, and more abrupt movements. These changes can negatively impact system stability, control accuracy, and user confidence. The goal of this research was to determine whether performance degradation under stress originates from **human factors**, **system limitations**, or an interaction of both.

To isolate these effects, a gesture control pipeline was developed with:
- a **stable hand reference point** (midpoint of wrist and middle finger MCP),
- a **deadzone** to suppress involuntary micro-movements,
- and a **first-order low-pass filter (LPF)** to smooth velocity commands.

The low-pass filter was intentionally included to reduce high-frequency noise while preserving responsiveness. This allowed the experiment to focus on how stress alters **intentional hand motion**, rather than amplifying tracking artifacts or sensor noise. By tuning the LPF parameter `alpha`, the system provided a balance between smoothness and immediacy that remained safe for human–robot interaction.

Participants were asked to guide the robot’s end-effector through a maze using hand gestures under two conditions: a normal condition and a stress-induced condition (time pressure and verbal prompts). Performance metrics such as completion time and wall contacts were recorded alongside subjective measures of workload and emotional state.

By combining a controlled gesture interface with psychological stressors, the study aimed to:
- quantify how stress impacts precision and smoothness in gesture-based robot control,
- evaluate whether filtering and deadzones sufficiently stabilize control under pressure,
- and inform the design of more robust, stress-aware human–robot interaction systems.

The findings highlight that gesture-based control is not solely a technical problem but a human-centered one, emphasizing the need to consider emotional and cognitive factors when designing interactive robotic systems.


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

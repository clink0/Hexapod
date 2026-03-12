# Hexapod Serial Control

Arduino Mega 2560 hexapod robot with serial command control, autonomous obstacle avoidance,
adaptive foot placement for uneven terrain, 9DoF IMU body leveling, and autonomous recovery.

---

## Hardware

| Component | Details |
|-----------|---------|
| Microcontroller | Arduino Mega 2560 |
| Servos | 18x standard PWM servos (3 per leg: coxa, femur, tibia) |
| Ultrasonic sensors | 2x HC-SR04 (left and right forward-facing) |
| IR proximity sensors | 2x active-LOW IR modules (left and right) |
| Foot contact sensors | 6x momentary switches (one per leg tip, active LOW) |
| IMU | SparkFun Qwiic ICM-20948 9DoF (I2C / Qwiic connector) |
| LED bar | 8x red/green LED pairs |
| Battery monitor | 2S LiPo via voltage divider to A0 |

---

## File Structure

| File | Purpose |
|------|---------|
| `hexapod_serial.ino` | Entry point: `setup()` and `loop()` only |
| `config.h` | All pin assignments, geometry, and tuning constants |
| `globals.h` | All shared state variables and Servo objects |
| `leds.h` | `LED_Bar()`, `battery_monitor()` |
| `sensors.h` | `get_distance()`, `autonomous_explore()` |
| `kinematics.h` | `leg_IK()` — 3-DOF inverse kinematics per leg |
| `gait.h` | Tripod, wave, ripple, tetrapod gaits + stride helpers |
| `control.h` | `translate_control()`, `rotate_control()`, `one_leg_lift()`, `set_all_90()` |
| `serial_cmd.h` | `process_serial()`, `parse_command()` |
| `debug.h` | `print_debug()` — CSV telemetry every frame |
| `foot_sensors.h` | Adaptive foot placement for uneven terrain |
| `imu.h` | ICM-20948 driver, body leveling, terrain/gait advisor |
| `recovery.h` | Stuck detection and 5-phase autonomous recovery (mode 6) |

---

## Required Libraries

Install via Arduino Library Manager:

- **Servo** (built-in, no install needed)
- **SparkFun 9DoF IMU Breakout - ICM 20948** by SparkFun Electronics

---

## Pin Assignments

### Servos

| Leg | Coxa | Femur | Tibia |
|-----|------|-------|-------|
| 1 Front-Right  | 19 | **8** | 23 |
| 2 Mid-Right    | 25 | 27 | 29 |
| 3 Rear-Right   | 31 | 33 | 35 |
| 4 Rear-Left    | 37 | 39 | 41 |
| 5 Mid-Left     | 43 | 45 | 47 |
| 6 Front-Left   | 49 | 51 | 53 |

Pulse range: 610–2400 µs.

Note: FEMUR1 is on pin 8 (not pin 21) because pin 21 is I2C SCL, reserved for the IMU.
Legs 2–6 follow the odd-pin pattern (23–53). Leg 1 coxa and tibia follow it too; only the femur is the exception.

### LED Bar (even digital pins 22–52)

8 red/green pairs. Each pair occupies 4 consecutive even pins:
Red1=22, Green1=24, Red2=26, Green2=28, ... Red8=50, Green8=52.

### Sensors

| Signal | Pin | Notes |
|--------|-----|-------|
| TRIG_L | 14 | HC-SR04 left trigger |
| ECHO_L | 15 | HC-SR04 left echo |
| TRIG_R | 16 | HC-SR04 right trigger |
| ECHO_R | 17 | HC-SR04 right echo |
| IR_L   | 18 | IR proximity left (active LOW) |
| IR_R   |  9 | IR proximity right (active LOW) — **NOT pin 20** (I2C SDA) |

### Foot Contact Sensors

| Leg | Pin |
|-----|-----|
| 0 Front-Right  | 2 |
| 1 Mid-Right    | 3 |
| 2 Rear-Right   | 4 |
| 3 Rear-Left    | 5 |
| 4 Mid-Left     | 6 |
| 5 Front-Left   | 7 |

All foot pins use `INPUT_PULLUP`. Switch connects pin to GND when foot touches ground.

### I2C / IMU

| Signal | Pin |
|--------|-----|
| SDA | 20 (hardware I2C, do not reassign) |
| SCL | 21 (hardware I2C, do not reassign) |

The ICM-20948 connects via the SparkFun Qwiic connector or directly to SDA/SCL/3.3V/GND.
AD0 high (default Qwiic) → I2C address 0x69.

### Battery Monitor

| Signal | Pin | Notes |
|--------|-----|-------|
| Batt analog | A0 | Voltage divider: R1=20kΩ (batt+), R2=10kΩ (GND). 8.4V max maps to ~2.8V on A0. |

---

## Wiring Guide (summary)

See `WIRING.txt` for the full component-by-component wiring description suitable for drawing a wiring diagram by hand.

---

## Leg Layout and Geometry

```
        FRONT
   [1]       [6]
     \       /
      \     /
  [2]--[body]--[5]
      /     \
     /       \
   [3]       [4]
        REAR
```

Leg 1 = Front-Right, Leg 2 = Mid-Right, Leg 3 = Rear-Right
Leg 4 = Rear-Left,  Leg 5 = Mid-Left,  Leg 6 = Front-Left

Each leg has three joints:
- **Coxa** — rotates leg forward/back (horizontal plane)
- **Femur** — raises/lowers the leg (vertical plane, hip)
- **Tibia** — extends/retracts foot (vertical plane, knee)

Link lengths (mm): Coxa=51, Femur=65, Tibia=121.

Home position toe coordinates from coxa pivot (mm):

| Leg | X | Y | Z |
|-----|---|---|---|
| 1 FR | +82 | +82 | -80 |
| 2 MR |   0 | +116 | -80 |
| 3 RR | -82 | +82 | -80 |
| 4 RL | -82 | -82 | -80 |
| 5 ML |   0 | -116 | -80 |
| 6 FL | +82 | -82 | -80 |

---

## Operating Modes

| Mode | Name | Description |
|------|------|-------------|
| 0 | Idle | Home position, no movement |
| 1 | Walk | Gait selected by `GAIT` command |
| 2 | Translate | Body translation via joystick |
| 3 | Rotate | Body rotation via joystick |
| 4 | One leg lift | Lift individual leg |
| 5 | Autonomous | Obstacle avoidance with sensor steering |
| 6 | Recovery | Auto-triggered when stuck; 5-phase escape sequence |
| 99 | Set all 90 | Drives all servos to 90° for calibration |

---

## Gait Reference

| Gait | ID | Legs in air | Speed | Best use |
|------|----|-------------|-------|----------|
| Tripod   | 0 | 3 at once | Fast   | Open flat ground |
| Wave     | 1 | 1 at a time | Slow  | Obstacles, slopes, careful maneuvering |
| Ripple   | 2 | 2 at once | Medium | General terrain |
| Tetrapod | 3 | 2 at once | Medium | Moderate terrain |

In **mode 5 (autonomous)**, gait switches automatically:
- Wave gait when an obstacle is detected (careful maneuvering)
- Tripod gait when path is clear (maximum speed)

---

## Serial Command Reference

Baud rate: **115200**. Send commands terminated with `\n` or `\r`.

| Command | Arguments | Description |
|---------|-----------|-------------|
| `MODE` | `<0-6,99>` | Set operating mode |
| `GAIT` | `<0-3>` | Select gait (stops movement, resets phases) |
| `SPEED` | `<0-1>` | 0=fast, 1=slow |
| `JOY` | `<RX> <RY> <LX> <LY>` | Set joystick axes (0–255, 128=center) |
| `CAPTURE` | — | Lock current translate/rotate offsets |
| `CLEAR` | — | Clear all offsets, reset step height |
| `HOME` | — | Return to home position, center joysticks |
| `STATUS` | — | Print full state + command reference |

### Joystick Directions

| Action | Command |
|--------|---------|
| Walk forward | `JOY 128 50 128 128` |
| Walk backward | `JOY 128 200 128 128` |
| Turn left | `JOY 128 128 50 128` |
| Turn right | `JOY 128 128 200 128` |
| Stop | `JOY 128 128 128 128` |

RX = right stick X (strafe), RY = right stick Y (forward/back),
LX = left stick X (turn), LY = left stick Y (unused).

---

## IMU Features (ICM-20948)

The IMU runs a complementary filter (96% gyro + 4% accelerometer) to compute pitch and roll.
Heading (yaw) uses tilt-compensated magnetometer fusion.

### Body Leveling

`apply_body_leveling()` computes per-leg Z offsets every frame using a 3D rotation matrix
derived from current pitch and roll. This keeps the body parallel to the ground on slopes.
The leveling offsets are stored separately from the capture offsets so they never accumulate.
Gain is set by `IMU_LEVEL_GAIN` (default 0.70 — full correction can over-rotate on steep slopes).

### Terrain Advisor (`imu_gait_advisor`)

- Measures terrain roughness via accelerometer RMS deviation (mg).
- Scales `step_height_multiplier` between 1.0× (smooth) and `IMU_STEP_HEIGHT_MAX` (2.0×, rough).
- Automatically promotes gait: tripod → wave when tilt exceeds `IMU_GAIT_TILT_THRESH` (12°).
- Hysteresis (`IMU_GAIT_TILT_HYST` = 4°) prevents rapid gait flapping.

---

## Foot Contact System

Each leg has a momentary switch at the foot tip connected to a digital pin with `INPUT_PULLUP`.
The switch closes (pulls LOW) when the foot contacts the ground.

### How It Works

1. **RESET** — When a leg enters swing phase (foot in air), its floor target resets to `HOME_Z`.
2. **LATCH** — When the switch closes, the current Z is saved as the ground height for that leg.
3. **PROBE** — If the foot reaches `HOME_Z` without contact, it continues probing downward at
   `FOOT_PROBE_RATE` mm/frame up to `FOOT_PROBE_DEPTH` mm below `HOME_Z`.
4. **CLAMP** — During stance, the leg's Z is not allowed to rise above the saved ground height,
   keeping the foot planted even if the gait would nominally pull it up.

This allows the robot to walk on surfaces that are 25 mm higher or lower than expected per leg.

---

## Recovery Mode (mode 6)

### Stuck Detection

While walking (modes 1 or 5) with an active joystick command, if the combined tilt
(`sqrt(pitch² + roll²)`) exceeds `STUCK_TILT_DEG` (20°) for `STUCK_CONFIRM_MS` (2000 ms),
the robot is declared stuck and recovery is triggered automatically.

### Recovery Sequence

| Phase | Duration | Action |
|-------|----------|--------|
| STAND    | 1.0 s | Stand tall (`z_body_offset = -40 mm`), hold still |
| FORWARD  | 2.5 s | Wave gait forward at tall stance |
| TURN     | 2.0 s | Turn right in place |
| BACKWARD | 2.5 s | Wave gait backward |
| FORWARD2 | 2.5 s | Wave gait forward again |
| FAILED   | — | Timeout: stop and go idle |

At any point, if tilt drops below `RECOVERY_EXIT_TILT_DEG` (8°), recovery exits with success.
Total timeout: `RECOVERY_TIMEOUT_MS` = 15 s. After recovery the robot returns to mode 0 (idle);
the user must re-send a `MODE` command to resume walking. This is intentional.

---

## Debug Output

Every frame the Arduino prints a CSV line to Serial:

```
<frame_ms>,<batt_V>,<pitch>,<roll>,<yaw>,<accel_rms>,<step_mult>,<mode>
```

| Field | Unit | Description |
|-------|------|-------------|
| frame_ms | ms | Elapsed time since frame start (should be ~0; spikes = overrun) |
| batt_V | V | Smoothed battery voltage |
| pitch | deg | Forward/back tilt |
| roll | deg | Left/right tilt |
| yaw | deg | Compass heading |
| accel_rms | mg | Terrain roughness estimate |
| step_mult | — | Current step height multiplier |
| mode | — | Current operating mode |

---

## Tuning

All tuning constants live in `config.h`.

### Key Constants

| Constant | Default | Effect |
|----------|---------|--------|
| `SAFE_DIST` | 30 cm | Obstacle detection range |
| `SENSOR_INTERVAL` | 60 ms | Time between ultrasonic pings |
| `IMU_COMP_ALPHA` | 0.96 | Complementary filter gyro weight |
| `IMU_LEVEL_GAIN` | 0.70 | Body leveling correction strength |
| `IMU_GAIT_TILT_THRESH` | 12° | Tilt to promote tripod → wave |
| `STUCK_TILT_DEG` | 20° | Tilt threshold for stuck detection |
| `STUCK_CONFIRM_MS` | 2000 ms | How long stuck before recovery triggers |
| `RECOVERY_STANCE_Z` | -40 mm | Extra leg extension during recovery |
| `FOOT_PROBE_DEPTH` | 25 mm | Max probe below HOME_Z |
| `FOOT_PROBE_RATE` | 1.0 mm/frame | Probe extension speed |

### Step Height Multiplier

`step_height_multiplier` scales how high each foot lifts during a step. It is set by the IMU
terrain advisor and also increased to `RECOVERY_STEP_HEIGHT` (2.5×) during recovery.
Larger values improve clearance on rough ground but slow the gait.

---

## Servo Calibration

Trim offsets in `config.h` compensate for mechanical misalignment:

```cpp
const int COXA_CAL[6]  = { 2, -1, -1, -3, -2, -3};
const int FEMUR_CAL[6] = { 4, -2,  0, -1,  0,  0};
const int TIBIA_CAL[6] = { 0, -3, -3, -2, -3, -1};
```

To recalibrate:
1. Send `MODE 99` to drive all servos to 90°.
2. Physically measure which servos are off-center.
3. Adjust the corresponding CAL value (positive = rotate CCW, negative = CW).
4. Repeat until all joints sit at true 90°.

---

## First-Time Setup Checklist

- [ ] Install **SparkFun 9DoF IMU Breakout - ICM 20948** library via Arduino Library Manager
- [ ] Wire `FEMUR1` servo to pin **8** (not pin 21, which is I2C SCL)
- [ ] Verify `IR_R` is wired to pin **9** (not pin 20, which is I2C SDA)
- [ ] Connect ICM-20948 via Qwiic cable to the Mega's Qwiic/I2C header (SDA=20, SCL=21)
- [ ] Wire voltage divider for battery monitor: R1=20kΩ (from batt+), R2=10kΩ (to GND), junction to A0
- [ ] Wire all 6 foot switches: one terminal to the foot pin (2–7), other terminal to GND
- [ ] Upload sketch and open Serial Monitor at **115200 baud**
- [ ] Send `STATUS` — verify IMU shows pitch/roll values (not "NOT FOUND")
- [ ] Send `MODE 99` and verify all servos move; adjust `COXA/FEMUR/TIBIA_CAL` as needed
- [ ] Send `HOME` then `MODE 1` to begin walking
- [ ] Check debug CSV for frame_ms near 0 (no overruns)

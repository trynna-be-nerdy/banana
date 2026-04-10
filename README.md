# VEX Team 20102A — Overdrive
### Program: V5RC | Season: High Stakes

This project implements an advanced motion control system for the **VEX V5** platform, with a core focus on **distance-sensor-assisted localization**, designed for the **VEX High Stakes** competition season. It is built on **PROS** and **LemLib**.

---

## 🚀 Core Feature: Distance-Based Localization

At the heart of this system is a real-time pose correction engine, implemented directly in `src/main.cpp`. The localization system enables the robot to:

- Continuously estimate and correct its **X and Y position** on the field using mounted distance sensors
- Apply **heading-gated corrections** — updates are only accepted when the robot is within a valid angular window, preventing bad data from corrupting the pose
- Fuse data from the **IMU** and **two distance sensors** for reliable odometry
- Correct for odometry drift and wheel slip during autonomous routines

### Why Distance-Sensor Localization?

Traditional VEX V5 robots rely solely on odometry and IMU for position tracking. Over time, these methods accumulate error from wheel slip and field imperfections, leading to unreliable autonomous paths.

This project addresses that by layering sensor-based position corrections on top of LemLib's odometry, delivering:

- More accurate global position estimates during autonomous
- Resilience to accumulated odometry drift
- Consistent, repeatable autonomous paths across match conditions

---

## 🛠️ General Features

- Curvature drive with LemLib exponential drive curves
- Autonomous mode selection via controller D-pad (shown on V5 screen)
- Intake and intake roller control motors
- Pneumatic match loader via ADI digital output
- Roller scoring helper using proportional drive and distance feedback
- Wall alignment utility for angular correction
- Live telemetry on the V5 LCD (X, Y, heading, sensor distances)
- Two prewritten autonomous match routines (left and right side)

---

## ⚙️ Hardware Configuration

| Subsystem        | Details                                      |
|------------------|----------------------------------------------|
| **Drivetrain**   | 2 motors per side — ports `2`, `3` (L) / `4`, `6` (R), green gearset |
| **Intake**       | Port `7` (intake) + Port `8` (intake roller), blue gearset |
| **Top Hat**      | Port `9`, blue gearset                       |
| **Match Loader** | ADI Digital Output — Port `A`               |
| **IMU**          | Port `10`                                    |
| **Left Distance**| Port `13`                                    |
| **Front Distance**| Port `14`                                   |

**LemLib Chassis Configuration:**
- Track width: `11.5 in`
- Wheel diameter: `3.25 in`
- RPM: `200`
- Horizontal drift: `8`

---

## 🗺️ Mock Autonomous Route

The image below shows the simulated autonomous paths for both match sides plotted on the VEX High Stakes field. The colored dotted lines represent the robot's planned trajectories.

![Field Route Map](assets/field_route.png)

> *Green paths indicate the main travel routes, yellow/lime paths show transition movements, and red/orange paths mark approach vectors toward scoring zones. Purple circles represent waypoints and decision nodes.*

### Left Side Route — `matchAuton()`

The left-side routine starts near the center of the field and sweeps toward the bottom-left scoring zone:

| Step | Action | Coordinates (in) |
|------|--------|-----------------|
| 1 | Start pose set | `(50.58, -1.29)` |
| 2 | Move forward | `(50.03, -21.04)` |
| 3 | Arc toward bottom | `(46.41, -48.22)` |
| 4 | Approach right roller | `(66.52, -46.95)` → `scoreHighRoller()` |
| 5 | Cross to left roller | `(22.67, -47.31)` → `scoreHighRoller()` |
| 6 | Re-center | `(44.41, -48.04)` |
| 7 | Drive back toward start zone | `(23.03, -23.03)` |
| 8 | Park near wall | `(12.04, -6.33)` |

### Right Side Route — `matchAuton2()`

The right-side routine mirrors the left, sweeping toward the top-right scoring zone:

| Step | Action | Coordinates (in) |
|------|--------|-----------------|
| 1 | Start pose set | `(49.85, -0.75)` |
| 2 | Move forward | `(47.86, 23.53)` |
| 3 | Arc toward top | `(45.86, 46.73)` |
| 4 | Approach right roller | `(67.25, 46.73)` → `scoreHighRoller()` |
| 5 | Cross to left roller | `(22.85, 46.91)` → `scoreHighRoller()` |
| 6 | Re-center | `(42.78, 46.55)` |
| 7 | Drive back toward start zone | `(23.40, 22.81)` |
| 8 | Park near wall | `(11.38, 6.82)` |

---

## 🕹️ Usage

1. Flash the program to your VEX V5 Brain.
2. Run the program.
3. Use the **D-pad (UP / DOWN)** on the controller to select an autonomous side — the selection is shown on the V5 Brain screen.
4. In driver control:
   - **Left Joystick (Y-axis):** Throttle
   - **Right Joystick (X-axis):** Steering (curvature drive)

---

## 🤖 Autonomous Modes

| Mode | Function |
|------|----------|
| `LEFT` (D-pad UP) | `matchAuton()` — bottom-side scoring route |
| `RIGHT` (D-pad DOWN) | `matchAuton2()` — top-side scoring route |

> **Note:** The live `autonomous()` entrypoint currently executes a `turnToHeading(90)` placeholder. To activate the match routines, `autonomous()` must be updated to branch on `auton_selection`.

Autonomous routines are implemented in `src/main.cpp`.

---

## 🧠 Localization Update Logic

The following code snippet illustrates the distance-based localization correction implemented in this project:

```cpp
// Correct the X position from the left distance sensor when heading is valid.
static void applyDistanceCorrectionX() {
  lemlib::Pose pose = chassis.getPose();
  const double h = std::abs(normalizeHeadingDeg(pose.theta));
  if (!(h <= HEADING_GATE_DEG || std::abs(h - 180.0) <= HEADING_GATE_DEG))
    return;

  const double leftIn = left_distance.get() / 25.4;
  if (!validDistanceIn(leftIn))
    return;

  const double xEstimate =
      std::max(0.0, std::min(FIELD_WIDTH_IN, leftIn + LEFT_SENSOR_OFFSET_IN));
  chassis.setPose(xEstimate, pose.y, pose.theta);
}

// Correct the Y position from the front distance sensor when heading is valid.
static void applyDistanceCorrectionY() {
  lemlib::Pose pose = chassis.getPose();
  const double h = std::abs(normalizeHeadingDeg(pose.theta));
  if (!(std::abs(h - 90.0) <= HEADING_GATE_DEG))
    return;

  const double frontIn = front_distance.get() / 25.4;
  if (!validDistanceIn(frontIn))
    return;

  const double yEstimate =
      std::max(0.0, std::min(FIELD_HEIGHT_IN, FIELD_HEIGHT_IN - frontIn -
                                                  FRONT_SENSOR_OFFSET_IN));
  chassis.setPose(pose.x, yEstimate, pose.theta);
}
```

This correction cycle performs:
1. **Heading Gate Check:** Only applies the correction if the robot is facing a wall within `±25°` of the expected angle, preventing bad readings.
2. **Distance Validation:** Rejects sensor readings outside the configured range (`1.5 in – 120 in`).
3. **Coordinate Clamping:** Estimates are clamped to the `144 × 144 in` field boundary.
4. **Pose Injection:** The corrected coordinate is pushed into the LemLib chassis pose, overriding drift.

---

### Roller Scoring Helper

```cpp
void scoreHighRoller() {
  const double kP_front = 0.4;
  const double kP_left = 0.25;
  const int timeout_ms = 2000;
  const int start = pros::millis();

  while (pros::millis() - start < timeout_ms) {
    const double frontMm = front_distance.get();
    const double leftMm = left_distance.get();

    if (frontMm <= 0) break;

    const double frontError = frontMm - ROLLER_TARGET_FRONT_MM;
    if (std::fabs(frontError) < 4.0) break;

    double drive = frontError * kP_front;
    drive = std::max(-60.0, std::min(60.0, drive));

    double steer = 0.0;
    if (leftMm > 0) {
      const double leftError = leftMm - ROLLER_TARGET_LEFT_MM;
      steer = leftError * kP_left;
      steer = std::max(-20.0, std::min(20.0, steer));
    }

    left_motors.move(drive + steer);
    right_motors.move(drive - steer);
    pros::delay(10);
  }

  intake.move_velocity(-ROLLER_OUTTAKE_SPEED);
  intake_roller.move_velocity(-ROLLER_OUTTAKE_SPEED);
  pros::delay(ROLLER_OUTTAKE_TIME_MS);
}
```

This function:
1. Drives the robot to a target front sensor distance using proportional control.
2. Steers to align laterally using the left distance sensor.
3. Spins the intake in reverse to score once positioned.

---

## 📁 Key Files and Directories

```
banana/
├── src/
│   └── main.cpp           # All robot logic: drivetrain, autonomous, telemetry, localization
├── include/
│   ├── main.h             # PROS entrypoint declarations and project-wide includes
│   ├── lemlib/            # LemLib drivetrain and odometry headers
│   ├── liblvgl/           # LVGL graphics library headers
│   └── pros/              # PROS kernel headers
├── firmware/              # Compiled PROS/LemLib firmware artifacts
├── static/
├── project.pros           # PROS project metadata and template configuration
├── Makefile               # Build instructions
├── PROS_SETUP.md          # Windows development setup guide
└── README.md
```

| File | Purpose |
|------|---------|
| `src/main.cpp` | Main entry point, driver control, autonomous routines, localization |
| `include/main.h` | Header for PROS competition callbacks |
| `project.pros` | PROS metadata: project name `banana`, target `v5`, upload slot `6` |
| `Makefile` | Project build instructions |

---

## 📚 Libraries Used

| Library | Version | Purpose |
|---------|---------|---------|
| **PROS** | `kernel@4.2.2` | C/C++ SDK for VEX V5 |
| **LemLib** | `0.5.6` | Advanced drivetrain control and odometry |
| **LVGL** | `9.2.0` | Lightweight graphics library for embedded systems |

---

## 🏗️ Building the Project

1. Ensure you have the **PROS CLI** installed.
2. Navigate to the project root in your terminal.
3. Run:

```powershell
pros make
```

---

## 🔌 Flashing to V5 Brain

1. Connect your VEX V5 Brain to your computer.
2. Run:

```powershell
pros upload
```

To monitor serial output from the brain:

```powershell
pros terminal
```

---

## 🖥️ VS Code Workflow

This repo includes preconfigured VS Code tasks in `.vscode/tasks.json`:

| Task | Command |
|------|---------|
| `PROS: Build` | Compiles the project |
| `PROS: Upload` | Flashes to V5 Brain |
| `PROS: Build + Upload` | Compiles and flashes |
| `PROS: Terminal` | Opens PROS serial terminal |

---

## 📝 License

This repository contains **PROS**, **LemLib**, and **LVGL** template content, each of which may carry its own upstream license terms. Review the included library files and template metadata before redistributing outside your team or organization.

The custom robot code in `src/main.cpp` is currently unlicensed.

---

*VEX Team 20102A — Overdrive | V5RC High Stakes*

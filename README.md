# VEX TEAM: 20102A 
Name: Overdrive
Program:	V5RC

# banana

PROS V5 robot codebase for a VEX Robotics project using LemLib for chassis control and odometry support. This repository is set up for local development with the PROS CLI and VS Code, and currently contains a compact competition template with drivetrain control, distance-sensor-assisted localization helpers, roller scoring logic, and basic autonomous route scaffolding.

## Overview

This project targets the **VEX V5 Brain** and is built on:

- **PROS kernel 4.2.2**
- **LemLib 0.5.6**
- **LVGL 9.2.0** (included through the PROS template)

The code is centered in `src/main.cpp`, which defines:

- drivetrain motor groups
- intake and mechanism motors
- an inertial sensor and two distance sensors
- LemLib drivetrain, controllers, and chassis objects
- screen telemetry during initialization
- a controller-based autonomous selector
- helper routines for wall alignment, localization correction, and roller scoring

## Current Project State

This repository is functional as a PROS project, but it is still in an early or partially tuned state.

Important notes about the current implementation:

- The drivetrain and controller constants include a `TODO` note indicating that they should be tuned for the exact robot.
- Two autonomous helper routines exist for different match-side scenarios.
- The competition autonomous entrypoint, `autonomous()`, does **not** currently call either of those routines.
- Instead, `autonomous()` currently resets pose and performs a single `turnToHeading(90, 100000)`.
- `competition_initialize()` lets the driver select a side, but that selection is not yet consumed by `autonomous()`.
- `opcontrol()` currently only handles arcade/curvature drive from the master controller sticks.

If you are publishing this repository for teammates, judges, or future maintainers, it is worth treating this repo as a solid chassis/mechanism foundation rather than a finished competition program.

## Features

### Driver Control

- Curvature drive using the master controller
- Left stick Y-axis for throttle
- Right stick X-axis for turning
- LemLib expo drive curves for throttle and steering

### Autonomous Support

- LemLib chassis object configured for motion commands
- Pose tracking through IMU-backed odometry
- Multiple prewritten autonomous behaviors
- A pre-match side selector shown on the V5 screen

### Localization Helpers

- X-position correction from a left-mounted distance sensor
- Y-position correction from a front-mounted distance sensor
- heading-gated correction logic to avoid applying bad updates at the wrong robot orientation

### Mechanism Control Helpers

- Roller scoring helper using front and left distance sensors
- Match loader digital output on ADI port `A`
- Utility alignment routine for turning toward a wall to achieve target spacing

### On-Brain Telemetry

During `initialize()`, a background task prints live telemetry to the V5 LCD:

- robot X position
- robot Y position
- robot heading
- left distance sensor reading
- front distance sensor reading

## Hardware Configuration

The current hardware mapping in `src/main.cpp` is:

### Controller

- Master controller

### Drivetrain

- Left motor group: ports `2`, `3` with reversed configuration on both motors
- Right motor group: ports `4`, `6`
- Gearset: `green`

### Mechanisms

- Intake: port `7`, gearset `blue`
- Intake roller: port `8`, gearset `blue`
- Top hat: port `9`, gearset `blue`
- Match loader: ADI digital output port `A`

### Sensors

- IMU: port `10`
- Left distance sensor: port `13`
- Front distance sensor: port `14`

## Software Architecture

Most custom logic lives in a single file:

- `src/main.cpp`: robot configuration, autonomous code, telemetry, and operator control
- `include/main.h`: PROS entrypoint declarations and project-wide includes
- `project.pros`: PROS metadata, template versions, upload slot, and target information
- `PROS_SETUP.md`: short Windows setup notes
- `.vscode/tasks.json`: VS Code tasks for build, upload, and terminal access

The repository also includes template-provided PROS, LemLib, and LVGL headers and firmware artifacts in `include/` and `firmware/`.

## Autonomous Routines

Multiple autonomous helper functions are already defined for different starting scenarios.

These routines:

- set an initial pose
- drive through a sequence of `moveToPoint(...)` commands
- call `scoreHighRoller()` when needed

However, the live competition autonomous entrypoint currently does this instead:

```cpp
void autonomous() {
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 100000);
}
```

That means the side selection shown during `competition_initialize()` is currently informational only. If you want match behavior to follow the selected routine, `autonomous()` will need to branch on `auton_selection`.

## Localization and Sensor Logic

This project includes simple distance-based pose correction helpers:

- `applyDistanceCorrectionX()`
- `applyDistanceCorrectionY()`
- `applyDistanceLocalizationX()`

These helpers:

- read the left and front distance sensors
- reject invalid readings outside configured distance limits
- only apply corrections when the robot heading is within allowed angular windows
- clamp estimated field coordinates to a `144 x 144` inch field

Field and correction constants are defined near the top of `src/main.cpp`, including:

- field width and height
- sensor offsets
- valid distance range
- heading gating threshold

## Roller Scoring Helper

The `scoreHighRoller()` helper:

- drives the robot toward a target front sensor distance
- optionally steers using the left distance sensor
- stops once the target is reached or timeout occurs
- spins the intake and intake roller in reverse to score

This routine uses fixed tuning values for:

- front distance target
- left distance target
- outtake speed
- outtake duration
- proportional drive and steering gains

These constants are likely robot-specific and may need retuning for reliable field performance.

## Development Setup

### Prerequisites

Install the following on Windows:

- Python 3.11 or newer
- PROS CLI
- VS Code (optional, but recommended)

Install PROS CLI with:

```powershell
py -m pip install pros-cli
```

Verify installation:

```powershell
pros --version
```

## Building and Uploading

From the repository root:

```powershell
pros make
pros upload
pros terminal
```

What these commands do:

- `pros make` builds the project
- `pros upload` flashes the compiled program to the V5 Brain
- `pros terminal` opens the PROS terminal for device interaction and logs

## VS Code Workflow

This repo includes preconfigured VS Code tasks in `.vscode/tasks.json`:

- `PROS: Build`
- `PROS: Upload`
- `PROS: Build + Upload`
- `PROS: Terminal`

Open the repository in VS Code and run these tasks from the command palette or task runner.

## Project Structure

```text
banana/
|-- .vscode/
|   `-- tasks.json
|-- firmware/
|-- include/
|   |-- main.h
|   |-- lemlib/
|   |-- liblvgl/
|   `-- pros/
|-- src/
|   `-- main.cpp
|-- static/
|   `-- example.txt
|-- Makefile
|-- PROS_SETUP.md
`-- project.pros
```

## Configuration Details

From `project.pros`:

- Project name: `banana`
- Target: `v5`
- Upload slot: `6`
- Upload icon: `X`

Installed templates:

- `kernel@4.2.2`
- `LemLib@0.5.6`
- `liblvgl@9.2.0`

## How Competition Control Works

PROS uses the standard competition callbacks declared in `include/main.h`:

- `initialize()`
- `disabled()`
- `competition_initialize()`
- `autonomous()`
- `opcontrol()`

Current behavior:

- `initialize()` calibrates the chassis and launches screen telemetry
- `disabled()` is empty
- `competition_initialize()` loops and allows side selection
- `autonomous()` performs a simple heading turn
- `opcontrol()` continuously reads the controller and drives the chassis

## Known Gaps and Recommended Next Steps

If you plan to continue development, the highest-value improvements are:

1. Connect `auton_selection` to `autonomous()` so the selected routine actually runs.
2. Tune the LemLib drivetrain and controller constants for the real robot.
3. Add operator controls for the intake, intake roller, top hat, and match loader.
4. Decide when to call the distance-correction helpers during autonomous.
5. Add comments or diagrams showing robot orientation and field coordinate conventions.
6. Split hardware definitions and autonomous routines into separate source files as the project grows.

## Troubleshooting

### `pros` command is not recognized

Make sure Python and PROS CLI are installed and available in your `PATH`. Reopen the terminal after installation if needed.

### Build fails after changing libraries or environment

Confirm that your PROS CLI version is working and that the local project still targets the expected templates listed in `project.pros`.

### Robot uploads but behavior is not what you expect

Check:

- motor port assignments
- reversed motor directions
- sensor port mappings
- IMU calibration behavior during startup
- chassis tuning constants
- whether `autonomous()` is actually calling the routine you intended

## Contributing

When modifying this codebase:

- keep hardware mappings synchronized with the real robot
- document any port changes directly in code and in this README
- retune autonomous constants after mechanical changes
- test all motion routines on the field before competition use

## License

This repository contains PROS, LemLib, and LVGL template content, each of which may carry its own upstream license terms. Review the included library files and template metadata before redistributing the project outside your team or organization.

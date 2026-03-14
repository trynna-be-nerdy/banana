#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// lemlib config
pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::green);
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::green);

// intake, roller, and top hat (added from reference project)
pros::Motor intake(7, pros::MotorGearset::blue);
pros::Motor intake_roller(8, pros::MotorGearset::blue);
pros::Motor top_hat(9, pros::MotorGearset::blue);

// Match loader piston (ADI port A)
pros::adi::DigitalOut match_loader('A');

// Drivetrain settings
// TODO: tune these for your exact robot
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                               &right_motors, // right motor group
                               11.5, // track width in inches
                               3.25, // drivetrain wheel diameter
                               200, // drivetrain rpm (green gearset)
                               8 // horizontal drift (traction wheels)
);

// IMU sensor - used for heading tracking
pros::Imu imu(10);

// Distance sensors
// left_distance  (port 13) - points LEFT wall,  used for X-position correction
// front_distance (port 14) - points FRONT wall, used for Y-position correction + roller alignment
pros::Distance left_distance(13);
pros::Distance front_distance(14);

// Field / robot geometry (inches)
constexpr double FIELD_WIDTH_IN  = 144.0;   // 12 ft field east-west
constexpr double FIELD_HEIGHT_IN = 144.0;   // 12 ft field north-south
constexpr double LEFT_SENSOR_OFFSET_IN  = 5.5;  // robot center -> left sensor
constexpr double FRONT_SENSOR_OFFSET_IN = 6.0;  // robot center -> front sensor
constexpr double DIST_MIN_IN  = 1.5;
constexpr double DIST_MAX_IN  = 120.0;
constexpr double HEADING_GATE_DEG = 25.0;   // heading tolerance for wall correction

// Roller scoring constants
// Target distance front sensor should read when flush against the roller (mm)
constexpr double ROLLER_TARGET_FRONT_MM  = 55.0;
// Target left-wall distance for lateral alignment at the roller (mm)
constexpr double ROLLER_TARGET_LEFT_MM   = 130.0;
// Outtake speed as % of max blue motor velocity (600 RPM)
// 60% = 360 RPM — enough to push all balls out reliably without wheel slip
constexpr double ROLLER_OUTTAKE_SPEED    = 360.0;
// How long to run the outtake after contact (ms) — gives all balls time to roll out
constexpr int    ROLLER_OUTTAKE_TIME_MS  = 1500;

static double normalizeHeadingDeg(double heading) {
	while (heading >  180.0) heading -= 360.0;
	while (heading < -180.0) heading += 360.0;
	return heading;
}

static bool validDistanceIn(double d) { return d >= DIST_MIN_IN && d <= DIST_MAX_IN; }

// Forward declaration
extern lemlib::Chassis chassis;

// Correct X-position using left wall sensor (robot facing E/W)
static void applyDistanceCorrectionX() {
	lemlib::Pose pose = chassis.getPose();
	const double h = std::abs(normalizeHeadingDeg(pose.theta));
	// Only trust when robot is mostly facing east or west
	if (!(h <= HEADING_GATE_DEG || std::abs(h - 180.0) <= HEADING_GATE_DEG)) return;

	const double leftIn = left_distance.get() / 25.4;
	if (!validDistanceIn(leftIn)) return;

	const double xEstimate = std::max(0.0, std::min(FIELD_WIDTH_IN,
	    leftIn + LEFT_SENSOR_OFFSET_IN));
	chassis.setPose(xEstimate, pose.y, pose.theta);
}

// Correct Y-position using front wall sensor (robot facing N/S)
static void applyDistanceCorrectionY() {
	lemlib::Pose pose = chassis.getPose();
	const double h = std::abs(normalizeHeadingDeg(pose.theta));
	// Only trust when robot is mostly facing north or south
	if (!(std::abs(h - 90.0) <= HEADING_GATE_DEG)) return;

	const double frontIn = front_distance.get() / 25.4;
	if (!validDistanceIn(frontIn)) return;

	const double yEstimate = std::max(0.0, std::min(FIELD_HEIGHT_IN,
	    FIELD_HEIGHT_IN - frontIn - FRONT_SENSOR_OFFSET_IN));
	chassis.setPose(pose.x, yEstimate, pose.theta);
}

// Run both corrections each opcontrol loop
static void applyDistanceLocalizationX() {
	applyDistanceCorrectionX();
	applyDistanceCorrectionY();
}

// =============================================================================
// HIGH ROLLER SCORING FUNCTION
// =============================================================================
// Uses both distance sensors together to accurately drive into the roller:
//   1. Front sensor controls forward approach — stops at ROLLER_TARGET_FRONT_MM
//   2. Left sensor applies differential correction — keeps robot parallel to wall
//      (if left reads too far, right side drives slightly faster to nudge left)
//   3. Once contact distance is reached, outtake runs at 60% for 1.5 seconds
//      so ALL balls have time to fully roll out before backing away
//
// kP_front  - how aggressively the robot drives toward the roller
// kP_left   - how aggressively it corrects lateral drift
// Call this function from autonomous when the robot is already roughly aligned.
// =============================================================================
void scoreHighRoller() {
	const double kP_front = 0.4;   // forward P gain
	const double kP_left  = 0.25;  // lateral correction P gain
	const int    timeout_ms = 2000;
	const int    start = pros::millis();

	// --- Phase 1: Drive into roller using both sensors ---
	while (pros::millis() - start < timeout_ms) {
		const double frontMm = front_distance.get();
		const double leftMm  = left_distance.get();

		if (frontMm <= 0) break; // sensor error

		const double frontError = frontMm - ROLLER_TARGET_FRONT_MM;
		if (std::fabs(frontError) < 4.0) break; // close enough

		// Forward drive: positive error = still too far, drive forward
		double drive = frontError * kP_front;
		drive = std::max(-60.0, std::min(60.0, drive));

		// Lateral correction: nudge left/right to stay aligned with left wall
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

	// Hold position briefly to settle against roller
	left_motors.move(0);
	right_motors.move(0);
	pros::delay(100);

	// --- Phase 2: Outtake at 60% power for ROLLER_OUTTAKE_TIME_MS ---
	// 50-70% of 600 RPM max = 300-420 RPM. 360 RPM is reliable for all balls.
	intake.move_velocity(-ROLLER_OUTTAKE_SPEED);
	intake_roller.move_velocity(-ROLLER_OUTTAKE_SPEED);
	pros::delay(ROLLER_OUTTAKE_TIME_MS);

	// Stop outtake
	intake.move_velocity(0);
	intake_roller.move_velocity(0);
}

// Odometry sensors - no tracking wheels, uses drivetrain motor encoders + IMU
// LemLib will average the left/right motor groups for forward/back tracking
// The IMU handles heading. No horizontal tracking (traction wheels don't strafe).
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1 (motor encoders used instead)
                            nullptr, // vertical tracking wheel 2
                            nullptr, // horizontal tracking wheel (not needed, no strafing)
                            nullptr, // horizontal tracking wheel 2
                            &imu     // inertial sensor for heading
);

// =============================================================================
// PID TUNING GUIDE
// =============================================================================
// A PID controller calculates motor output based on the error between where
// the robot is and where it wants to be. Three terms work together:
//
//   kP (Proportional) - Reacts to current error.
//       Too low  → robot moves slowly, may not reach target
//       Too high → robot overshoots target and oscillates back and forth
//       Tune first. Start at 5, increase until robot just starts to oscillate,
//       then back off ~30%.
//
//   kI (Integral) - Reacts to accumulated error over time.
//       Fixes steady-state error (robot stops just short of target every time).
//       Almost always leave at 0 for drivetrain. Only add a tiny value (0.01)
//       if the robot consistently stops short on long straight moves.
//
//   kD (Derivative) - Reacts to how fast the error is changing (dampening).
//       Reduces overshoot and oscillation caused by kP.
//       Too low  → oscillation remains
//       Too high → robot becomes jerky / twitchy
//       Tune second. Increase until overshoot is gone but motion stays smooth.
//
//   Anti-windup - Caps how much integral can accumulate.
//       Prevents kI from going crazy when the robot is far from target.
//       Leave at 3 unless you are using kI.
//
//   Small error range / timeout - Once error is within this range (inches or
//       degrees) for this many milliseconds, the movement is considered done.
//       Tighten these for more precise auton, loosen if robot times out too fast.
//
//   Large error range / timeout - Safety fallback. If the robot is still within
//       this larger range after this time, movement ends anyway. Prevents the
//       robot from getting stuck chasing a target it can't quite reach.
//
//   Slew (max acceleration) - Limits how fast the motors can ramp up.
//       Prevents wheel slip on fast starts. Units: motor power units per 10ms.
//       0 = no limit. Only lateral needs this; angular can stay 0.
//
// HOW TO TUNE STEP BY STEP:
//   1. Set kP=5, kI=0, kD=0. Run a straight 24-inch move. Raise kP until
//      the robot oscillates around the target, then lower it a little.
//   2. Keep that kP. Raise kD until oscillation is gone and stopping is clean.
//   3. Only add kI if robot consistently undershoots. Start at 0.01.
//   4. Repeat for angular (turning) with a 90-degree turn test.
//
// MOTOR ENCODER ODOMETRY NOTE:
//   Since there are no dedicated tracking wheels, LemLib uses the average of
//   the 6 drivetrain motor encoders (3 left + 3 right) to track forward/back
//   position, and the IMU for heading. This is less accurate than tracking
//   wheels over long paths but works well for most match autons. The 2 distance
//   sensors will correct X-position drift during the match.
// =============================================================================

// Lateral (forward/backward) PID - tuned for 200 RPM green gearset, 3.25" wheels
// Increase kP if robot is slow to reach targets.
// Increase kD if robot overshoots.
lemlib::ControllerSettings lateral_controller(8,   // kP - start here, tune up/down
                                               0,   // kI - leave at 0
                                               40,  // kD - increase if overshooting
                                               3,   // anti windup
                                               1,   // small error range (inches)
                                               100, // small error timeout (ms)
                                               3,   // large error range (inches)
                                               500, // large error timeout (ms)
                                               20   // slew - limits wheel spin on start
);

// Angular (turning) PID - controls left/right turning accuracy
// Increase kP if turns are too slow or stop short.
// Increase kD if robot oscillates at the end of a turn.
lemlib::ControllerSettings angular_controller(3,   // kP - start here, tune up/down
                                               0,   // kI - leave at 0
                                               20,  // kD - increase if oscillating
                                               3,   // anti windup
                                               1,   // small error range (degrees)
                                               100, // small error timeout (ms)
                                               3,   // large error range (degrees)
                                               500, // large error timeout (ms)
                                               0    // slew - 0 is fine for turns
);

// Drive curves - make joystick feel less twitchy at low speeds
// ExpoDriveCurve(deadband, minOutput, curve)
//   deadband  - joystick values below this are treated as 0 (prevents motor whine)
//   minOutput - minimum motor output when joystick exits deadband (prevents stall)
//   curve     - exponential curve factor (higher = more aggressive at full stick)
lemlib::ExpoDriveCurve throttle_curve(3, 6, 1.019);  // forward/back
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);    // left/right turning

// Chassis object - includes drive curves for throttle/steer priority
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::screen::erase();
	pros::screen::print(TEXT_MEDIUM, 1, "Hello PROS User!");

	// Calibrate the chassis (IMU and tracking wheels)
	chassis.calibrate();

	// Wait for IMU calibration to complete
	while (imu.is_calibrating()) {
		pros::delay(10);
	}

	// Additional settling time for tracking wheels
	pros::delay(500);

	// Set initial pose (x, y, theta) - adjust based on your starting position
	chassis.setPose(0, 0, 0);

	// Persistent screen task (reference behavior + local odom debug)
	static pros::Task screen_task([] {
		while (true) {
			const lemlib::Pose pose = chassis.getPose();
			pros::lcd::print(0, "X: %.2f", pose.x);
			pros::lcd::print(1, "Y: %.2f", pose.y);
			pros::lcd::print(2, "Theta: %.2f", pose.theta);
			pros::lcd::print(3, "Left dist: %d mm", left_distance.get());
			pros::lcd::print(4, "Front dist: %d mm", front_distance.get());
			pros::delay(20);
		}
	});
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
// Auton selector (0 = match 1, 1 = match 2)
static int auton_selection = 0;

void competition_initialize() {
	// D-pad UP = Match 1, D-pad DOWN = Match 2
	// Selection is shown on the brain screen
	while (true) {
		if (controller.get_digital_new_press(DIGITAL_UP)) {
			auton_selection = 0;
		} else if (controller.get_digital_new_press(DIGITAL_DOWN)) {
			auton_selection = 1;
		}
		pros::screen::erase();
		pros::screen::print(TEXT_MEDIUM, 1, "Auton: %s", auton_selection == 0 ? "LEFT" : "RIGHT");
		pros::screen::print(TEXT_MEDIUM, 2, "UP=LEFT   DOWN=RIGHT");
		pros::delay(50);
	}
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
// Align robot to be ~2cm (20mm) from side wall using distance sensor
void alignToWall(double target_mm = 20.0, int timeout_ms = 1500) {
	const int start = pros::millis();
	while (pros::millis() - start < timeout_ms) {
		double dist = left_distance.get();
		if (dist <= 0) break; // sensor error, bail
		double error = dist - target_mm;
		if (std::fabs(error) < 5.0) break; // within 5mm tolerance

		// P-controller: small differential correction to push toward/away from wall
		double correction = error * 0.4;
		if (correction > 25) correction = 25;
		if (correction < -25) correction = -25;

		left_motors.move(correction);
		right_motors.move(-correction);
		pros::delay(10);
	}
	left_motors.move(0);
	right_motors.move(0);
	pros::delay(50);
}

// =============================================================================
// LEFT AUTON  (path.jerryio Path 1 - Y goes negative / south side)
// Starting position: (50.58, -1.29) facing 0 degrees
// Path flows: south → hook right → sweep left → return center
// =============================================================================
void matchAuton() {
	chassis.setPose(50.58, -1.29, 0);

	// Segment 1: drive south to mid-field
	chassis.moveToPoint(50.03, -21.04, 2500);

	// Segment 2: continue south-west to lower roller zone
	chassis.moveToPoint(46.41, -48.22, 3000);

	// Segment 3: sweep right to far right roller
	chassis.moveToPoint(66.52, -46.95, 2500);
	scoreHighRoller(); // score right-side roller

	// Segment 4: sweep left across field
	chassis.moveToPoint(22.67, -47.31, 4500);
	scoreHighRoller(); // score left-side roller

	// Segment 5: back to center-right
	chassis.moveToPoint(44.41, -48.04, 2500);

	// Segment 6: return toward center
	chassis.moveToPoint(23.03, -23.03, 3500);

	// Final: return to start zone
	chassis.moveToPoint(12.04, -6.33, 2500);
}

// =============================================================================
// RIGHT AUTON  (path.jerryio Path 2 - Y goes positive / north side)
// Starting position: (49.85, -0.75) facing 0 degrees
// Path flows: north → hook right → sweep left → return center
// =============================================================================
void matchAuton2() {
	chassis.setPose(49.85, -0.75, 0);

	// Segment 1: drive north to mid-field
	chassis.moveToPoint(47.86, 23.53, 3000);

	// Segment 2: continue north to upper roller zone
	chassis.moveToPoint(45.86, 46.73, 2500);

	// Segment 3: sweep right to far right roller
	chassis.moveToPoint(67.25, 46.73, 2500);
	scoreHighRoller(); // score right-side roller

	// Segment 4: sweep left across field
	chassis.moveToPoint(22.85, 46.91, 4500);
	scoreHighRoller(); // score left-side roller

	// Segment 5: back to center-right
	chassis.moveToPoint(42.78, 46.55, 2500);

	// Segment 6: return toward center
	chassis.moveToPoint(23.40, 22.81, 3500);

	// Final: return to start zone
	chassis.moveToPoint(11.38, 6.82, 2500);
}

void autonomous() {
	if (auton_selection == 1) {
		matchAuton2();
	} else {
		matchAuton();
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
		applyDistanceLocalizationX();

		// Get current pose from odometry
		lemlib::Pose pose = chassis.getPose();
		pros::screen::print(TEXT_MEDIUM, 2, "X: %.1f Y: %.1f", pose.x, pose.y);
		pros::screen::print(TEXT_MEDIUM, 3, "Heading: %.1f", pose.theta);

		// Double-stick curvature drive:
		// left Y = throttle, right X = steer
		const int throttle = controller.get_analog(ANALOG_LEFT_Y);
		const int steer = controller.get_analog(ANALOG_RIGHT_X);
		chassis.curvature(throttle, steer, false); // false -> use throttle/steer curves

		// Added mechanism controls from reference project mapping
		const bool intakeIn = controller.get_digital(DIGITAL_R1);
		const bool intakeOut = controller.get_digital(DIGITAL_R2);
		if (intakeIn) {
			intake.move_velocity(100);
			intake_roller.move_velocity(100);
		} else if (intakeOut) {
			intake.move_velocity(-100);
			intake_roller.move_velocity(-100);
		} else {
			intake.move_velocity(0);
			intake_roller.move_velocity(0);
		}

		if (controller.get_digital(DIGITAL_L1)) {
			top_hat.move_velocity(100);
		} else if (controller.get_digital(DIGITAL_L2)) {
			top_hat.move_velocity(-100);
		} else {
			top_hat.move_velocity(0);
		}

		// Match loader piston toggle (B button)
		if (controller.get_digital_new_press(DIGITAL_B)) {
			static bool match_loader_state = false;
			match_loader_state = !match_loader_state;
			match_loader.set_value(match_loader_state);
		}

		pros::delay(20); // Run for 20 ms then update
	}
}

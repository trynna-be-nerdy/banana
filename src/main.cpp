#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>

// lemlib config
pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::green);
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::green);

// Drivetrain settings
// TODO: tune these for your exact robot
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                               &right_motors, // right motor group
                               11.5, // track width in inches
                               lemlib::Omniwheel::NEW_4, // drivetrain wheel diameter
                               600, // drivetrain rpm
                               2 // horizontal drift
);

// IMU + odometry sensors (as requested)
// TODO: update these ports / reverse flags to match your robot build
pros::Imu imu(10);
pros::Rotation vertical_encoder(11);   // vertical odom wheel
pros::Rotation horizontal_encoder(12); // horizontal odom wheel

// Distance sensors for wall-based localization correction
// Left sensor points WEST wall, right sensor points EAST wall
pros::Distance left_distance(13);
pros::Distance right_distance(14);

// Field / robot geometry for distance-sensor localization (inches)
constexpr double FIELD_WIDTH_IN = 144.0;         // 12 ft field
constexpr double LEFT_SENSOR_OFFSET_IN = 5.5;    // center -> left sensor
constexpr double RIGHT_SENSOR_OFFSET_IN = 5.5;   // center -> right sensor
constexpr double DIST_MIN_IN = 2.0;
constexpr double DIST_MAX_IN = 120.0;
constexpr double HEADING_GATE_DEG = 25.0;        // only trust when mostly E/W aligned

static double normalizeHeadingDeg(double heading) {
	while (heading > 180.0) heading -= 360.0;
	while (heading < -180.0) heading += 360.0;
	return heading;
}

static bool headingAllowsWallCorrection(double headingDeg) {
	const double h = std::abs(normalizeHeadingDeg(headingDeg));
	return h <= HEADING_GATE_DEG || std::abs(h - 180.0) <= HEADING_GATE_DEG;
}

static bool validDistanceIn(double d) { return d >= DIST_MIN_IN && d <= DIST_MAX_IN; }

// Forward declaration: defined later after controller/sensor setup.
extern lemlib::Chassis chassis;

// Fuse left/right distance sensors into X-position estimate (east-west axis)
static void applyDistanceLocalizationX() {
	lemlib::Pose pose = chassis.getPose();

	if (!headingAllowsWallCorrection(pose.theta)) return;

	const double leftIn = left_distance.get() / 25.4;
	const double rightIn = right_distance.get() / 25.4;

	const bool leftValid = validDistanceIn(leftIn);
	const bool rightValid = validDistanceIn(rightIn);

	if (!leftValid && !rightValid) return;

	// If X=0 is WEST wall and +X is EAST:
	// left sensor (pointing WEST):  x = d_left + left_offset
	// right sensor (pointing EAST): x = field_width - d_right - right_offset
	double xEstimate = pose.x;
	if (leftValid && rightValid) {
		const double fromLeft = leftIn + LEFT_SENSOR_OFFSET_IN;
		const double fromRight = FIELD_WIDTH_IN - rightIn - RIGHT_SENSOR_OFFSET_IN;
		xEstimate = (fromLeft + fromRight) * 0.5;
	} else if (leftValid) {
		xEstimate = leftIn + LEFT_SENSOR_OFFSET_IN;
	} else {
		xEstimate = FIELD_WIDTH_IN - rightIn - RIGHT_SENSOR_OFFSET_IN;
	}

	if (xEstimate < 0) xEstimate = 0;
	if (xEstimate > FIELD_WIDTH_IN) xEstimate = FIELD_WIDTH_IN;

	chassis.setPose(xEstimate, pose.y, pose.theta);
}

// Tracking wheel setup
// Sign convention:
// vertical wheel: left of center = negative, right of center = positive
// horizontal wheel: behind center = negative, in front = positive
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_275,
                                              -2.0,
                                              1.0);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_275,
                                                -5.0,
                                                1.0);

// Odometry sensors configuration
lemlib::OdomSensors sensors(&vertical_tracking_wheel,  // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2 (not used)
                            &imu     // inertial sensor
);

// PID controllers for chassis movement
// Lateral (forward/backward) controller settings
lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP)
                                               0,   // integral gain (kI)
                                               3,   // derivative gain (kD)
                                               3,   // anti windup
                                               1,   // small error range (inches)
                                               100, // small error timeout (ms)
                                               3,   // large error range (inches)
                                               500, // large error timeout (ms)
                                               20   // max acceleration (slew)
);

// Angular (turning) controller settings
lemlib::ControllerSettings angular_controller(2,   // proportional gain (kP)
                                                0,   // integral gain (kI)
                                                10,  // derivative gain (kD)
                                                3,   // anti windup
                                                1,   // small error range (degrees)
                                               100, // small error timeout (ms)
                                               3,   // large error range (degrees)
                                               500, // large error timeout (ms)
                                                0    // max acceleration (slew)
);

// Throttle/Steer priority curves
// Slight steer-priority setup:
// - throttle has lower minimum output
// - steer has higher minimum output
lemlib::ExpoDriveCurve throttle_curve(3, 6, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

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
void competition_initialize() {}

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
void autonomous() {}

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		applyDistanceLocalizationX();

		// Get current pose from odometry
		lemlib::Pose pose = chassis.getPose();
		pros::screen::print(TEXT_MEDIUM, 2, "X: %.1f Y: %.1f", pose.x, pose.y);
		pros::screen::print(TEXT_MEDIUM, 3, "Heading: %.1f", pose.theta);

		// Double-stick curvature drive:
		// left Y = throttle, right X = steer
		const int throttle = master.get_analog(ANALOG_LEFT_Y);
		const int steer = master.get_analog(ANALOG_RIGHT_X);
		chassis.curvature(throttle, steer, false); // false -> use throttle/steer curves

		pros::delay(20); // Run for 20 ms then update
	}
}

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// lemlib config
pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::green);
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::green);
// drivetrain settings with 3.25 inch wheels
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              9, // track width in inches
                              lemlib::Omniwheel::NEW_325, // 3.25 inch wheels
                              333.33, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// IMU sensor for heading tracking
pros::Imu imu(10);

// Distance sensors for obstacle detection and positioning
// Adjust port numbers as needed for your robot configuration
pros::Distance distance_front(11); // Front-facing distance sensor
pros::Distance distance_back(12);  // Back-facing distance sensor (optional)

// Odometry wheels (2 vertical tracking wheels)
// Using rotation sensors - adjust port numbers as needed
// Use negative port number for reversed rotation sensor
pros::Rotation vertical_encoder_left(13);   // Left vertical tracking wheel
pros::Rotation vertical_encoder_right(-14); // Right vertical tracking wheel (reversed)

// Create tracking wheels with 2.75" omni wheels (common for odometry)
// Distance is from the tracking center - adjust based on your robot
lemlib::TrackingWheel vertical_tracking_wheel_left(&vertical_encoder_left,
                                                    lemlib::Omniwheel::NEW_275,
                                                    -4.5); // 4.5 inches left of center
lemlib::TrackingWheel vertical_tracking_wheel_right(&vertical_encoder_right,
                                                     lemlib::Omniwheel::NEW_275,
                                                     4.5); // 4.5 inches right of center

// Odometry sensors configuration
lemlib::OdomSensors sensors(&vertical_tracking_wheel_left,  // vertical tracking wheel 1 (left)
                            &vertical_tracking_wheel_right, // vertical tracking wheel 2 (right)
                            nullptr, // horizontal tracking wheel 1 (not used)
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

// Chassis object - combines drivetrain, sensors, and PID controllers
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);



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
		// Get distance sensor readings (in mm, convert to inches for display)
		double front_distance_mm = distance_front.get_distance();
		double back_distance_mm = distance_back.get_distance();
		double front_distance_in = front_distance_mm / 25.4;
		double back_distance_in = back_distance_mm / 25.4;

		// Display distance readings on brain screen
		pros::screen::print(TEXT_MEDIUM, 2, "Front: %.1f in", front_distance_in);
		pros::screen::print(TEXT_MEDIUM, 3, "Back: %.1f in", back_distance_in);

		// Get current pose from odometry
		lemlib::Pose pose = chassis.getPose();
		pros::screen::print(TEXT_MEDIUM, 4, "X: %.1f Y: %.1f", pose.x, pose.y);
		pros::screen::print(TEXT_MEDIUM, 5, "Heading: %.1f", pose.theta);

		// Arcade control scheme using chassis
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		chassis.arcade(dir, turn);

		// Optional: Use distance sensor for collision avoidance
		// If object detected within 12 inches in front, stop or slow down
		if (front_distance_mm > 0 && front_distance_mm < 304.8) { // 12 inches = 304.8 mm
			master.rumble("-"); // Rumble controller to warn driver
		}

		pros::delay(20); // Run for 20 ms then update
	}
}

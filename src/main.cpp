#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cmath>

// Set up the driver controller and drive motors.
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left_motors({-2, -3}, pros::MotorGearset::green);
pros::MotorGroup right_motors({4, 6}, pros::MotorGearset::green);

// Define the scoring and utility mechanisms.
pros::Motor intake(7, pros::MotorGearset::blue);
pros::Motor intake_roller(8, pros::MotorGearset::blue);
pros::Motor top_hat(9, pros::MotorGearset::blue);

pros::adi::DigitalOut match_loader('A');

// Configure the drivetrain and mounted sensors.
// TODO: tune these for your exact robot
lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 11.5, 3.25, 200, 8);

pros::Imu imu(10);

pros::Distance left_distance(13);
pros::Distance front_distance(14);

// Store the field dimensions and sensor correction limits.
// field dimensions in inches, sensor offsets in inches, distance limits in inches, heading gate in degrees
constexpr double FIELD_WIDTH_IN = 144.0;
constexpr double FIELD_HEIGHT_IN = 144.0;
constexpr double LEFT_SENSOR_OFFSET_IN = 5.5;
constexpr double FRONT_SENSOR_OFFSET_IN = 6.0;
constexpr double DIST_MIN_IN = 1.5;
constexpr double DIST_MAX_IN = 120.0;
constexpr double HEADING_GATE_DEG = 25.0;

// Set the target values used when scoring the roller.
// roller
constexpr double ROLLER_TARGET_FRONT_MM = 55.0;
constexpr double ROLLER_TARGET_LEFT_MM = 130.0;
constexpr double ROLLER_OUTTAKE_SPEED = 360.0;
constexpr int ROLLER_OUTTAKE_TIME_MS = 1500;

// Normalize headings so localization checks use a consistent range.
static double normalizeHeadingDeg(double heading) {
  while (heading > 180.0)
    heading -= 360.0;
  while (heading < -180.0)
    heading += 360.0;
  return heading;
}

// Reject distance readings that fall outside the usable range.
static bool validDistanceIn(double d) {
  return d >= DIST_MIN_IN && d <= DIST_MAX_IN;
}

extern lemlib::Chassis chassis;

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

// Run both distance-based localization corrections together.
static void applyDistanceLocalizationX() {
  applyDistanceCorrectionX();
  applyDistanceCorrectionY();
}

// Drive into position and spin the roller to score it.
void scoreHighRoller() {
  const double kP_front = 0.4;
  const double kP_left = 0.25;
  const int timeout_ms = 2000;
  const int start = pros::millis();

  while (pros::millis() - start < timeout_ms) {
    const double frontMm = front_distance.get();
    const double leftMm = left_distance.get();

    if (frontMm <= 0)
      break;

    const double frontError = frontMm - ROLLER_TARGET_FRONT_MM;
    if (std::fabs(frontError) < 4.0)
      break;

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

  left_motors.move(0);
  right_motors.move(0);
  pros::delay(100);

  intake.move_velocity(-ROLLER_OUTTAKE_SPEED);
  intake_roller.move_velocity(-ROLLER_OUTTAKE_SPEED);
  pros::delay(ROLLER_OUTTAKE_TIME_MS);

  intake.move_velocity(0);
  intake_roller.move_velocity(0);
}

// Register the odometry sensors and chassis tuning.
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ControllerSettings lateral_controller(8, 0, 40, 3, 1, 100, 3, 500, 20);

lemlib::ControllerSettings angular_controller(3, 0, 20, 3, 1, 100, 3, 500, 0);

lemlib::ExpoDriveCurve throttle_curve(3, 6, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// Build the main chassis object used by autonomous and driver control.
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

// Initialize the robot and start the live telemetry task.
void initialize() {
  pros::lcd::initialize();
  pros::screen::erase();
  pros::screen::print(TEXT_MEDIUM, 1, "Hello PROS User!");

  chassis.calibrate();
  chassis.setPose(0, 0, 0);

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

// Keep the disabled callback available for future logic.
void disabled() {}

// Track which autonomous route is selected on the controller.
static int auton_selection = 0;

// Allow the driver to choose the autonomous route before the match.
void competition_initialize() {
  while (true) {
    if (controller.get_digital_new_press(DIGITAL_UP)) {
      auton_selection = 0;
    } else if (controller.get_digital_new_press(DIGITAL_DOWN)) {
      auton_selection = 1;
    }
    pros::screen::erase();
    pros::screen::print(TEXT_MEDIUM, 1, "Auton: %s",
                        auton_selection == 0 ? "LEFT" : "RIGHT");
    pros::screen::print(TEXT_MEDIUM, 2, "UP=LEFT   DOWN=RIGHT");
    pros::delay(50);
  }
}

// Rotate the robot until it reaches the requested wall spacing.
void alignToWall(double target_mm = 20.0, int timeout_ms = 1500) {
  const int start = pros::millis();
  while (pros::millis() - start < timeout_ms) {
    double dist = left_distance.get();
    if (dist <= 0)
      break;
    double error = dist - target_mm;
    if (std::fabs(error) < 5.0)
      break;

    double correction = error * 0.4;
    if (correction > 25)
      correction = 25;
    if (correction < -25)
      correction = -25;

    left_motors.move(correction);
    right_motors.move(-correction);
    pros::delay(10);
  }
  left_motors.move(0);
  right_motors.move(0);
  pros::delay(50);
}

// Run the left-side autonomous path for match play.
void matchAuton() {
  chassis.setPose(50.58, -1.29, 0);

  chassis.moveToPoint(50.03, -21.04, 2500);
  chassis.moveToPoint(46.41, -48.22, 3000);
  chassis.moveToPoint(66.52, -46.95, 2500);
  scoreHighRoller();
  chassis.moveToPoint(22.67, -47.31, 4500);
  scoreHighRoller();
  chassis.moveToPoint(44.41, -48.04, 2500);
  chassis.moveToPoint(23.03, -23.03, 3500);
  chassis.moveToPoint(12.04, -6.33, 2500);
}

// Run the right-side autonomous path for match play.
void matchAuton2() {
  chassis.setPose(49.85, -0.75, 0);

  chassis.moveToPoint(47.86, 23.53, 3000);
  chassis.moveToPoint(45.86, 46.73, 2500);
  chassis.moveToPoint(67.25, 46.73, 2500);
  scoreHighRoller();
  chassis.moveToPoint(22.85, 46.91, 4500);
  scoreHighRoller();
  chassis.moveToPoint(42.78, 46.55, 2500);
  chassis.moveToPoint(23.40, 22.81, 3500);
  chassis.moveToPoint(11.38, 6.82, 2500);
}

// Execute the currently active autonomous routine.
void autonomous() {
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 100000);
}

// Read the controller sticks and drive during operator control.
void opcontrol() {
  while (true) {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.curvature(leftY, rightX);

    pros::delay(25);
  }
}

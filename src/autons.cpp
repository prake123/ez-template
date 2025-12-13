#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I 
  chassis.pid_drive_constants_set(15, 0, 100);         // ORIGINALLY (20,0,100) Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(10.0, 0.05, 65.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .
void matchLeft() {
  wings.set(true);
  chassis.pid_drive_set(8_in, 90);
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-22,40);
  chassis.pid_wait();
  bottomIntake.move(127);
  middleIntake.move(127);
  topIntake.move(-127); 
  chassis.pid_drive_set(31_in, 20);
  chassis.pid_wait();
  pros::delay(300);
  bottomIntake.move(-50);
  middleIntake.move(-50);
  topIntake.move(-50);
  pros::delay(50);
  bottomIntake.move(0);
  middleIntake.move(0);
  topIntake.move(0);
  chassis.pid_drive_set(-32_in, 70);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-70, 45);
  chassis.pid_wait();
  chassis.pid_drive_set(33_in ,80);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90,50);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in,80);
  wings.set(false);
  chassis.pid_wait();
  scraper.set(true);
  chassis.pid_wait();
  bottomIntake.move(127);
  middleIntake.move(127);
  topIntake.move(-127); 
  chassis.pid_wait();
  pros::delay(1900);
  wings.set(true);
  chassis.pid_drive_set(34_in, 45 );
  chassis.pid_wait();
  chassis.pid_drive_set(-3_in, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(5_in, 60);
  pros::delay(800);
  bottomIntake.move(0);
  middleIntake.move(0);
  topIntake.move(-0);
  wings.set(false);
  chassis.pid_drive_set(-32_in,70);
  chassis.pid_wait();
  wings.set(false);
  pros::delay(1000);
  bottomIntake.move(127);
  middleIntake.move(127);
  topIntake.move(-127);
  pros::delay(800);
  chassis.pid_drive_set(5_in,60);
  chassis.pid_wait();
  chassis.pid_drive_set(-7_in, 90 );
  pros::delay(3000);
}
                                                                          


void matchRight(){
  wings.set(true);
  chassis.pid_drive_set(8_in, 90);
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(22,40);
  chassis.pid_wait();
  bottomIntake.move(127);
  middleIntake.move(127);
  topIntake.move(-127); 
  chassis.pid_drive_set(30_in, 40);
  chassis.pid_wait();
  bottomIntake.move(-60);
  middleIntake.move(-60);
  topIntake.move(-60);
  pros::delay(60);
  bottomIntake.move(0);
  middleIntake.move(0);
  topIntake.move(0);
  chassis.pid_wait();
  chassis.pid_drive_set(-32_in,70);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(70,45);
  // chassis.pid_wait();
  // chassis.pid_drive_set(36_in,80);
  // chassis.pid_wait();
  // chassis.pid_turn_relative_set(90,50);
  // wings.set(false);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-15_in,80);
  // chassis.pid_wait();
  // scraper.set(true);
  // chassis.pid_wait();
  // bottomIntake.move(127);
  // middleIntake.move(127);
  // topIntake.move(-127); 
  // chassis.pid_wait();
  // pros::delay(1900);
  // wings.set(true);
  // chassis.pid_drive_set(34_in, 45 );
  // chassis.pid_wait();
  // chassis.pid_drive_set(-3_in, 70);
  // chassis.pid_wait();
  // chassis.pid_drive_set(5_in, 100);
  // pros::delay(1000);
  // chassis.pid_drive_set(-32_in,70);
  // chassis.pid_wait();
  // wings.set(false);
  // pros::delay(1000);
  // bottomIntake.move(127);
  // middleIntake.move(127);
  // topIntake.move(-127);
  // pros::delay(200);
  // chassis.pid_drive_set(5_in,60);
  // chassis.pid_wait();
  // chassis.pid_drive_set(-7_in, 90);
  // pros::delay(3000);
  
}
void autonSkills() {
  scraper.set(true);
  wings.set(true);
  chassis.pid_odom_set({{0_in, 35_in},fwd, 90});
  chassis.pid_wait();
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_odom_set({{14_in, 35_in}, fwd, 90});
  pros::delay(1500);
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in, 35_in,-90_deg}, rev, 90});
  chassis.pid_wait();
  topIntake.move(0);
  middleIntake.move(0);
  bottomIntake.move(0);
  scraper.set(false);
  chassis.pid_odom_set({{-28_in, 45_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{-87_in, 45_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{-87_in, 35_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{-76_in, 35_in}, rev, 90});
  wings.set(false);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  scraper.set(true);
  pros::delay(3000);
  wings.set(true);
  chassis.pid_wait();//47.6,15.5
  chassis.pid_odom_set({{-109_in, 35_in}, fwd, 90});
  pros::delay(2000);
  chassis.pid_wait();
  chassis.pid_odom_set({{-76_in, 35_in}, rev, 90});
  wings.set(false);
  scraper.set(false);
  pros::delay(3000);
  chassis.pid_wait();
  chassis.pid_odom_set({{-87_in, 35_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{-93.4_in, -64.5_in}, fwd, 90});
  scraper.set(true);
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_odom_set({{-109.3_in, -64.5_in}, fwd, 90});
  pros::delay(2000);
  chassis.pid_wait();
  chassis.pid_odom_set({{-93.4_in, -64.5_in}, rev, 90});
  scraper.set(false);
  chassis.pid_wait();
  chassis.pid_odom_set({{-74.05_in, -77.043_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{5.734_in, -32_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{5.734_in, -64.5_in, 90_deg}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{-22.13_in, -64.5_in}, rev, 90});
  wings.set(false);
  pros::delay(4000);
  wings.set(true);
  scraper.set(true);
  chassis.pid_wait();
  chassis.pid_odom_set({{14_in, -64.5_in}, fwd, 90});
  pros::delay(3000);
  chassis.pid_wait();
  chassis.pid_odom_set({{-22.13_in, -64.5_in}, rev, 90});
  wings.set(false);
  scraper.set(false);
  chassis.pid_wait();
  pros::delay(2300);
  chassis.pid_odom_set({{5.734_in, -64.832_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{18.588_in, -34.411_in}, fwd, 90});
  chassis.pid_wait();
  scraper.set(true);
  chassis.pid_odom_set({{18.588_in, -15.4235_in}, fwd, 90});
  scraper.set(false);
}




void testAuton() {
  chassis.pid_drive_set(24_in, 50);
  chassis.pid_wait();

  // chassis.pid_turn_set(90_deg, 50);
  // chassis.pid_wait();
}
void matchrightOdom() { // 
  wings.set(true);
chassis.pid_odom_set({0_in, 8_in}, 90);
chassis.pid_wait();
wings.set(true);
chassis.pid_odom_set({{11_in, 28_in}, fwd,  20});
chassis.pid_wait();

bottomIntake.move(127);
middleIntake.move(127);
topIntake.move(-127);

chassis.pid_odom_set({{0_in, 7_in}, rev, 70});
chassis.pid_wait();

bottomIntake.move(-60);
middleIntake.move(-60);
topIntake.move(-60);
pros::delay(60);
bottomIntake.move(0);
middleIntake.move(0);
topIntake.move(0);
chassis.pid_wait();

chassis.pid_odom_set({{36_in, 7_in}, fwd, 80}); // roughly back to y = -2 if we were at y=30
chassis.pid_wait();
wings.set(false);

chassis.pid_odom_set({{36_in , 23_in}, rev, 80});
scraper.set(true);
chassis.pid_wait();
bottomIntake.move(127);
middleIntake.move(127);
topIntake.move(-127); 
pros::delay(1900);
chassis.pid_wait();

chassis.pid_odom_set({{36_in, -11_in},fwd, 50});
chassis.pid_wait();   
wings.set(true);
chassis.pid_odom_set({{36_in , -6_in}, rev, 90});
chassis.pid_wait();

chassis.pid_odom_set({{36_in,-12_in}, fwd, 90});
chassis.pid_wait();
pros::delay(800);
chassis.pid_wait();
pros::delay(1900);

chassis.pid_odom_set({{36_in, 23_in},rev, 90});
chassis.pid_wait();
wings.set(false);
pros::delay(3000);

}
void AWP() {
  
  doublePark.set(false);
  chassis.pid_odom_set({{0_in, 35_in},fwd, 100});
  scraper.set(true);
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(87,90);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_wait();
  chassis.pid_odom_set({{17_in, 35_in}, fwd, 90});
  chassis.pid_wait();
  pros::delay(200);
  topIntake.move(0);
  middleIntake.move(0);
  bottomIntake.move(0);
  wings.set(false);
  chassis.pid_odom_set({{-22_in, 35.5_in}, rev, 100});
  chassis.pid_wait();
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  scraper.set(false);
  //chassis.pid_wait();
  wings.set(false);
  pros::delay(1000);
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  pros::delay(350);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  pros::delay(1250);
  //chassis.pid_odom_set({{-17_in, 36.3_in}, fwd, 60});
  chassis.pid_drive_set(9_in, 50);
  chassis.pid_wait();
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_odom_set({{-22_in,36.5_in}, rev, 110});
  chassis.pid_wait();
  chassis.pid_odom_set({{-9_in, 36.3_in}, fwd, 90});
  chassis.pid_wait();
  wings.set(true);
  chassis.pid_wait();
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_odom_set({{-31.5_in, 5.5_in}, fwd, 70});
  pros::delay(1100);
  scraper.set(true);
  pros::delay(250);
  scraper.set(false);
  chassis.pid_wait();
  //chassis.pid_odom_set({{-26_in, 38_in}, fwd, 90});
  chassis.pid_wait();
  bottomIntake.move(0);
  middleIntake.move(0); 
  topIntake.move(0);  
  wings.set(false);
  chassis.pid_odom_set({{-38.6_in, -1.5_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, 70);
  chassis.pid_wait();
  bottomIntake.move(-127);
  middleIntake.move(-127); //47.6,15.5
  topIntake.move(-127);
  pros::delay(5000);

  //chassis.pid_odom_set({{17_in,-15_in}, fwd, 90});
  
  
  // wings.set(true);
  // bottomIntake.move(0);
  // middleIntake.move(0); 
  // topIntake.move(0);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-30.5_in, 1.5_in}, fwd, 90});
  // pros::delay(700);
  // chassis.pid_odom_set({{0_in, -62_in, 90_deg}, fwd, 90});
  // scraper.set(true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{14_in, -62_in}, fwd, 90});
  // pros::delay(20);
  // chassis.pid_wait();
  // bottomIntake.move(0);
  // middleIntake.move(0);
  // topIntake.move(0);  
  // wings.set(false);
  // chassis.pid_odom_set({{-20_in, -62_in}, rev, 90});
  // longScoring();
  // pros::delay(2000);
  
}

void testAWP(){//match loads both, scores on all 3
//scraper takes 500 ms to get 3 balls
  doublePark.set(false);
  chassis.pid_odom_set({{0_in, 35_in},fwd, 127});
  scraper.set(true);
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(87,127);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_wait();
  chassis.pid_odom_set({{17_in, 35_in}, fwd, 90});
  chassis.pid_wait();
  pros::delay(70);
  topIntake.move(0);
  middleIntake.move(0);
  bottomIntake.move(0);
  wings.set(false);
  chassis.pid_odom_set({{-22_in, 35.5_in}, rev, 127});
  pros::delay(920);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  scraper.set(false);
  //chassis.pid_wait();
  wings.set(false);
  pros::delay(1300);
  // topIntake.move(127); this is all anti jam and pushing blocks in for control
  // middleIntake.move(-127);
  // bottomIntake.move(-127);
  // pros::delay(350);
  // topIntake.move(-127);
  // middleIntake.move(127);
  // bottomIntake.move(127);
  // pros::delay(1250);
  // //chassis.pid_odom_set({{-17_in, 36.3_in}, fwd, 60});
  // chassis.pid_drive_set(9_in, 50);
  // chassis.pid_wait();+
  // wings.set(true);
  // chassis.pid_wait();
  // chassis.pid_odom_set({{-22_in,36.5_in}, rev, 110});
  chassis.pid_odom_set({{{-3_in, 36.3_in}, fwd, 127},
                       {{-31.5_in, 3.5_in}, fwd, 90}});//originally -31.6, 5.5
  wings.set(true);
  topIntake.move(-127); 
  middleIntake.move(127);
  bottomIntake.move(127);
  //chassis.pid_odom_set({{-31.5_in, 5.5_in}, fwd, 70});
  pros::delay(1600);
  scraper.set(true);
  pros::delay(250);
  scraper.set(false);
  //chassis.pid_wait();
  bottomIntake.move(0);
  middleIntake.move(0); 
  topIntake.move(0);  
  wings.set(false);
  chassis.pid_wait(); 
  chassis.pid_odom_set({{-38.9_in, -1.5_in}, fwd, 90});
  chassis.pid_wait();
  bottomIntake.move(-127);
  middleIntake.move(-127); //47.6,15.5
  topIntake.move(-127);
  pros::delay(600);
  chassis.pid_odom_set({{-32_in,16_in,-185_deg} , rev, 127});
  bottomIntake.move(127); 
  middleIntake.move(127);
  topIntake.move(-127);
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_odom_set({{-29.5_in, -37.5_in, 150_deg}, fwd, 127});
  //add scraper to hold balls here
  pros::delay(1050);
  scraper.set(true);
  pros::delay(250);
  scraper.set(false);
  chassis.pid_wait();
  // chassis.pid_odom_set({{38.6_in,-25.5_in},rev,90});
  // topIntake.move(127);
  // middleIntake.move(127);
  // bottomIntake.move(127);
  chassis.pid_odom_set({{{0_in, -63.5_in, 90_deg}, fwd, 120},
                       {{14_in, -64.5_in}, fwd, 90}});
  scraper.set(true); 
  //chassis.pid_odom_set({{14_in, -64.5_in}, fwd, 90});
  pros::delay(2500);
  pros::delay(20); 
  bottomIntake.move(0);
  middleIntake.move(0);
  topIntake.move(0);  
  wings.set(false);                                                                                                                         
  chassis.pid_odom_set({{-22_in, -64.5_in}, rev, 127});
  pros::delay(820);
  bottomIntake.move(127);
  middleIntake.move(127);
  topIntake.move(-127);
  chassis.pid_wait();
  pros::delay(10000);
  
  
  
  }
  void skillsawp(){
    
  doublePark.set(false);
  chassis.pid_odom_set({{0_in, 35_in},fwd, 100});
  scraper.set(true);
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(87,90);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_wait();
  chassis.pid_odom_set({{15_in, 35_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in,60);
  chassis.pid_wait();
  chassis.pid_odom_set({{15_in,35_in},fwd, 70});
  pros::delay(4000);
  topIntake.move(0);
  middleIntake.move(0);
  bottomIntake.move(0);
  wings.set(false);
  chassis.pid_odom_set({{-22_in, 36.5_in}, rev, 100});
  chassis.pid_wait();
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  scraper.set(false);
  //chassis.pid_wait();
  wings.set(false);
  pros::delay(3000);
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  pros::delay(350);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  pros::delay(3250);
  //chassis.pid_odom_set({{-17_in, 36.3_in}, fwd, 60});
  chassis.pid_drive_set(9_in, 50);
  chassis.pid_wait();
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_odom_set({{-22_in,36.5_in}, rev, 110});
  chassis.pid_wait();
  chassis.pid_odom_set({{-9_in, 36.3_in}, fwd, 90});
  chassis.pid_wait();
  wings.set(true);
  chassis.pid_wait();
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_odom_set({{-31.5_in, 5.5_in}, fwd, 70});
  pros::delay(1100);
  scraper.set(true);
  pros::delay(250);
  scraper.set(false);
  chassis.pid_wait();
  //chassis.pid_odom_set({{-26_in, 38_in}, fwd, 90});
  chassis.pid_wait();
  bottomIntake.move(0);
  middleIntake.move(0); 
  topIntake.move(0);  
  wings.set(false);
  chassis.pid_odom_set({{-38.6_in, -1.5_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_drive_set(-2_in, 70);
  chassis.pid_wait();
  bottomIntake.move(-127);
  middleIntake.move(-127); //47.6,15.5
  topIntake.move(-127);
  pros::delay(5000);
  chassis.pid_drive_set(-5_in, 70);
  chassis.pid_wait();
  chassis.pid_odom_set({{18_in,-15_in}, fwd, 90});
  }
  void LeftAWP() {
    doublePark.set(false);
  chassis.pid_odom_set({{0_in, 35_in},fwd, 100});
  scraper.set(true);
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-87,90);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_wait();
  chassis.pid_odom_set({{-15_in, 35_in}, fwd, 90});
  chassis.pid_wait();
  pros::delay(200);
  topIntake.move(0);
  middleIntake.move(0);
  bottomIntake.move(0);
  wings.set(false);
  chassis.pid_odom_set({{22_in, 36.5_in}, rev, 100});
  chassis.pid_wait();
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  scraper.set(false);
  //chassis.pid_wait();
  wings.set(false);
  pros::delay(1000);
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  pros::delay(350);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  pros::delay(1250);
  //chassis.pid_odom_set({{-17_in, 36.3_in}, fwd, 60});
  chassis.pid_drive_set(9_in, 50);
  chassis.pid_wait();
  wings.set(true);
  chassis.pid_wait();
  chassis.pid_odom_set({{22_in,36.5_in}, rev, 110});
  chassis.pid_wait();
  chassis.pid_odom_set({{9_in, 36.3_in}, fwd, 90});
  chassis.pid_wait();
  wings.set(true);
  chassis.pid_wait();
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_odom_set({{31.5_in, 5.5_in}, fwd, 70});
  pros::delay(1100);
  scraper.set(true);
  pros::delay(250);
  scraper.set(false);
  chassis.pid_wait();
  //chassis.pid_odom_set({{-26_in, 38_in}, fwd, 90});
  chassis.pid_wait();
  bottomIntake.move(0);
  middleIntake.move(0); 
  topIntake.move(0);  
  wings.set(false);
  chassis.pid_odom_set({{38.6_in, -1.5_in}, rev, 90});
  chassis.pid_wait();
  chassis.pid_drive_set(-1_in, 70);
  chassis.pid_wait();
  bottomIntake.move(-127);
  middleIntake.move(-127); //47.6,15.5
  topIntake.move(127);
  pros::delay(5000);
  
  }
 

//driver control
void storing() {
  wings.set(true);
  bottomIntake.move(127);
  middleIntake.move(127);
  topIntake.move(-127);
}
void longScoring() {
  
  bottomIntake.move(127);
  middleIntake.move(127);
  topIntake.move(-127);
  wings.set(false);
}
void middleScoring() {
  bottomIntake.move(127);
  middleIntake.move(127);
  topIntake.move(127);
  wings.set(false);
}
void runDoublePark(bool active) {
    if(!doublePark.get() && active){
        topIntake.move(90);
        middleIntake.move(-90);
        bottomIntake.move(-90);
    }

    if(distancesensor.get() < 85){
        pros::delay(60 );
        topIntake.move(0);
        middleIntake.move(0);
        bottomIntake.move(0);
        doublePark.set(true);
    }
} 
void blueColorSort() {
  while (true){
    if (optical.get_hue() > 100){
      //blue
      bottomIntake.move(127);
      middleIntake.move(127);
      topIntake.move(-127);
    }
    else if(optical.get_hue() > 100){
      //reds
      bottomIntake.move(127);
      middleIntake.move(127); 
      topIntake.move(127);
    }
    pros::delay(100);
  }
}
void s() {
  topIntake.move(127);
  pros::delay(10000);
}
void redColorSort(bool middle) {
    int direction = -127; //negative is red, positive is blue
    if (optical.get_hue() > 350){
      //red
    if(!middle){
      bottomIntake.move(0);
      middleIntake.move(-5);
      topIntake.move(direction*.7);
      pros::delay(100);
      bottomIntake.move(127);
      middleIntake.move(40);
      topIntake.move(-127);
      pros::delay(150);
      direction = -127;
    }
    else{
      bottomIntake.move(0);
      middleIntake.move(-10);
      topIntake.move(direction*.8);
      pros::delay(100);
      bottomIntake.move(127);
      middleIntake.move(40);
      topIntake.move(-127);
      pros::delay(150);
      direction = 127;
    }
    }
    else if(optical.get_hue() > 100 && optical.get_hue() < 300){
      if(!middle){
        bottomIntake.move(0);
        middleIntake.move(-10);
        topIntake.move(direction*.7);
        pros::delay(90);
        bottomIntake.move(127);
        middleIntake.move(40);
        topIntake.move(127);
        pros::delay(490);
        direction = 127;
    }
    else{
      bottomIntake.move(0);
      middleIntake.move(-20);
      topIntake.move(direction*.8);
      pros::delay(190);
      bottomIntake.move(127);
      middleIntake.move(40) ;
      topIntake.move(-127);
      pros::delay(190);
      direction = -127;
    }
  
    
  }
}
#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
const double right_sensor_offset = 3.25;  // This is the distance from the front of the robot to the center of the front distance sensor, used for odom calculations
const double left_sensor_offset = 3.25;   // This is the distance from the side of the robot to the center of the side distance sensor, used for odom calculations
const string x = "x";
const string y = "y";
///
// Constants
///
void default_constants() {
  // P, I, D, and Start I 
  //DO NOT TOUCH ANY OF THESE VALUES WITHOUT TELLING PRAKET, AND YOU HAVE TO KNOW HOW TO TUNE CAUSE THIS CAN BREAK THE AUTONS
  //==============================================================================
  //========================PID CONSTANTS=========================================
  chassis.pid_drive_constants_set(18, 0.04, 100);         // ORIGINALLY (20,0,100) Fwd/rev constants, used for odom and non odom motions
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


///
// Swing Example
///
void swing_example() {  
scraper.set(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg,120,40);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg,120,43);
  chassis.pid_wait_quick_chain();
// chassis.pid_drive_set(5_in, 127);
// chassis.pid_wait();
// chassis.pid_drive_set(-5_in, 127);
chassis.pid_wait_quick();
wings.set(true);
topIntake.move(-127);
middleIntake.move(127);
bottomIntake.move(127);
chassis.pid_drive_set(12_in, 100);//barrier cross curves
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(47_in,70);
pros::delay(900);
scraper.set(true);
chassis.pid_wait();
chassis.pid_drive_set(-17_in,60);
chassis.pid_wait();
chassis.pid_turn_relative_set(90_deg, 120);
chassis.pid_wait_quick();
chassis.pid_drive_set(-7_in,90);
chassis.pid_wait();
scraper.set(false);
chassis.odom_xyt_set(36, 127,chassis.odom_theta_get());
chassis.pid_wait();
chassis.pid_odom_set({{36_in, 85.5_in}, fwd, 90});//moving towards middle
pros::delay(800);

scraper.set(true);
chassis.pid_wait();
chassis.pid_turn_set(40_deg, 120);
chassis.pid_wait_quick();
chassis.pid_odom_set({{26.7_in, 69.7_in}, rev, 90});
chassis.pid_wait_quick();
bottomIntake.move(-127);
middleIntake.move(-127); 
topIntake.move(127);
pros::delay(150);
bottomIntake.move(127);
middleIntake.move(127); 
topIntake.move(-127);
chassis.pid_wait();


// anti-jam

topIntake.move(127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(200);

//scoring middle

bottomIntake.move(127);
topIntake.move(90);
middleIntake.move(100);
pros::delay(600);

bottomIntake.move(80);
topIntake.move(35);
middleIntake.move(50);
pros::delay(2000);
                                                                 
}
///
// Motion Chaining 
/// 


///
// Auto that tests everything
///


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


///
// Odom Pure Pursuit
///


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
/*void distanceReset(std::string axis, const double field_length){
  double x,y;
  if(axis == "x"){
    double left = left_distance.get()/25.4 + left_sensor_offset;
    double right = right_distance.get()/25.4 + right_sensor_offset;
    x = (left);
    chassis.odom_xyt_set(x, chassis.odom_y_get(), chassis.odom_theta_get());
    master.print(6, 0, "x reset -> %.2f\n", chassis.odom_x_get());
  }
  else if(axis == "y"){
    double left = left_distance.get()/25.4 + left_sensor_offset;
    double right = right_distance.get()/25.4 + right_sensor_offset;
    y = (left);
    chassis.odom_xyt_set(chassis.odom_x_get(), y, chassis.odom_theta_get());

  }
}*/

void distancetest(){
  chassis.pid_drive_set(10_in, 127);
  chassis.pid_wait();
  distanceReset(x);
  chassis.pid_turn_set(90_deg, 127);
  chassis.pid_wait();
  distanceReset(y);
  chassis.pid_odom_set({{0_in, 0_in}, fwd, 127});
  chassis.pid_wait();
}

void middle(){

//anti jam
topIntake.move(-127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(200);

// score fast

bottomIntake.move(127);
topIntake.move(90);
middleIntake.move(90);
pros::delay(750);

// anti jam

topIntake.move(-127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(400);

// score slow

topIntake.move(45);
middleIntake.move(60);
bottomIntake.move(127);
pros::delay(1000);

// anti jam

topIntake.move(-127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(400);

// score slow

topIntake.move(35);
middleIntake.move(60);
bottomIntake.move(127);
pros::delay(10000);


/*topIntake.move(-127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(200);

topIntake.move(25);
middleIntake.move(40);
bottomIntake.move(117);
pros::delay(2000);*/

}
void parkOnly(){
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-16_in, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(30_in, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(20_in,127);
}
void edge(){
  chassis.pid_wait_quick();
  wings.set(true);
  chassis.pid_drive_set(-6_in,127);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(8_in, 127);
  chassis.pid_wait_quick();
}
void autonSkills() {//needs to be tuned for alignment and changed in future for higher scores
  scraper.set(true);
  wings.set(true);
  chassis.pid_odom_set({{0_in, 34_in},fwd, 90});
  chassis.pid_wait();
  chassis.pid_turn_set(90,90);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);
  chassis.pid_wait();
  chassis.pid_odom_set({{19.5_in, 35_in}, fwd, 80});
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, 127);
  pros::delay(500);
  edge();
  pros::delay(200);
  chassis.pid_wait();
  chassis.pid_drive_set(-19_in,90);
  chassis.pid_wait();
  topIntake.move(0);
  middleIntake.move(0);
  bottomIntake.move(0);
  scraper.set(false);
  chassis.pid_odom_set({{-10_in, 20_in}, fwd, 120});
  chassis.pid_wait();
  chassis.pid_odom_set({{-90_in, 20_in}, fwd, 120});
  chassis.pid_wait();
  chassis.pid_odom_set({{-90_in, 34_in}, fwd, 90});
  chassis.pid_wait();
  chassis.pid_odom_set({{-73_in, 34_in}, rev, 90});//moving to other side and scoring
  chassis.pid_wait();
  wings.set(false);
  scraper.set(true);

  // anti-jam
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);

  pros::delay(2000);
  chassis.pid_wait();//47.6,15.5
  wings.set(true);
  chassis.pid_odom_set({{-111_in, 35_in}, fwd, 90});//scraping
  edge();
  pros::delay(500); //changed from 2500
  chassis.pid_wait();
  chassis.pid_odom_set({{-76_in, 34_in}, rev, 90});//scoring
  chassis.pid_wait();
  wings.set(false);
  scraper.set(false);

  // anti-jam
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);

  pros::delay(1900);
  chassis.pid_wait();
  chassis.pid_odom_set({{-87_in, 35_in}, fwd, 90});//reversing
  chassis.pid_wait();
  chassis.pid_odom_set({{-93.4_in, -64_in}, fwd, 120});//going to other side
  scraper.set(true);
  wings.set(true);
  chassis.pid_wait();

  //chassis.pid_odom_set({{-114_in, -64.6_in}, fwd, 100});//scraping
  chassis.pid_odom_set({{-114_in, -66_in}, fwd, 85}); //NEW LINE FOR SCRAPING
  edge();
  pros::delay(1100);
  chassis.pid_wait();
  wings.set(true);
  chassis.pid_odom_set({{-95_in, -60.5_in}, rev, 120});//reversing to other side
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-95_in, -44_in}, fwd, 120});//reversing to other side
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{0_in, -44_in}, fwd, 120});//reversing to other side
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{0_in, -66.8_in}, fwd, 120});//reversing to other side
  chassis.pid_wait_quick();

  //chassis.pid_odom_set({{-22_in, -68.8_in}, rev, 90});//scoring at long goal
  chassis.pid_odom_set({{-22_in, -68.5_in}, rev, 90}); //NEW LINE FOR SCORING AT LONG GOAL

  chassis.pid_wait();
  wings.set(false);

  // anti-jam
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);

  pros::delay(2000);
  wings.set(true);
  scraper.set(true);
  chassis.pid_wait();

  //chassis.pid_odom_set({{14_in, -68.8_in}, fwd, 90});//going to scraper
  chassis.pid_odom_set({{14_in, -68_in}, fwd, 90}); //NEW LINE FOR GOING TO SCRAPER
  edge();
  chassis.pid_wait();
  pros::delay(1300); //changed from 2500

  //chassis.pid_odom_set({{-22.13_in, -67.5_in}, rev, 120});//going back to long goal
  chassis.pid_odom_set({{-22.13_in, -68.5_in}, rev, 120});//NEW LINE FOR GOING BACK TO LONG GOAL

  chassis.pid_wait();
  wings.set(false);
  scraper.set(false);
  chassis.pid_wait();

  // anti-jam
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);

  pros::delay(1500);
  chassis.pid_odom_set({{0.734_in, -64.832_in}, fwd, 127});//jank parking route try to fix if you have time
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{18.588_in, -34.411_in}, fwd, 127});
  chassis.pid_wait_quick_chain();
  scraper.set(true);
  chassis.pid_odom_set({{18.588_in, -11.4235_in}, fwd, 127});
  scraper.set(false);
  chassis.pid_wait();
  pros::delay(10000);
}

// killer instinct

void autonSkillsplus(){

  // scraper button same as auto run; it sometimes deploys

  scraper.set(false);

  //get cluster

  wings.set(true);
  bottomIntake.move(127);
  middleIntake.move(127); 
  topIntake.move(-127);
  chassis.pid_odom_set({{0_in, 10_in}, fwd, 127});
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-20_deg, 127);
  chassis.pid_wait();
  chassis.pid_odom_set({{-7_in, 27_in}, fwd, 127});
  pros::delay(350);
  scraper.set(true);
  chassis.pid_wait_quick();
  

  // go to middle

  chassis.pid_turn_relative_set(-110_deg, 127);
  pros::delay(100);
  scraper.set(false);
  chassis.pid_wait();
  chassis.pid_odom_set({{6_in, 41_in}, rev, 127});
  chassis.pid_wait();

  //anti-jam

  bottomIntake.move(127);
  middleIntake.move(127); 
  topIntake.move(127);
  pros::delay(300);

  // score middle
bottomIntake.move(-127);
  middleIntake.move(-127); 
  topIntake.move(-127);
  pros::delay(150);
  bottomIntake.move(127);
  topIntake.move(90);
  middleIntake.move(100);
  pros::delay(600);

  // make sure no blocks are stuck in middle intake

  bottomIntake.move(-127);
  middleIntake.move(-127); 
  topIntake.move(-127);
  pros::delay(400);

  // turn on intake for matchload

  bottomIntake.move(127);
  middleIntake.move(127); 
  topIntake.move(-127);

  // going to first matchloader

  chassis.pid_odom_set({{-32_in, 0_in}, fwd, 127});
  scraper.set(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_relative_set(-40_deg, 127);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-32_in, -9_in}, fwd, 127});
  chassis.pid_wait_quick();
  pros::delay(700);
  edge();

  // going to other side

  chassis.pid_odom_set({{-32_in, 4_in}, rev, 127});
  topIntake.move(0);
  middleIntake.move(0);
  bottomIntake.move(0);
  wings.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(-45_deg, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-47_in, 19_in}, rev, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(45_deg, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-47_in, 85_in}, rev, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(45_deg, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-35_in, 96_in}, rev, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0,90);
  chassis.pid_wait();
  chassis.pid_odom_set({{-36_in, 79_in}, rev, 127});
  chassis.pid_wait();

  //score long goal

  wings.set(false);
  scraper.set(true);

  // anti-jam
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  pros::delay(200);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);

  pros::delay(2000);

  // scraping

  wings.set(true);
  chassis.pid_odom_set({{-35_in, 113_in}, fwd, 110});
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-35_in, 120_in}, fwd, 70});
  edge();
  pros::delay(500);
  chassis.pid_wait();
  wings.set(false);

  // scoring long goal

  chassis.pid_odom_set({{-36_in, 82_in}, rev, 120});
  chassis.pid_wait_quick();

  wings.set(false);
  scraper.set(false);

  // anti-jam
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  pros::delay(200);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);

  pros::delay(2000);

  //after you finish long goal code:
  scraper.set(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg,120,40);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg,120,43);
  chassis.pid_wait_quick_chain();
// chassis.pid_drive_set(5_in, 127);
// chassis.pid_wait();
// chassis.pid_drive_set(-5_in, 127);
chassis.pid_wait_quick_chain();
wings.set(true);
topIntake.move(-127);
middleIntake.move(127);
bottomIntake.move(127);
chassis.pid_drive_set(12_in, 120);//barrier cross curves
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(47_in,70);
pros::delay(900);
scraper.set(true);
chassis.pid_wait_quick();
chassis.pid_drive_set(-17_in,60);
chassis.pid_wait();
chassis.pid_turn_relative_set(90_deg, 120);
chassis.pid_wait_quick();
chassis.pid_drive_set(-7_in,120);
chassis.pid_wait_quick();
scraper.set(false);
chassis.odom_xyt_set(36, 127,chassis.odom_theta_get());
chassis.pid_wait_quick();
chassis.pid_odom_set({{36_in, 85.5_in}, fwd, 90});//moving towards middle
pros::delay(800);
scraper.set(true);
chassis.pid_wait();
chassis.pid_turn_set(40_deg, 120);
chassis.pid_wait_quick();
chassis.pid_odom_set({{26.7_in, 74.7_in}, rev, 90});
chassis.pid_drive_set(1_in,127);
chassis.pid_wait_quick();
chassis.pid_wait_quick();
bottomIntake.move(-127);
middleIntake.move(-127); 
topIntake.move(127);
pros::delay(150);
bottomIntake.move(127);
middleIntake.move(127); 
topIntake.move(-127);
chassis.pid_wait();


//anti jam
topIntake.move(-127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(200);

// score fast

bottomIntake.move(127);
topIntake.move(90);
middleIntake.move(90);
pros::delay(750);

// anti jam

topIntake.move(-127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(400);

// score slow

topIntake.move(45);
middleIntake.move(60);
bottomIntake.move(127);
pros::delay(1000);

// anti jam

topIntake.move(-127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(400);

// score slow

topIntake.move(35);
middleIntake.move(60);
bottomIntake.move(127);
pros::delay(1000);

// make sure no blocks are stuck in middle intake

bottomIntake.move(-127);
middleIntake.move(-127); 
topIntake.move(-127);
pros::delay(200);
topIntake.move(-127);
middleIntake.move(127);
bottomIntake.move(127);
wings.set(true);
chassis.pid_odom_set({{64_in, 107_in}, fwd, 127});//moving towards middle??? i dont think so
chassis.pid_wait_quick();
chassis.pid_turn_set(0,90);
chassis.pid_wait_quick();
scraper.set(true);
topIntake.move(-127);
middleIntake.move(127);
bottomIntake.move(127);

wings.set(true);

chassis.pid_odom_set({{65_in, 125_in}, fwd, 90});//at loader
chassis.pid_wait();
pros::delay(500);
edge();

chassis.pid_odom_set({{65_in, 120_in}, rev, 120});
chassis.pid_wait_quick();
topIntake.move(0);
middleIntake.move(0);
bottomIntake.move(0);
wings.set(false);
//scraper.set(false);
//wings.set(false);
chassis.pid_wait();
chassis.pid_odom_set({{74_in, 105_in}, rev, 120});
scraper.set(false);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(0, 120);
chassis.pid_wait_quick_chain();
chassis.pid_odom_set({{74_in, 20_in}, rev, 120});
chassis.pid_wait_quick_chain();
chassis.pid_odom_set({{69_in, 25_in}, rev, 120});
chassis.pid_wait_quick_chain();
chassis.pid_odom_set({{65_in, 35_in}, rev, 120});
chassis.pid_wait_quick();

wings.set(false);//at long goal

// anti-jam
topIntake.move(127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(200);
topIntake.move(-127);
middleIntake.move(127);
bottomIntake.move(127);

pros::delay(2000);

wings.set(true);
scraper.set(true);
chassis.pid_odom_set({{65_in, 5_in}, fwd, 90});
pros::delay(1100);
edge();

chassis.pid_wait();
chassis.pid_odom_set({{65_in, 34_in}, rev, 120});

wings.set(false);//at long goal

// anti-jam
topIntake.move(127);
middleIntake.move(-127);
bottomIntake.move(-127);
pros::delay(200);
topIntake.move(-127);
middleIntake.move(127);
bottomIntake.move(127);

pros::delay(2000);

chassis.pid_wait_quick();       
scraper.set(false);
chassis.pid_swing_set(ez::LEFT_SWING, -135_deg,120,40);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, 127);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, -90_deg,120,45);
chassis.pid_wait_quick_chain();
wings.set(true);
pros::delay(1000);
topIntake.move(-127);
middleIntake.move(127);
bottomIntake.move(127);
chassis.pid_drive_set(30_in, 127);//barrier cross curves 
chassis.pid_wait();
scraper.set(false);
pros::delay(10000);
}

void barriercross(){
  chassis.pid_odom_set({{0_in, 14_in}, fwd, 127});
  chassis.pid_wait();
  chassis.pid_turn_set(45,90);
  chassis.pid_wait();
  chassis.pid_odom_set({{1_in, 20_in}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{4_in, 25_in}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{9_in, 30_in}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_wait_quick_chain();
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{16_in, 34_in}, fwd, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{29_in, 35_in}, fwd, 127});
  chassis.pid_wait_quick_chain();
  }
  //incomplete - just 92 rn
  void autonSkillsplus114(){
scraper.set(false);
/*chassis.pid_swing_set(ez::LEFT_SWING, 45_deg,120,40);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, 127);
chassis.pid_wait_quick_chain();
chassis.pid_swing_set(ez::LEFT_SWING, 90_deg,120,43);
chassis.pid_wait_quick_chain();
// chassis.pid_drive_set(5_in, 127);
// chassis.pid_wait();
// chassis.pid_drive_set(-5_in, 127);
chassis.pid_wait_quick();*/
wings.set(true);
topIntake.move(-127);
middleIntake.move(127);
bottomIntake.move(127);
chassis.pid_drive_set(12_in, 100);//barrier cross curves
chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(47_in,80);
  pros::delay(800);
  scraper.set(true);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in,70);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90_deg, 120);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-7_in,90);
  chassis.pid_wait();
  scraper.set(false);
  chassis.odom_xyt_set(36, 127,chassis.odom_theta_get());
  chassis.pid_wait();

  chassis.odom_xyt_set(-4,-14,chassis.odom_theta_get());

  //use the first part of 97, i based the coordinates above off that


  // scraper button same as auto run; it sometimes deploys

  scraper.set(false);

  //get cluster

  wings.set(true);
  bottomIntake.move(127);
  middleIntake.move(127); 
  topIntake.move(-127);
  chassis.pid_odom_set({{0_in, 10_in}, fwd, 127});
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-20_deg, 127);
  chassis.pid_wait();
  chassis.pid_odom_set({{-7_in, 27_in}, fwd, 127});
  scraper.set(true);
  chassis.pid_wait_quick();

  // go to middle

  chassis.pid_turn_relative_set(-110_deg, 127);
  chassis.pid_wait();
  chassis.pid_odom_set({{6_in, 41_in}, rev, 127});
  chassis.pid_wait();

  //anti-jam

  bottomIntake.move(127);
  middleIntake.move(127); 
  topIntake.move(127);
  pros::delay(300);

  // score middle

  bottomIntake.move(127);
  middleIntake.move(127); 
  topIntake.move(127);
  pros::delay(450);

  // make sure no blocks are stuck in middle intake

  bottomIntake.move(-127);
  middleIntake.move(-127); 
  topIntake.move(-127);
  pros::delay(350);

  // turn on intake for matchload

  bottomIntake.move(127);
  middleIntake.move(127); 
  topIntake.move(-127);

  // going to first matchloader

  chassis.pid_odom_set({{-32_in, 0_in}, fwd, 127});
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-42_deg, 127);
  chassis.pid_wait();
  chassis.pid_odom_set({{-32_in, -9_in}, fwd, 127});
  chassis.pid_wait();
  pros::delay(800);
  edge();

  // going to other side

  chassis.pid_odom_set({{-32_in, 4_in}, rev, 127});
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-45_deg, 127);
  chassis.pid_wait();
  chassis.pid_odom_set({{-45_in, 19_in}, rev, 127});
  chassis.pid_wait();
  chassis.pid_turn_relative_set(45_deg, 127);
  chassis.pid_wait();
  chassis.pid_odom_set({{-45_in, 85_in}, rev, 127});
  chassis.pid_wait();
  chassis.pid_turn_relative_set(45_deg, 127);
  chassis.pid_wait();
  chassis.pid_odom_set({{-32_in, 96_in}, rev, 127});
  chassis.pid_wait();
  chassis.pid_turn_set(0,90);
  chassis.pid_wait();
  chassis.pid_odom_set({{-34.5_in, 82_in}, rev, 127});
  chassis.pid_wait();

  //score long goal

  wings.set(false);
  scraper.set(true);

  // anti-jam
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);

  pros::delay(2000);

  // scraping

  wings.set(true);
  chassis.pid_odom_set({{-34.5_in, 120_in}, fwd, 70});
  edge();
  pros::delay(500);
  chassis.pid_wait();

  // scoring long goal

  chassis.pid_odom_set({{-34.5_in, 82_in}, rev, 90});
  chassis.pid_wait();

  wings.set(false);
  scraper.set(false);

  // anti-jam
  topIntake.move(127);
  middleIntake.move(-127);
  bottomIntake.move(-127);
  topIntake.move(-127);
  middleIntake.move(127);
  bottomIntake.move(127);

  pros::delay(2000);

  //keep this at the topttom
  
  pros::delay(5000);
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
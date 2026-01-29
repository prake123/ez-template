#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

 inline pros::Motor topIntake(-8 );
// inline pros::adi::DigitalIn limit_switch('A');
 inline ez::Piston wings('G'); 
 inline pros::Motor bottomIntake(7);
 inline ez::Piston scraper('H');
 inline pros::Motor middleIntake(-15);
 inline pros::MotorGroup leftMotors({ -16, 12, -2});
 inline pros::MotorGroup rightMotors({13, -14, 11} );
 inline ez::Piston  doublePark('E');  
 inline pros ::Optical optical ( 8 );
 inline pros::Distance distancesensor(3);
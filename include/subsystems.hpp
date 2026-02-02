#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

 inline pros::Motor topIntake(-8);
// inline pros::adi::DigitalIn limit_switch('A');
 inline ez::Piston wings('H'); 
 inline pros::Motor bottomIntake(7);
 inline ez::Piston scraper('G');
 inline pros::Motor middleIntake(-15);
 inline pros::MotorGroup leftMotors({ -6, 12, -2}); //changed from -16, 12, -2
 inline pros::MotorGroup rightMotors({13, -14, 16} ); //changed from 13, -14, 11
 inline ez::Piston intakeLift('E');  
 inline pros ::Optical optical ( 8 );
 inline pros::Distance distancesensor(3);

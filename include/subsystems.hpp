#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

 inline pros::Motor topIntake(-9);
// inline pros::adi::DigitalIn limit_switch('A');
 inline ez::Piston wings('H'); 
 inline pros::Motor bottomIntake(4);
 inline ez::Piston scraper('G');
 inline pros::Motor middleIntake(-8);
 inline pros::MotorGroup leftMotors({-16, 11, -1});
 inline pros::MotorGroup rightMotors({14, -15, 12});
 inline pros::Motor bottomLeft(-16);
 inline pros::Motor middleLeft(11);
 inline pros::Motor topLeft(-1);
 inline pros::Motor bottomRight(14);
 inline pros::Motor middleRight(-15);
 inline pros::Motor topRight(12);
 inline ez::Piston doublePark('F'); 
 inline pros ::Optical optical ( 6 );
 inline ez::Piston hood('b');


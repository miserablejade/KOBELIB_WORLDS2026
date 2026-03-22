#include "vex.h"
#include "movement.h"
#include "threads.h"
#include <climits>
#include <cstdlib>
#include <iterator>
#include <utility>
using namespace vex;
#include <cmath>
#include "math.h"
#include <vector>
#include "globals.h"

float endtime = 0;
float starttime = 0;

// ANGULAR PID CONSTANTS
float TKI = 0.01;

float _0DegTKP = 0.27;
float _0DegTKD = 1.4;

float _90DegTKP = 0.27;
float _90DegTKD = 1.46;

float _180DegTKP = 0.2;
float _180DegTKD = 1.2;

float _270DegTKP = 0.2;
float _270DegTKD = 1.38;

float _360DegTKP = 0.2; 
float _360DegTKD = 1.38;

// HEADING PID CONSTANTS
float HKP = _0DegTKP;
float HKD = _0DegTKD;

// LINEAR PID CONSTANTS
float YKP = 1.1; //1.3
float YKI = 0;
float YKD = 5.5; //10

// ARC CORRECTION CONSTANTS
float ARC_CT_KP = 1;  // cross-track P (distance from arc)
float ARC_HDG_KP = 0.6; // heading P (alignment with arc tangent)

// SLEW
float slew = 0;

//TRACK WIDTH - Important for arcs and BLRS Curvature calculations (which is the curveTo backend)
float trackwidth = 11.75;

//Default speed caps
float ymaxSpeed = 12.7;
float amaxSpeed = 12.7;
float hmaxSpeed = ymaxSpeed/2;

//Drive minimum output
float drive_min_output = 0;

//Sweep turns
float sweep_ratio = 0; 
//A sweep ratio of 0.5 would mean the locked side goes half as fast as the turning side. 
// A sweep ratio of 1 would mean both sides output the same value.

//Straightline Jumpstarts
bool bangbangJumpstart = true;
double bangbangDist = 0.25;

//Smart turning early exits
bool enableTurningEarlyExits = false;
bool turningEarlyExits = false;
double turningEarlyExitLength = 0.9;

//Lateral movement early exits
double exitMovementIn = 0;

//Don't touch this stuff
int turnmargin = 65;
int turnstop = 5;
int track = 10;
float breakang = 5;

void autonStart() {
  chassis.resetData();
  chassis.ytarg = 0;
  chassis.xtarg = 0;
  chassis.htarg = 0;
  starttime = Brain.timer(msec);
}

void autonStop() { //Kobe used this, idk what its for ngl
    endtime = Brain.timer(msec) - starttime;
    delay(10000000);
}

void auto_settings() {
  autonStart();
  move.turndisablelength = 6; // After the robot is this distance away from the point, heading correction ends.
  move.bigTime = 300;
  turnPID.integralbound = 10;
  turnPID.maxintegral = 2;

  move.seamlessTransitions = false;

  turnPID.scalePID = true;
  drivePID.scalePID = false;

  turnPID.scalablePID = true;
  drivePID.scalablePID = false;

  turnPID.setScalableConstants(
  TKI,
  _0DegTKP, _0DegTKD,
  _90DegTKP, _90DegTKD,
  _180DegTKP, _180DegTKD,
  _270DegTKP, _270DegTKD,
  _360DegTKP, _360DegTKD
);
  
}

float angleDiff(float target, float current) {
  float diff = target - current;

  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;

  return diff;
}                                                                                                                          

///////////////////////////////////////////////////////////////////////////////////////
//  __          __     _      _        _____  ______          _____ _____ _   _  _____
//  \ \        / /\   | |    | |      |  __ \|  ____|   /\   |  __ \_   _| \ | |/ ____|
//   \ \  /\  / /  \  | |    | |      | |__) | |__     /  \  | |  | || | |  \| | |  __ 
//    \ \/  \/ / /\ \ | |    | |      |  _  /|  __|   / /\ \ | |  | || | | . ` | | |_ |
//     \  /\  / ____ \| |____| |____  | | \ \| |____ / ____ \| |__| || |_| |\  | |__| |
//      \/  \/_/    \_\______|______| |_|  \_\______/_/    \_\_____/_____|_| \_|\_____|
///////////////////////////////////////////////////////////////////////////////////////

// Sensor offsets (distance from sensor to tracking center)
double sensor1Offset = 18.97;
double sensor2Offset = 0;
double sensor3Offset = 19.1;
double sensor4Offset = 18.7;

void readWall(int wall, int sensor, bool bypassSettleDelay = false) {
  rightdrive.stop(brake);
  leftdrive.stop(brake);

  if (not bypassSettleDelay) {
    delay(10);
  }

  double distanceReading = 0;
  double sensorOffset = 0;

  if (sensor == 1) { distanceReading = DistSensor1.objectDistance(inches); sensorOffset = sensor1Offset; }
  else if (sensor == 2) { distanceReading = DistSensor2.objectDistance(inches); sensorOffset = sensor2Offset; }
  else if (sensor == 3) { distanceReading = DistSensor3.objectDistance(inches); sensorOffset = sensor3Offset; }
  else if (sensor == 4) { distanceReading = DistSensor4.objectDistance(inches); sensorOffset = sensor4Offset; }

  double relativeOffset = sensorOffset - distanceReading;

  // Walls 1 & 2 are positive edges (+48), walls 3 & 4 are negative edges (-48)
  double sign;
  if (wall <= 2) {
    sign = 1;
  } else {
    sign = -1;
  }

  double newPos = sign * (48 + relativeOffset);

  // Walls 1 & 3 set X, walls 2 & 4 set Y
  if (wall == 1 || wall == 3) {
    chassis.setPos(newPos, chassis.y, chassis.h);
  }
  
  if (wall == 2 || wall == 4) {
    chassis.setPos(chassis.x, newPos, chassis.h);
  }

}

/*
Note: walls look like this:

   2
3     1
   4

*/

////////////////////////////////////////////////////////////////////
//   ____          _   _  _____        ____          _   _  _____ 
//  |  _ \   /\   | \ | |/ ____|      |  _ \   /\   | \ | |/ ____|
//  | |_) | /  \  |  \| | |  __ ______| |_) | /  \  |  \| | |  __ 
//  |  _ < / /\ \ | . ` | | |_ |______|  _ < / /\ \ | . ` | | |_ |
//  | |_) / ____ \| |\  | |__| |      | |_) / ____ \| |\  | |__| |
//  |____/_/    \_\_| \_|\_____|      |____/_/    \_\_| \_|\_____|
////////////////////////////////////////////////////////////////////

void bangbangDrive(float dist, float breakWithin, bool coastExit = true, float coastFor = 10) {
  // Record starting position
  float startX = chassis.x;
  float startY = chassis.y;

  float targetDist = fabs(dist);
  bool backward = (dist < 0);

  float traveled = 0;
  float breakDist = targetDist * breakWithin;

  // Decide direction
  if (!backward) {
    move.voldrive(12.7, 12.7);
  } else {
    move.voldrive(-12.7, -12.7);
  }

  // Bang-bang loop
  while (traveled < (targetDist - breakDist)) {
    float dx = chassis.x - startX;
    float dy = chassis.y - startY;
    traveled = sqrt(dx * dx + dy * dy);
    delay(10);
  }

  // Brake hard
  if (coastExit) {
    leftdrive.stop(coast);
    rightdrive.stop(coast);
    delay(coastFor);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//    ____  _    _ _____ _____ _  __  ______ _    _ _   _  _____ _______ _____ ____  _   _  _____ 
//   / __ \| |  | |_   _/ ____| |/ / |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
//  | |  | | |  | | | || |    | ' /  | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___  
//  | |  | | |  | | | || |    |  <   |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \ 
//  | |__| | |__| |_| || |____| . \  | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) |
//   \___\_\\____/|_____\_____|_|\_\ |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/ 
/////////////////////////////////////////////////////////////////////////////////////////////////////

float calculateTargetHeading(float targetX, float targetY, bool ifBackward, float fromPointX = 0, float fromPointY = 0) {
  float dx;
  float dy;

  if (fromPointX == 0 && fromPointY == 0) {
    dx = targetX - chassis.x;
    dy = targetY - chassis.y;
  } else {
    dx = targetX - fromPointX;
    dy = targetY - fromPointY;
  }

  // atan2 gives 0° = +X, 90° = +Y
  float headingRad = atan2(dy, dx);
  float headingDeg = headingRad * 180.0 / M_PI;

  // Rotate frame so 0° = +Y
  headingDeg = 90 - headingDeg;

  // Normalize to [0, 360)
  while (headingDeg < 0) headingDeg += 360;
  while (headingDeg >= 360) headingDeg -= 360;

  if (ifBackward) {
    headingDeg += 180;
    if (headingDeg >= 360) headingDeg -= 360;
  }

  return headingDeg;
}
                                               
//These exist for parameter autofill. Way quicker to write driveForward(24) than move.straighth(24, chassis.h, YKP, YKD, HKP, HKD, ymaxSpeed, hmaxSpeed, slew, 3);
//The root cause for the lack of autofill is because ClangD conflicts with the VexCode compiler, but ClangD is goated so just keep it.

//Quick forwards backward motions
void driveForward(float dist, float heading = chassis.h, float breakdist = 3, float yspeed = ymaxSpeed, float ykp = YKP, float ykd = YKD) {

  if (bangbangJumpstart) {
    bangbangDrive(dist * bangbangDist, 0.2, true);
    dist -= dist * bangbangDist;
  }

  if (exitMovementIn != 0) {
    move.straighth(dist, heading, ykp, ykd, HKP, HKD, yspeed, hmaxSpeed, slew, 0);
    delay(exitMovementIn);
    exitMovementIn = 0;
  }

  move.straighth(dist, heading, ykp, ykd, HKP, HKD, yspeed, hmaxSpeed, slew, breakdist);

}

void driveBackward(float dist, float heading = chassis.h, float breakdist = 3, float yspeed = ymaxSpeed, float ykp = YKP, float ykd = YKD) {

  if (bangbangJumpstart) {
    bangbangDrive(-dist * bangbangDist, 0.2, true);
    dist -= dist * bangbangDist;
  }

  if (exitMovementIn != 0) {
    move.straighth(-dist, heading, ykp, ykd, HKP, HKD, yspeed, hmaxSpeed, slew, 0);
    delay(exitMovementIn);
    exitMovementIn = 0;
  }

  move.straighth(-dist, heading, ykp, ykd, HKP, HKD, yspeed, hmaxSpeed, slew, breakdist);

}

// Easy turning motions
void face(float heading, float breakang = 8, int dir = 0) {

  move.face(heading, amaxSpeed, breakang, dir);

}

void facePoint(float x, float y, float breakang = 8, bool backward = false, int dir = 0) {

  move.facePoint(x, y, amaxSpeed, breakang, backward, dir);

}

void swingOnRight(float heading, float breakang = 8, bool backward = false, int dir = 0) {

  move.swingOnRight(heading, amaxSpeed, breakang, dir);

}

void swingOnLeft(float heading, float breakang = 8, bool backward = false, int dir = 0) {

  move.swingOnLeft(heading, amaxSpeed, breakang, dir);

}

void swingOnRightToPoint(float x, float y, float breakang = 8, bool backward = false, int dir = 0) {

  move.swingOnRighttoPoint(x, y, amaxSpeed, breakang, backward, dir);

}

void swingOnLeftToPoint(float x, float y, float breakang = 8, bool backward = false, int dir = 0) {

  move.swingOnLefttoPoint(x, y, amaxSpeed, breakang, backward, dir);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//    _____ _______ _____            _____ _____ _    _ _______ _      _____ _   _ _____ _   _  _____ 
//   / ____|__   __|  __ \     /\   |_   _/ ____| |  | |__   __| |    |_   _| \ | |_   _| \ | |/ ____|
//  | (___    | |  | |__) |   /  \    | || |  __| |__| |  | |  | |      | | |  \| | | | |  \| | |  __ 
//   \___ \   | |  |  _  /   / /\ \   | || | |_ |  __  |  | |  | |      | | | . ` | | | | . ` | | |_ |
//   ____) |  | |  | | \ \  / ____ \ _| || |__| | |  | |  | |  | |____ _| |_| |\  |_| |_| |\  | |__| |
//  |_____/   |_|  |_|  \_\/_/    \_\_____\_____|_|  |_|  |_|  |______|_____|_| \_|_____|_| \_|\_____|
/////////////////////////////////////////////////////////////////////////////////////////////////////////

float calculateStraightlineDist(float targetX, float targetY) {
  //Find distance using distance formula
  float x1 = chassis.x;
  float y1 = chassis.y;
  float dist = std::abs(std::sqrt((targetX-x1)*(targetX-x1)+(targetY-y1)*(targetY-y1)));

  return dist;
}

void straightTo(float x, float y, bool backward = false, float breakdist = 3, float breakang = 5, float yspeed = ymaxSpeed, float hspeed = amaxSpeed) {

  float distance = calculateStraightlineDist(x, y);
  float heading  = calculateTargetHeading(x, y, backward);

  if (backward) {
    if (bangbangJumpstart) {
    bangbangDrive(-distance * bangbangDist, 0.2, true);

    distance = calculateStraightlineDist(x, y);

    }
    
    move.straighth(-distance, heading, YKP, YKD, HKP, HKD, yspeed, hmaxSpeed, slew, breakdist);
  }

  if (not backward) {
    if (bangbangJumpstart) {
    bangbangDrive(distance * bangbangDist, 0.2, true);

    distance = calculateStraightlineDist(x, y);
    
    }

    move.straighth(distance, heading, YKP, YKD, HKP, HKD, yspeed, hmaxSpeed, slew, breakdist);
  }

}

void straightlineTo(float x, float y, bool backward = false, float breakdist = 3, float breakang = 5, float yspeed = ymaxSpeed, float hspeed = amaxSpeed, float ykp = YKP, float ykd = YKD) {

  facePoint(x, y, breakang, backward);
  straightTo(x, y, backward, breakdist, breakang, yspeed, hspeed);

}

void swingOnLeftAndStraightTo(float x, float y, bool backward = false, float breakdist = 3, float breakang = 5, float yspeed = ymaxSpeed, float hspeed = amaxSpeed, float ykp = YKP, float ykd = YKD) {

  swingOnLeftToPoint(x, y, breakang, backward);
  straightTo(x, y, backward, breakdist, breakang, yspeed, hspeed);

}

void swingOnRightAndStraightTo(float x, float y, bool backward = false, float breakdist = 3, float breakang = 5, float yspeed = ymaxSpeed, float hspeed = amaxSpeed, float ykp = YKP, float ykd = YKD) {

  swingOnRightToPoint(x, y, breakang, backward);
  straightTo(x, y, backward, breakdist, breakang, yspeed, hspeed);

}

////////////////////////////////////////////////////////////////////////////////////////////////
//    _____  ____ _______ ____    ______ _    _ _   _  _____ _______ _____ ____  _   _  _____ 
//   / ____|/ __ \__   __/ __ \  |  ____| |  | | \ | |/ ____|__   __|_   _/ __ \| \ | |/ ____|
//  | |  __| |  | | | | | |  | | | |__  | |  | |  \| | |       | |    | || |  | |  \| | (___  
//  | | |_ | |  | | | | | |  | | |  __| | |  | | . ` | |       | |    | || |  | | . ` |\___ \ 
//  | |__| | |__| | | | | |__| | | |    | |__| | |\  | |____   | |   _| || |__| | |\  |____) |
//   \_____|\____/  |_|  \____/  |_|     \____/|_| \_|\_____|  |_|  |_____\____/|_| \_|_____/ 
////////////////////////////////////////////////////////////////////////////////////////////////
enum TurnType {spot, leftswing, rightswing, noturn};

void moveTo(float x, float y, bool backward = false, float breaklength = 8, float turnmargin = 60, TurnType turnType = spot, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = HKP) {
  if (backward) {
    move.bto(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength, turnmargin);
  }

  if (not backward) {
    if (turnType == spot) {
      move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength, turnmargin, 0);
    }

    if (turnType == leftswing) {
      move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength, turnmargin, 1);
    }

    if (turnType == rightswing) {
      move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength, turnmargin, 2);
    }
  }

}

void goTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 3, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = HKP) {

  facePoint(x, y, breakang, backward);
  moveTo(x, y, backward, breaklength, breakang, spot, yspeed, hspeed, ykP, hkP);


}

void swingOnLeftAndGoTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 10, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = HKP) {
  move.swingOnLefttoPoint(x, y, amaxSpeed, breakang, backward);

  if (backward) {
    move.bto(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

  if (not backward) {
    move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

}

void swingOnRightAndGoTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 10, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = HKP) {
  move.swingOnRighttoPoint(x, y, amaxSpeed, breakang, backward);

  if (backward) {
    move.bto(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

  if (not backward) {
    move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

}

///////////////////////////////////////////////////////////////////////
//   ____   ____   ____  __  __ ______ _____            _   _  _____
//  |  _ \ / __ \ / __ \|  \/  |  ____|  __ \     /\   | \ | |/ ____|
//  | |_) | |  | | |  | | \  / | |__  | |__) |   /  \  |  \| | |  __
//  |  _ <| |  | | |  | | |\/| |  __| |  _  /   / /\ \ | . ` | | |_ |
//  | |_) | |__| | |__| | |  | | |____| | \ \  / ____ \| |\  | |__| |
//  |____/ \____/ \____/|_|  |_|______|_|  \_\/_/    \_\_| \_|\_____|
///////////////////////////////////////////////////////////////////////

void boomerangTo(
    float x, float y, float target_heading,
    bool backward = false,
    float breaklength = 3,
    float lead = 0.6, // Carrot point pullback = lead * distance + setback
    float setback = 5, //Setback is a minimum distance from the target point at which the robot will start to prioritize heading like crazy
    float min_voltage = 0, //For exiting at high speed and motion chaining
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3, //The robot switches to final pose correction when within 3 inches of the target. If it stays within 3 inches for 200ms AND heading is within 5 degrees, the movement exits.
    float timeout = 0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD
) {

  move.boomerang(x, y, target_heading, backward,
                 drive_kp, drive_kd,
                 heading_kp, heading_kd,
                 drive_max, heading_max,
                 lead, min_voltage, settle_error,
                 setback, slew, timeout,
                 breaklength);
}

void boomerangToFacePoint(
    float x, float y, 
    float faceX, float faceY,
    bool backward = false,
    float breaklength = 3,
    float lead = 0.6, // Carrot point pullback = lead * distance + setback
    float setback = 0, //Setback is a minimum distance from the target point at which the robot will start to prioritize heading like crazy
    float min_voltage = 0, //For exiting at high speed and motion chaining
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3, //The robot switches to final pose correction when within 3 inches of the target. If it stays within 3 inches for 200ms AND heading is within 5 degrees, the movement exits.
    float timeout = 0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD
) {

  float target_heading = calculateTargetHeading(faceX, faceY, backward, x, y);

  move.boomerang(x, y, target_heading, backward,
                 drive_kp, drive_kd,
                 heading_kp, heading_kd,
                 drive_max, heading_max,
                 lead, min_voltage, settle_error,
                 setback, slew, timeout,
                 breaklength);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//   _____ _   _  _____ ______ _____ _______ _____ ____  _   _   ____   ____   ____  __  __
//  |_   _| \ | |/ ____|  ____|  __ \__   __|_   _/ __ \| \ | | |  _ \ / __ \ / __ \|  \/  |
//    | | |  \| | |    | |__  | |__) | | |    | || |  | |  \| | | |_) | |  | | |  | | \  / |
//    | | | . ` | |    |  __| |  ___/  | |    | || |  | | . ` | |  _ <| |  | | |  | | |\/| |
//   _| |_| |\  | |____| |____| |      | |   _| || |__| | |\  | | |_) | |__| | |__| | |  | |
//  |_____|_| \_|\_____|______|_|      |_|  |_____\____/|_| \_| |____/ \____/ \____/|_|  |_|
///////////////////////////////////////////////////////////////////////////////////////////////////

void _2029BoomerangTo(
    float x, float y, float target_heading,
    bool backward = false,
    float exit_dist = 3,
    float dLead = 15.0,
    float gLead = 0.4,
    float min_voltage = 0,
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3,
    float timeout = 0,
    float close_end_dist = 8.0,
    float close_ghost_dist = 20.0,
    float parallel_dist = 8.0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD
) {

  move.inceptionBoomerang(x, y, target_heading, backward,
                          drive_kp, drive_kd,
                          heading_kp, heading_kd,
                          drive_max, heading_max,
                          dLead, gLead,
                          min_voltage, settle_error,
                          slew, timeout,
                          close_end_dist, close_ghost_dist,
                          parallel_dist, exit_dist);
}

void _2029BoomerangToFacePoint(
    float x, float y,
    float faceX, float faceY,
    bool backward = false,
    float exit_dist = 3,
    float dLead = 15.0,
    float gLead = 0.4,
    float min_voltage = 0,
    float drive_max = ymaxSpeed,
    float heading_max = hmaxSpeed,
    float settle_error = 3,
    float timeout = 0,
    float close_end_dist = 8.0,
    float close_ghost_dist = 20.0,
    float parallel_dist = 8.0,
    float drive_kp = YKP,
    float drive_kd = YKD,
    float heading_kp = HKP,
    float heading_kd = HKD
) {

  float target_heading = calculateTargetHeading(faceX, faceY, backward, x, y);

  move.inceptionBoomerang(x, y, target_heading, backward,
                          drive_kp, drive_kd,
                          heading_kp, heading_kd,
                          drive_max, heading_max,
                          dLead, gLead,
                          min_voltage, settle_error,
                          slew, timeout,
                          close_end_dist, close_ghost_dist,
                          parallel_dist, exit_dist);
}

///////////////////////////////////////////////////////////////////
//    _____ _    _ _______      ________   _______ ____
//   / ____| |  | |  __ \ \    / /  ____| |__   __/ __ \
//  | |    | |  | | |__) \ \  / /| |__       | | | |  | |
//  | |    | |  | |  _  / \ \/ / |  __|      | | | |  | |
//  | |____| |__| | | \ \  \  /  | |____     | | | |__| |
//   \_____|\____/|_|  \_\  \/   |______|    |_|  \____/
///////////////////////////////////////////////////////////////////

void curveTo(float x1, float y1, float x2, float y2, float lookDist = 12, float breakLength = 5, TurnType turnType = noturn, float breakang = 8, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykp = YKP, float hkp = HKP) {
  // Compute the true Bezier control point: cp = p1 * 2 - (p0 + p2) / 2
  // where p0 = current position, p1 = pass-through point, p2 = end point
  float cpX = x1 * 2 - (chassis.x + x2) / 2;
  float cpY = y1 * 2 - (chassis.y + y2) / 2;

  // Face the true control point first based on turnType
  if (turnType == spot) {
    facePoint(cpX, cpY, breakang);
  } else if (turnType == leftswing) {
    swingOnLeftToPoint(cpX, cpY, breakang);
  } else if (turnType == rightswing) {
    swingOnRightToPoint(cpX, cpY, breakang);
  }

  move.CurveTo(x1, y1, x2, y2, yspeed, hspeed, ykp, hkp, slew, lookDist, breakLength);
}

////////////////////////////////////////////////////////////////////////////////////
//    _______          ________ ______ _____    _______ _    _ _____  _   _  _____ 
//   / ____\ \        / /  ____|  ____|  __ \  |__   __| |  | |  __ \| \ | |/ ____|
//  | (___  \ \  /\  / /| |__  | |__  | |__) |    | |  | |  | | |__) |  \| | (___  
//   \___ \  \ \/  \/ / |  __| |  __| |  ___/     | |  | |  | |  _  /| . ` |\___ \ 
//   ____) |  \  /\  /  | |____| |____| |         | |  | |__| | | \ \| |\  |____) |
//  |_____/    \/  \/   |______|______|_|         |_|   \____/|_|  \_\_| \_|_____/ 
////////////////////////////////////////////////////////////////////////////////////                                                                  

void sweepOnLeft(float heading, float sweepRatio, float breakang = 8) {
  sweep_ratio = sweepRatio;
  move.swingOnLeft(heading, amaxSpeed, breakang);
  sweep_ratio = 0;
}

void sweepOnRight(float heading, float sweepRatio, float breakang = 8) {
  sweep_ratio = sweepRatio;
  move.swingOnRight(heading, amaxSpeed, breakang);
  sweep_ratio = 0;
}

void sweepOnLeftToPoint(float x, float y, float sweepRatio, float breakang = 8, bool backward = false) {
  sweep_ratio = sweepRatio;
  move.swingOnLefttoPoint(x, y, amaxSpeed, breakang, backward);
  sweep_ratio = 0;
}

void sweepOnRightToPoint(float x, float y, float sweepRatio, float breakang = 8, bool backward = false) {
  sweep_ratio = sweepRatio;
  move.swingOnRighttoPoint(x, y, amaxSpeed, breakang, backward);
  sweep_ratio = 0;
}

/////////////////////////////////////
//            _____   _____  _____ 
//      /\   |  __ \ / ____|/ ____|
//     /  \  | |__) | |    | (___  
//    / /\ \ |  _  /| |     \___ \ 
//   / ____ \| | \ \| |____ ____) |
//  /_/    \_\_|  \_\\_____|_____/ 
/////////////////////////////////////

void arcTo(float x1, float y1, float x2, float y2, float speed = ymaxSpeed, float breakLength = 5, bool backward = false, float arcCtKP = ARC_CT_KP, float arcHdgKP = ARC_HDG_KP, float alignMargin = 8) {
  move.arc(x1, y1, x2, y2, speed, breakLength, arcCtKP, arcHdgKP, alignMargin, backward, amaxSpeed);
}

///////////////////////////////////////////////////////
//   _____ _____ _____    __  __  ____  _____   _____
//  |  __ \_   _|  __ \  |  \/  |/ __ \|  __ \ / ____|
//  | |__) || | | |  | | | \  / | |  | | |  | | (___
//  |  ___/ | | | |  | | | |\/| | |  | | |  | |\___ \
//  | |    _| |_| |__| | | |  | | |__| | |__| |____) |
//  |_|   |_____|_____/  |_|  |_|\____/|_____/|_____/
///////////////////////////////////////////////////////

float oldHKP = HKP;
float oldHKD = HKD;

float oldYKP = YKP;
float oldYKI = YKI;
float oldYKD = YKD;

float oldARC_CT_KP = ARC_CT_KP;
float oldARC_HDG_KP = ARC_HDG_KP;

void modifyAngularPID(float newTKP, float newTKD) {
  turnPID.scalePID = false;
  turnPID.setFallbackConstants(newTKP, newTKD);
}

void modifyHeadingPID(float newHKP, float newHKD) {
  HKP = newHKP;
  HKD = newHKD;
}

void modifyLateralPID(float newYKP, float newYKD) {
  YKP = newYKP;
  YKD = newYKD;
}

void modifyArcGains(float newCtKP, float newHdgKP) {
  ARC_CT_KP = newCtKP;
  ARC_HDG_KP = newHdgKP;
}

void revertToOriginalPIDs() {
  turnPID.scalePID = true;

  HKP = oldHKP;
  HKD = oldHKD;

  YKP = oldYKP;
  YKI = oldYKI;
  YKD = oldYKD;

  ARC_CT_KP = oldARC_CT_KP;
  ARC_HDG_KP = oldARC_HDG_KP;
}

/////////////////////////////////////////////////////////////////////////////
//           _    _ _______ ____  _   _  _____   _    _ ______ _____  ______ 
//      /\  | |  | |__   __/ __ \| \ | |/ ____| | |  | |  ____|  __ \|  ____|
//     /  \ | |  | |  | | | |  | |  \| | (___   | |__| | |__  | |__) | |__   
//    / /\ \| |  | |  | | | |  | | . ` |\___ \  |  __  |  __| |  _  /|  __|  
//   / ____ \ |__| |  | | | |__| | |\  |____) | | |  | | |____| | \ \| |____ 
//  /_/    \_\____/   |_|  \____/|_| \_|_____/  |_|  |_|______|_|  \_\______|
//
/////////////////////////////////////////////////////////////////////////////

//Delayed matchloader drop
bool runDelayedDrop = false;

double dropMatchloaderIn = 1000;
double dropMatchloaderFor = 0;

void delayedDrop(double dropIn, double dropFor = 0) {
  dropMatchloaderIn = dropIn;
  dropMatchloaderFor = dropFor;
  runDelayedDrop = true;
}

double dropX1 = 0, dropY1 = 0, dropDist1 = 0, dropTime1 = 0;
double dropX2 = 0, dropY2 = 0, dropDist2 = 0, dropTime2 = 0;
bool drop1Active = false, drop2Active = false;

void distanceToPointDrop(double x1, double y1, double dist1, double time1, double x2 = 0, double y2 = 0, double dist2 = 0, double time2 = 0) {
  dropX1 = x1; dropY1 = y1; dropDist1 = dist1; dropTime1 = time1;
  drop1Active = true;
  if (dist2 > 0) {
    dropX2 = x2; dropY2 = y2; dropDist2 = dist2; dropTime2 = time2;
    drop2Active = true;
  }
}

int delayedMatchloaderDropFcn() {
  while (true) {
    if (runDelayedDrop) {
      delay(dropMatchloaderIn);
      matchloaderDown();
      if (dropMatchloaderFor > 0) {
        delay(dropMatchloaderFor);
        matchloaderUp();
        dropMatchloaderFor = 0;
      }
    runDelayedDrop = false; 
    }

    if (drop1Active) {
      double dist = sqrt(pow(chassis.x - dropX1, 2) + pow(chassis.y - dropY1, 2));
      if (dist <= dropDist1) {
        matchloaderDown();
        if (dropTime1 > 0) { delay(dropTime1); matchloaderUp(); }
        drop1Active = false;
      }
    }

    if (drop2Active) {
      double dist = sqrt(pow(chassis.x - dropX2, 2) + pow(chassis.y - dropY2, 2));
      if (dist <= dropDist2) {
        matchloaderDown();
        if (dropTime2 > 0) { delay(dropTime2); matchloaderUp(); }
        drop2Active = false;
      }
    }

    vex::this_thread::sleep_for(10);
  }
  return(0);
}

//CONSTANTS ////////////////////////////////////////////////////////////////
double longGoalX = 48;
double longGoalY = -30;

double trueLongGoalX = 48;
double trueLongGoalY = -24.240;

double matchloaderX = 46;
double matchloaderY = -63;

double trueMatchloaderY = -55.6;

void scoreLong(double flip, double time2, double time1 = 400) {
  boomerangTo(flip*longGoalX, trueLongGoalY, 180, true, 8, 0.6, 8);
  // _2029BoomerangTo(flip * longGoalX, trueLongGoalY, 180, true, 8, 12);
  ymaxSpeed = 8;
  driveBackward(20, 180, 0);
  delay(time1);
  hoodUp();
  intakeMode = longGoal;
  face(180, 0);
  delay(time2*0.3);
  move.voldrive(-10,-10);
  delay(time2*0.7);
  ymaxSpeed = 12.7;

}




















void SmallTest(){
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 0;
  bangbangDist = 0;

  chassis.setPos(trueLongGoalX, trueLongGoalY, 180);

  face(-90);
  _2029BoomerangTo(-24, -24, -90, false, 8, 48);
  goTo(-48, -50);
  move.voldrive(0,0);
  delay(5000000);

}

void HookProcedure() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;

  chassis.setPos(0,0,0);

  float originalHeading = chassis.h;

  modifyAngularPID(100, 0);
  swingOnRight(originalHeading+47, 5);
  revertToOriginalPIDs();
  face(originalHeading+7);

}

bool nextmovement = false;

void setnextmovementtrue(vex::axisType axis, double x, double y, double z) {
  nextmovement = true;
}

void arpit() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;
  midDescoreDown();

  hookDown();

  dropMatchloaderIn = 450;
  dropMatchloaderFor = 0;
  runDelayedDrop = true;

  chassis.setPos(13, -45.82,0);

  readWall(1, 1,true);
  readWall(4,4, true);

  moveTo(27, -17, false, 10, 10, spot, 12);

  facePoint(48, -30, 2, true);

  nextmovement = false;
  IMU1.collision(setnextmovementtrue);
  move.voldrive(-8,-8);
  while (not nextmovement) {
    delay(10);
  }

  modifyAngularPID(YKP*1.5, YKD);
  sweepOnRight(180, 0.3, 0);
  revertToOriginalPIDs();
  delay(500);
  move.voldrive(-8, -8);
  intakeMode = longGoal;
  matchloaderUp();
  delay(300);
  face(180,0);
  delay(300);
  move.voldrive(-8,-8);
  delay(700);

  hookDown();
  face(135);
  modifyLateralPID(YKP*5, 0);
  driveForward(6.5, 135);
  face(180);
  modifyLateralPID(YKP, YKD*0.7);
  revertToOriginalPIDs();
  driveBackward(28, 170, 0, 12);


}

void tenB() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;

  intakeMode = intake;
  midDescoreDown();

  hookUp();

  chassis.setPos(-13, -45.82,0);

  readWall(3, 3,true);
  readWall(4,4, true);

  delayedDrop(450, 0);

  moveTo(-22, -24, false, 12, 60);
  matchloaderUp();

  delayedDrop(550, 0);

  ymaxSpeed = 7;
  moveTo(-42, -10, false, 0);
  delay(1000);

  ymaxSpeed = 12.7;
  face(-90);
  delay(100);

  //Back up
  hmaxSpeed = 6;
  drive_min_output = 8;
  move.ptpCosineScaling = false;
  moveTo(-24, -24, true, 12);
  drive_min_output = 0;

  moveTo(-48, -39, true, 6);

  move.ptpCosineScaling = true;
  revertToOriginalPIDs();

  face(180);

  readWall(3, 1);

}

void sevenRush() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  move.ptpCosineScaling = true;

  intakeMode = intake;
  midDescoreDown();

  hookUp();

  chassis.setPos(-13, -45.82,0);

  readWall(3, 3,true);
  readWall(4,4, true);

  delayedDrop(450, 0);

  moveTo(-22, -24, false, 12, 60);

  //To matchloader

  ymaxSpeed = 9;
  facePoint(-matchloaderX, matchloaderY + calculateStraightlineDist(-matchloaderX, matchloaderY)*0.2 + 6);
  boomerangTo(-matchloaderX+2, matchloaderY, 180, false, 8, 0.2, 6);


  // goTo(-44, -48);

  // straightlineTo(-matchloaderX, matchloaderY, false, 8);

  delay(800);

  // why does the boomerang on line 1007 cross the line of the lead (dyk what i mean) and then try to curve back? i am not noticing any wheel slip

  ymaxSpeed = 10;
  _2029BoomerangTo(-longGoalX, longGoalY, 180, true, 12);
  driveBackward(20, 180, 0);
  delay(400);
  hoodUp();
  intakeMode = longGoal;
  delay(1500);

  driveForward(8);
  hoodDown();

  drivePID.minOutput = 8;
  driveBackward(12);

  move.voldrive(0,0);
  delay(50000000);

}






















void tenBRight() {
 
}

void SOLO() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;

  hookUp();

  chassis.setPos(0,0,270);
  //readWall(1, 4, true);
  readWall(4, 3, true);
  intakeMode = intake;
  modifyLateralPID(YKP, 0);
  driveForward(8, chassis.h, 0);
  delay(500);
  revertToOriginalPIDs();
  //voltagedrive(3, 3);
  drivePID.minOutput = 12.7;
  straightTo(46.5, chassis.y, true, 0);
  delay(250);
  drivePID.minOutput = 0;
  modifyLateralPID(YKP*0.8, YKD*1.3);
  straightTo(47.5, -45, true, 3);
  face(180);
  delay(80);
  readWall(1, 3, false);
  revertToOriginalPIDs();
  matchloaderDown();

  straightlineTo(48, -67, false, 0, 5, 6);
  delay(1000);
  
  ymaxSpeed = 8;
  boomerangTo(48, -24, 180, true, 12);
  intakeMode = topStageJamInsurance;
  matchloaderUp();
  move.voldrive(-7, -7);
  intakeMode = longGoal;
  delay(50);
  hoodUp();
  delay(600);
  revertToOriginalPIDs();

  ymaxSpeed = 12.7;
  chassis.setPos(48, -28.982, chassis.h);
  //swingOnRightToPoint(-20, -24, 5, false);
  modifyLateralPID(YKP*0.8, YKD*1.3);
  facePoint(-19, -25);
  ymaxSpeed = 10;
  hoodDown();
  distanceToPointDrop(-19, -25, 24, 1000);
  straightTo(-19, -25, false, 8, 6);
  facePoint(-48, -48);
  revertToOriginalPIDs();
  moveTo(-40, -45, false, 10, 20);
  delay(80);
  face(180);
  readWall(3, 1);
  ymaxSpeed = 8;
  boomerangTo(-48, -28, 180, true, 8);
  move.voldrive(-7, -7);
  hoodUp();
  intakeMode = longGoal;
  matchloaderDown();
  delay(600);
  straightlineTo(-48, -67, false, 0, 5, 6);
  hoodDown();
  intakeMode = intake;
  delay(1000);
  sweepOnLeftToPoint(-20, -20, 0.2, 8, true);
  moveTo(-7, -10, true, 12);
  drop4bar();
  intakeMode = midGoal;
  // moveTo(24, -24, false, 12);
  // matchloaderUp();

  //moveTo(-20, -24, false, 10);
  //intakeMode = intake;

  /*intakeMode = topStageJamInsurance;
  goTo(-46, -48, false);
  face(180);
  readWall(3, 1);

  modifyLateralPID(YKP, YKD*0.5);
  straightlineTo(-48, -24, true, 12, 8, 7);
  revertToOriginalPIDs();

  intakeMode = longGoal;
  move.voldrive(-7,-7);
  delay(250);
  face(180, 0);
  delay(500);
  move.voldrive(-10,-10);
  delay(1000);
  chassis.setPos(-48,-28.982,180);

  matchloaderDown();

  intakeMode = intake;
  ymaxSpeed = 12.7;
  moveTo(-48, -68, false, 0);
  delay(200);
  ymaxSpeed = 4;
  moveTo(-48, -68, false, 0);
  delay(1400);
  ymaxSpeed = 12.7;
  chassis.setPos(chassis.x, -55.34, chassis.h);

  matchloaderUp();
  move.voldrive(-12,-12);
  delay(50);
  boomerangTo(-26, -26, -135, true);

  ymaxSpeed = 7;
  moveTo(-8,-8, true, 16);

  move.voldrive(0,0);
  intakeMode = midGoal;
  delay(100000);


  matchloaderUp();*/

}

void rightSplit() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;

  hookUp();

  chassis.setPos(0,0,90);
  readWall(1, 2, true);
  readWall(4, 1, true);

  intakeMode = intake;
  straightTo(46.5, chassis.y);
  matchloaderDown();

  face(180);
  readWall(1, 3);

  straightlineTo(48, -65, false, 12, 5, 6);
  delay(900);

  modifyLateralPID(YKP, YKD*0.5);
  intakeMode = topStageJamInsurance;
  moveTo(48, -24, true, 12);
  matchloaderUp();

  intakeMode = longGoal;
  move.voldrive(-10,-10);
  delay(500);
  face(180, 0);
  delay(500);
  move.voldrive(-10, -10);
  delay(1000);

  chassis.setPos(48,-28.982,chassis.h);

  intakeMode = intake;
  driveForward(12);

  dropMatchloaderIn = 1000;
  dropMatchloaderFor = 0;
  runDelayedDrop = true;
  goTo(24, -24);
  matchloaderUp();

  goTo(12,-12, false, 15);
  move.voldrive(3,3);
  midDescoreUp();
  intakeMode = spit;
  delay(1000);

  moveTo(38, -28, true, 10);
  face(0);
  hookDown();
  driveForward(10);
  delay(5000000);
}

void twoMidGoals() {
  tenB();

  matchloaderDown();
  ymaxSpeed = 6;
  straightlineTo(-46, -60, false, 15);
  delay(1000);
  chassis.setPos(chassis.x, -55.6, chassis.h);
  ymaxSpeed = 12.7;

  intakeMode = antiSpill;
  delay(100);
  drop4bar();
  _2029BoomerangTo(-10, -13, -135, true, 8);

  matchloaderUp();
  move.voldrive(-3,-3);
  delay(250);
  //intakeMode = onlyTopStage;
  move.voldrive(0,0);
  delay(1000);

  intakeMode = antiSpill;

  modifyAngularPID(10, 0);
  move.bigTime = 150;
  swingOnLeft(180);

  move.bigTime = 150;
  revertToOriginalPIDs();
  swingOnLeftToPoint(8, -8);

  ymaxSpeed = 8;
  modifyLateralPID(YKP, YKD*0.8);
  driveForward(18, chassis.h, 0);
  delay(500);
  ymaxSpeed = 12.7;

  move.bigTime = 300;
  swingOnLeft(-45, 0);
  delay(500);

  midDescoreUp();
  intakeMode = spit;
  move.voldrive(-1,-1);
  delay(500);
  move.voldrive(0,0);
  delay(500);

  swingOnLeft(45);
  driveBackward(5);
  facePoint(-3, 0, 8, true);

  drivePID.minOutput = 8;
  driveBackward(24, 135, 0);
  delay(300);
  move.voldrive(0,0);
  delay(500);

  face(135, 3);
  driveForward(0.5, chassis.h, 0);
  delay(500);


  delay(500000000);
}

void sixRush() {
  //tenB();
  leftdrive.stop(hold);
  rightdrive.stop(hold);
  delay(500000);
}
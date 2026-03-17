#include "vex.h"
#include "movement.h"
#include "threads.h"
#include <climits>
#include <cstdlib>
#include <utility>
using namespace vex;
#include <cmath>
#include "math.h"
#include <vector>
#include "globals.h"

float endtime = 0;
float starttime = 0;

// ANGULAR PID CONSTANTS
float  TKP = 0.316; //2; //0.29
float  TKI = 0.01; //0.01
float  TKD = 2; //19.25; //1.9

// HEADING PID CONSTANTS
float HKP = TKP;
float HKD = TKD;

// LINEAR PID CONSTANTS
float YKP = 1.3;
float YKI = 0;
float YKD = 10;

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

  intakeMode = coasting;
  
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
void face(float heading, float breakang = 8) {

  move.face(heading, TKP, TKI, TKD, amaxSpeed, breakang);

}

void facePoint(float x, float y, float breakang = 8, bool backward = false) {

  move.facePoint(x, y, TKP, TKI, TKD, amaxSpeed, breakang, backward);

}

void swingOnRight(float heading, float breakang = 8, bool backward = false) {

  move.swingOnRight(heading, TKP, TKI, TKD, amaxSpeed, breakang);

}

void swingOnLeft(float heading, float breakang = 8, bool backward = false) {

  move.swingOnLeft(heading, TKP, TKI, TKD, amaxSpeed, breakang);

}

void swingOnRightToPoint(float x, float y, float breakang = 8, bool backward = false) {

  move.swingOnRighttoPoint(x, y, TKP, TKI, TKD, amaxSpeed, breakang, backward);

}

void swingOnLeftToPoint(float x, float y, float breakang = 8, bool backward = false) {

  move.swingOnLefttoPoint(x, y, TKP, TKI, TKD, amaxSpeed, breakang, backward);

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

void moveTo(float x, float y, bool backward = false, float breaklength = 8, float turnmargin = 40, TurnType turnType = spot, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = TKP, float tkD = TKD) {
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

void goTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 3, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = TKP, float tkD = TKD) {

  facePoint(x, y, breakang, backward);
  moveTo(x, y, backward, breaklength, breakang, spot, yspeed, hspeed, ykP, hkP);


}

void swingOnLeftAndGoTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 10, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = TKP, float tkD = TKD) {
  move.swingOnLefttoPoint(x, y, TKP, TKI, TKD, amaxSpeed, breakang, backward);

  if (backward) {
    move.bto(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

  if (not backward) {
    move.to(x, y, yspeed, hspeed, ykP, hkP, slew, breaklength);
  }

}

void swingOnRightAndGoTo(float x, float y, bool backward = false, float breaklength = 8, float breakang = 10, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykP = YKP, float hkP = TKP, float tkD = TKD) {
  move.swingOnRighttoPoint(x, y, TKP, TKI, TKD, amaxSpeed, breakang, backward);

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
    float heading_kp = TKP,
    float heading_kd = TKD
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
    float heading_kp = TKP,
    float heading_kd = TKD
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

///////////////////////////////////////////////////////////////////
//    _____ _    _ _______      ________   _______ ____
//   / ____| |  | |  __ \ \    / /  ____| |__   __/ __ \
//  | |    | |  | | |__) \ \  / /| |__       | | | |  | |
//  | |    | |  | |  _  / \ \/ / |  __|      | | | |  | |
//  | |____| |__| | | \ \  \  /  | |____     | | | |__| |
//   \_____|\____/|_|  \_\  \/   |______|    |_|  \____/
///////////////////////////////////////////////////////////////////

void curveTo(float x1, float y1, float x2, float y2, float lookDist = 12, float breakLength = 5, TurnType turnType = noturn, float breakang = 8, float yspeed = ymaxSpeed, float hspeed = hmaxSpeed, float ykp = YKP, float hkp = TKP) {
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
  move.swingOnLeft(heading, TKP, TKI, TKD, amaxSpeed, breakang);
  sweep_ratio = 0;
}

void sweepOnRight(float heading, float sweepRatio, float breakang = 8) {
  sweep_ratio = sweepRatio;
  move.swingOnRight(heading, TKP, TKI, TKD, amaxSpeed, breakang);
  sweep_ratio = 0;
}

void sweepOnLeftToPoint(float x, float y, float sweepRatio, float breakang = 8, bool backward = false) {
  sweep_ratio = sweepRatio;
  move.swingOnLefttoPoint(x, y, TKP, TKI, TKD, amaxSpeed, breakang, backward);
  sweep_ratio = 0;
}

void sweepOnRightToPoint(float x, float y, float sweepRatio, float breakang = 8, bool backward = false) {
  sweep_ratio = sweepRatio;
  move.swingOnRighttoPoint(x, y, TKP, TKI, TKD, amaxSpeed, breakang, backward);
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
  move.arc(x1, y1, x2, y2, speed, breakLength, arcCtKP, arcHdgKP, alignMargin, backward, TKP, TKI, TKD, amaxSpeed);
}

///////////////////////////////////////////////////////
//   _____ _____ _____    __  __  ____  _____   _____
//  |  __ \_   _|  __ \  |  \/  |/ __ \|  __ \ / ____|
//  | |__) || | | |  | | | \  / | |  | | |  | | (___
//  |  ___/ | | | |  | | | |\/| | |  | | |  | |\___ \
//  | |    _| |_| |__| | | |  | | |__| | |__| |____) |
//  |_|   |_____|_____/  |_|  |_|\____/|_____/|_____/
///////////////////////////////////////////////////////

float oldTKP = TKP;
float oldTKI = TKI;
float oldTKD = TKD;

float oldHKP = HKP;
float oldHKD = HKD;

float oldYKP = YKP;
float oldYKI = YKI;
float oldYKD = YKD;

float oldARC_CT_KP = ARC_CT_KP;
float oldARC_HDG_KP = ARC_HDG_KP;

void modifyAngularPID(float newTKP, float newTKD) {
  TKP = newTKP;
  TKD = newTKD;
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
  TKP = oldTKP;
  TKI = oldTKI;
  TKD = oldTKD;

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

void SmallTest(){
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 0;
  bangbangDist = 0;

  chassis.setPos(-24,0,180);
  
  arcTo(0, -36, 24, 0, 8);

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

  modifyAngularPID(TKP*100, TKD*0);
  swingOnRight(originalHeading+47, 5);
  revertToOriginalPIDs();
  // driveForward(6);
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

void tenB(bool hook) {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;
  midDescoreDown();

  hookDown();

  chassis.setPos(-13, -45.82,0);

  readWall(3, 3,true);
  readWall(4,4, true);

  delayedDrop(400, 0);

  moveTo(-27, -17, false, 10, 10, spot, 12);
  matchloaderUp();

  delayedDrop(280, 2000);
  moveTo(-48, -4, false, 14);
  delay(0);

  hmaxSpeed = 6;
  drivePID.minOutput = 8;
  moveTo(-24, -24, true, 12);
  drivePID.minOutput = 0;
  moveTo(-39, -47, true, 12);

  swingOnLeft(180);
  readWall(3, 1);
  revertToOriginalPIDs();
  modifyLateralPID(YKP, YKD*0.7);
  moveTo(-46, -18, true, 13, 3);
  matchloaderUp();
  revertToOriginalPIDs();

  intakeMode = longGoal;
  move.voldrive(-10,-10);
  delay(500);
  face(180, 0);
  delay(500);
  move.voldrive(-10, -10);
  delay(500);
  chassis.setPos(-48,-18.62,180);

  if (hook) {
    hookDown();
    face(135);
    modifyLateralPID(YKP*5, 0);
    driveForward(6.5, 135);
    face(180);
    modifyLateralPID(YKP, YKD*0.7);
    revertToOriginalPIDs();
    driveBackward(28, 170, 0, 12);
  }
}

void tenBRight() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;
  midDescoreDown();

  hookDown();

  chassis.setPos(13, -45.82,0);

  readWall(1, 1,true);
  readWall(4,4, true);

  delayedDrop(400, 0);

  moveTo(27, -17, false, 10, 10, spot, 12);
  matchloaderUp();

  delayedDrop(280, 2000);
  moveTo(48, -4, false, 14);
  delay(0);

  hmaxSpeed = 6;
  drivePID.minOutput = 8;
  moveTo(24, -24, true, 12);
  drivePID.minOutput = 0;
  moveTo(39, -40, true, 12);

  swingOnRight(180);
  delay(100);
  readWall(1, 3);
  modifyLateralPID(YKP, YKD*0.7);
  facePoint(48, 24, 3, true);
  moveTo(48, -20, true, 13, 3);
  matchloaderUp();
  revertToOriginalPIDs();

  intakeMode = longGoal;
  move.voldrive(-10,-10);
  delay(500);
  face(180, 0);
  delay(500);
  move.voldrive(-10, -10);
  delay(700);
  chassis.setPos(-46,-18.62,180);

  hookDown();
  face(135);
  modifyLateralPID(YKP*5, YKD*0);
  driveForward(6.5, 135);
  face(180);
  modifyLateralPID(YKP, YKD*0.7);
  revertToOriginalPIDs();
  modifyLateralPID(YKP*4, YKD*0.8);
  driveBackward(25, 170, 0, 12);
  delay(500);
  leftdrive.stop(hold);
  rightdrive.stop(hold);

}

void hookInsurance(bool reset = true) {
  hookUp();

  matchloaderDown();

  if (reset) {
    face(180, 3);
    readWall(4, 2);
    readWall(3, 1, true);


    intakeMode = intake;
    boomerangTo(-48, -40, -135, false);

    face(180, 0);
    delay(400);
    readWall(3, 1);
  }

  intakeMode = intake;
  ymaxSpeed = 4;
  moveTo(-48, -68, false, 0);
  delay(1500);
  ymaxSpeed = 12.7;
  chassis.setPos(chassis.x, -55.34, chassis.h);

  matchloaderUp();
  move.voldrive(-12,-12);
  delay(50);
  ymaxSpeed = 8;
  boomerangTo(-24, -24, -135, true, 8, 0.9);

  ymaxSpeed = 5;
  moveTo(-8,-8, true, 12);

  move.voldrive(0,0);
  intakeMode = midGoal;
  delay(2000);


  matchloaderUp();
}

void SOLO() {
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
  delay(600);

  chassis.setPos(48,-28.982,chassis.h);

  intakeMode = intake;
  face(-90);
  readWall(4, 3);

  ymaxSpeed = 12.7;

  // delayedDrop(250, 0);
  // moveTo(24, -24, false, 12);
  // matchloaderUp();

  delayedDrop(800, 0);
  moveTo(-24, -24, false, 12);

  intakeMode = topStageJamInsurance;
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


  matchloaderUp();



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
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;
  midDescoreDown();

  hookUp();

  chassis.setPos(-13, -45.82,0);

  readWall(3, 3,true);
  readWall(4,4, true);

  delayedDrop(400, 0);

  moveTo(-27, -17, false, 10, 10, spot, 12);
  matchloaderUp();

  delayedDrop(280, 4000);
  moveTo(-48, -5, false, 10);

  hmaxSpeed = 6;
  drivePID.minOutput = 8;
  moveTo(-24, -24, true, 12);
  drivePID.minOutput = 0;
  moveTo(-50, -40, true, 12);

  face(180);
  readWall(3, 1);

  ymaxSpeed = 6;
  straightlineTo(-48, -68, false, 15);
  delay(500);
  ymaxSpeed = 12.7;

  //TO OTHER SIDEEEEEEE

  sweepOnLeftToPoint(20, -20, 0.2, 8, true);
  moveTo(20, -20, true, 8);

  spitIn = 200;
  intakeMode = spit;
  facePoint(0,0);
  modifyLateralPID(YKP, YKD*0.5);
  driveForward(24, -45, 16);
  revertToOriginalPIDs();
  delay(300);
  spitIn = 0;

  intakeMode = intake;

  move.bigTime = 100;
  sweepOnLeftToPoint(-36, -6, 0.05, 8, true);
  modifyLateralPID(YKP*3, 0);
  moveTo(-48, -8, true, 25);
  move.bigTime = 300;
  revertToOriginalPIDs();

  face(170);
  delay(100);

  hookDown();
  driveForward(32, 170);
  hookUp();

  face(180);
  readWall(3, 1);
  readWall(4, 2);

  ymaxSpeed = 10;
  facePoint(-10,-11, 5, true);
  moveTo(-10, -11, true, 10);
  move.voldrive(-2, -2);
  intakeMode = midGoal;
  delay(500000);

}

void sixRush() {
  tenB();
  leftdrive.stop(hold);
  rightdrive.stop(hold);
  delay(500000);
}

void sixRushWithInsurance() {
  tenB();

  while (chassis.h > 170) {
    delay(10);
  }

  hookInsurance();
}

void sixRushSplitQUAL() {
  tenB();
  delay(700);
  hookInsurance();
}

void sixRushSplit() {
  tenB(false);
  delay(700);
  hookInsurance(false);
  ymaxSpeed = 12.7;
  moveTo(-36, -27, false,10);
  face(180);
  hookDown();
  driveBackward(24);
  leftdrive.stop(hold);
  rightdrive.stop(hold);
}

void leaveGoal() {
  driveForward(25, 180, 10, 12);
}




void hkMovement() {
  auto_settings();
  ymaxSpeed = 12.7;
  hmaxSpeed = ymaxSpeed;
  slew = 12.7;
  bangbangDist = 0;
  intakeMode = intake;
  
  chassis.setPos(24, 60, 0);
  matchloaderDown();

  delay(500);

  boomerangTo(27, 40, -70, true, 15, 0.8);

  drivePID.minOutput = 8;
  move.turndisablelength = 5;
  moveTo(34, 36, false, 8, 120);
  drivePID.minOutput = 0;

  modifyAngularPID(TKP*0.5, TKD);
  driveForward(24, 180);


  leftdrive.stop(brake);
  rightdrive.stop(brake);

}
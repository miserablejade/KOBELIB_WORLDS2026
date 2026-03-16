#include "vex.h"
#include "threads.h"
#include "movement.h"
#include "driver.h"
#include "auton.h"
#include <cstdio>
#include <ios>
using namespace vex;
#include <cmath>
#include "math.h"

brain Brain;
controller Controller = controller(primary);
std::string currentAuton;

// Rollers
motor intakeRollers = motor(PORT11, ratio6_1, false);
motor secondRollers = motor(PORT12, ratio18_1, true);
motor scoreRoller = motor(PORT12, ratio18_1, true);

// Chassis
motor fl = motor(PORT13, ratio6_1, true);
motor ml = motor(PORT14, ratio6_1, true);
motor bl = motor(PORT15, ratio6_1, true);

motor fr = motor(PORT18, ratio6_1, false);
motor mr = motor(PORT19, ratio6_1, false);
motor br = motor(PORT20, ratio6_1, false);

motor_group leftdrive = motor_group(fl, ml, bl);
motor_group rightdrive = motor_group(fr, mr, br);

// Odometry and Heading
inertial IMU1 = inertial(PORT17);
inertial IMU2 = inertial(PORT17);

rotation TX = rotation(PORT6, false);
rotation TY = rotation(PORT21, true);

//Tracker wheel effective diameter
double txD = 2.02;
double tyD = 2.03;

//Tracker wheel offsets
double txOffset = 2.7422;
double tyOffset = -1.8884;

//IMU Multiplier
double imu_multiplier = 0.9938888889;

//Wall reading distance sensors
distance DistSensor1 = distance(PORT8);
distance DistSensor2 = distance(PORT2);
distance DistSensor3 = distance(PORT16);
distance DistSensor4 = distance(PORT9);

//DRIVE CHOICE
bool curvatureDrive = false;
bool arcadeDrive = true;

//Mag distance sensors
distance OtwDistSensor = distance(PORT8);
distance ChamberDistSensor = distance(PORT15);

//Color sensor
optical OtwColorSensor = optical(PORT11);

//Pistons
digital_out hook1 = digital_out(Brain.ThreeWirePort.E);
digital_out hook2 = digital_out(Brain.ThreeWirePort.D);
digital_out hood = digital_out(Brain.ThreeWirePort.A);
digital_out matchloader = digital_out(Brain.ThreeWirePort.D);
digital_out midDescore = digital_out(Brain.ThreeWirePort.C);
digital_out intakeLift = digital_out(Brain.ThreeWirePort.B);
digital_out odomLift = digital_out(Brain.ThreeWirePort.F);

//Timers
timer R2Time;

bool l1Pressed_first = false;
bool l1Pressed_second = false;
bool l1released = false;

bool l2Pressed_first = false;
bool l2Pressed_second = false;
bool l2released = false;

bool r1Pressed_first = false;
bool r1Pressed_second = false;
bool r1released = false;

bool r2Pressed_first = false;
bool r2Pressed_second = false;
bool r2released = false;

bool noOtherButtonPressed = true;

int controllerButtonMonitoringFcn() {
  bool prevL1 = false, prevL2 = false, prevR1 = false, prevR2 = false;

  while (true) {
    bool curL1 = Controller.ButtonL1.pressing();
    bool curL2 = Controller.ButtonL2.pressing();
    bool curR1 = Controller.ButtonR1.pressing();
    bool curR2 = Controller.ButtonR2.pressing();

    // Classify each button on its rising edge — stays stable until released
    if (curL1 && !prevL1) {
      bool combo = l2Pressed_first || r1Pressed_first || r2Pressed_first;
      l1Pressed_first = !combo; l1Pressed_second = combo;
    }
    if (!curL1) { l1Pressed_first = false; l1Pressed_second = false; }

    if (curL2 && !prevL2) {
      bool combo = l1Pressed_first || r1Pressed_first || r2Pressed_first;
      l2Pressed_first = !combo; l2Pressed_second = combo;
    }
    if (!curL2) { l2Pressed_first = false; l2Pressed_second = false; }

    if (curR1 && !prevR1) {
      bool combo = l1Pressed_first || l2Pressed_first || r2Pressed_first;
      r1Pressed_first = !combo; r1Pressed_second = combo;
    }
    if (!curR1) { r1Pressed_first = false; r1Pressed_second = false; }

    if (curR2 && !prevR2) {
      bool combo = l1Pressed_first || l2Pressed_first || r1Pressed_first;
      r2Pressed_first = !combo; r2Pressed_second = combo;
    }
    if (!curR2) { r2Pressed_first = false; r2Pressed_second = false; }

    noOtherButtonPressed = !(curL1 || curL2 || curR1 || curR2);

    prevL1 = curL1; prevL2 = curL2; prevR1 = curR1; prevR2 = curR2;
    vex::this_thread::sleep_for(5);
  }
  return 0;
}

bool midDescoreIsUp;

void midDescoreUp() {
  midDescore.set(1);
  midDescoreIsUp = true;
}

void midDescoreDown() {
  midDescore.set(0);
  midDescoreIsUp = false;
}

void midDescoreToggle() {
  if (midDescoreIsUp) {
    midDescoreDown();
  } else {
    midDescoreUp();
  }
}

void hook1Up() {
  hook1.set(0);
}

void hook1Down() {
  hook1.set(1);
}

void hook2Up() {
  hook2.set(1);
}

void hook2Down() {
  hook2.set(0);
}

void hoodUp() {
  hood.set(0);
}

void hoodDown() {
  hood.set(1);
}

void matchloaderUp() {
  matchloader.set(0);
}

void matchloaderDown() {
  matchloader.set(1);
}

void coastAllRollers() {
  intakeRollers.stop(coast);
  secondRollers.stop(coast);
  scoreRoller.stop(coast);
}

void intakeFcn() {
  intakeRollers.spin(forward, 100, percent);
  secondRollers.spin(forward, 100, percent);
}

void longGoalFcn() {
  intakeLift.set(false);
  intakeRollers.spin(forward, 100, percent);
  secondRollers.spin(forward, 100, percent);
  scoreRoller.spin(forward, 100, percent);
}

void spitFcn() {
  intakeRollers.spin(reverse, 100, percent);
  secondRollers.spin(reverse, 100, percent);
}


double lowGoalSpeed = 20;
void lowGoalFcn() {
    intakeRollers.spin(reverse, lowGoalSpeed, percent);
    secondRollers.spin(reverse, 100, percent);
}

void midGoalFcn(int speed) {
  intakeLift.set(true);
  hood.set(true);
  intakeRollers.spin(forward, 100, percent);
  secondRollers.spin(forward, 100, percent);
  scoreRoller.spin(forward, speed, percent);
}

//Intake monitor variables
bool blockOtw = false;

bool readingRed = false;
bool readingBlue = false;

bool blockOtwIsRed = false;
bool blockOtwIsBlue = false;

bool blockInChamber = false;

//Color sensor threshold constants
double redThresholdUpper = 12;
double redThresholdLower = 0;
double blueThresholdUpper = 230;
double blueThresholdLower = 215;

//Distance sensor threshold
double magDistanceThreshold = 2;
double chamberDistanceThreshold = 1.5;

//Reverse amount for color sort
double reverseTimeLongSort = 300;
double reverseTimeMidSort = 500;

//Chamber purge time
double chamberPurgeTime = 200;

//Purge oppo colors during store
bool purgingDuringStore = true;

//Intake control
IntakeMode intakeMode = coasting;

//Color sort toggles
ColorSortMode colorSortMode = none;

//hook1 mode
bool hook1Mode = false;

//Mid goal speed
int midGoalSpeed = 100;

//Low goal delay
double lowGoalIn = 0;

//Anti jam
bool intakeAntiJamEnabled = true;

//Delayed goal prime
double startPrimingAfter = 0;
bool startPrimingLowGoal = false;
double spitIn = 0;

bool suspendScoreRoller = false;
bool slowLongGoal = false;

bool firstLongGoal = false;

// Intake Control Task
int magControlFcn() {
  while (true) {
    intakeRollers.setVelocity(100, percent);
    secondRollers.setVelocity(100, percent);
    switch (intakeMode) {
      case (intake):
        if (purgingDuringStore && (colorSortMode == teamRedColorSort) && blockOtwIsBlue) { //Used to be && !blockInChamber
          //If a bad block comes in during storing, purge it (RED TEAM)
          secondRollers.stop(hold);
          intakeRollers.stop(hold);
          scoreRoller.spin(reverse, 100, percent);
          Controller.rumble(".");
          delay(reverseTimeLongSort);

        } else if (purgingDuringStore && (colorSortMode == teamBlueColorSort) && blockOtwIsRed) { //Used to be && !blockInChamber
          //If a bad block comes in during storing, purge it (BLUE TEAM)
          secondRollers.stop(hold);
          intakeRollers.stop(hold);
          scoreRoller.spin(reverse, 100, percent);
          Controller.rumble(".");
          delay(reverseTimeLongSort);
        } else {
          intakeFcn();
          scoreRoller.stop(hold);
        }

        firstLongGoal = true;
        break;

      case (spit):
        if (spitIn > 0) {
          delay(spitIn);
          spitIn = 0;
        }
        spitFcn();

        firstLongGoal = true;
        break;

      case (lowGoal):
        if (not suspendScoreRoller) {
          scoreRoller.spin(reverse, 100, percent);
        } else {
          scoreRoller.stop(coast);
        }

        if (intakeRollers.velocity(rpm) == 0) {
          intakeRollers.spin(forward, 100, percent);
          delay(50);
          lowGoalFcn();
          delay(50);

        }
        lowGoalFcn();

        firstLongGoal = true;
        break;

      case (coasting):
        coastAllRollers();
        break;

      case (longGoal):
        // if (firstLongGoal) {
        //   spitFcn();
        //   scoreRoller.stop(hold);
        //   delay(100);
        //   scoreRoller.spin(forward, 100, percent);
        //   delay(100);
        //   intakeFcn();
        //   scoreRoller.spin(forward, 100, percent);
        //   delay(200);
        

        if (blockInChamber) {
          spitFcn();
          scoreRoller.stop(hold);
          delay(100);
          scoreRoller.spin(forward, 100, percent);
          delay(100);
          intakeFcn();
          scoreRoller.spin(forward, 100, percent);
          delay(200);
          

        } else {
          intakeFcn();
          scoreRoller.spin(forward, 100, percent);
        }

        if (slowLongGoal) {
          intakeFcn();
          scoreRoller.spin(forward, 50, percent);
        }

        if (intakeRollers.velocity(rpm) < 20) {
          intakeRollers.spin(reverse, 100, percent);
          delay(200);
          scoreRoller.spin(forward, 100, percent);
          intakeRollers.spin(forward, 100, percent);
          delay(200);
        }

        if (secondRollers.velocity(rpm) < 10) {
          intakeRollers.spin(forward, 100, percent);
          scoreRoller.spin(forward, 100, percent);

          secondRollers.spin(reverse, 100, percent);
          delay(200);
          secondRollers.spin(forward, 100, percent);
          delay(200);
        }

        if (scoreRoller.velocity(rpm) < 10) {
          intakeRollers.spin(forward, 100, percent);
          secondRollers.spin(forward, 100, percent);

          scoreRoller.spin(reverse, 100, percent);
          delay(200);
          scoreRoller.spin(forward, 100, percent);
          delay(200);
        }

        firstLongGoal = false;
        break;

      case (midGoal):
        if (midDescoreIsUp) {
          midDescoreDown();
          delay(300);
        }

        if (colorSortMode == teamRedColorSort && blockOtwIsBlue) {
          scoreRoller.spin(forward, 100, percent);
          intakeFcn();
          Controller.rumble(".");
          delay(reverseTimeMidSort);

        } else if (colorSortMode == teamBlueColorSort && blockOtwIsRed) {
          scoreRoller.spin(forward, 100, percent);
          intakeFcn();
          Controller.rumble(".");
          delay(reverseTimeMidSort);

        } else {
          midGoalFcn(100);
          delay(100);
            if (scoreRoller.velocity(rpm) > -10) {
              secondRollers.spin(reverse, 100, percent);
              scoreRoller.spin(forward, 100, percent);
              delay(200);

              secondRollers.spin(forward, 100, percent);
              scoreRoller.spin(reverse, 100, percent);
              delay(100);
            }
        }

        firstLongGoal = true;
        break;

      case (midGoalSlow):
        midGoalFcn(80);

        firstLongGoal = true;
        break;

      case (truncatedIntake):
        intakeRollers.spin(forward, 100, percent);

        firstLongGoal = true;
        break;

      case (primeLowGoal):
        if (startPrimingLowGoal) {
          scoreRoller.stop(hold);
          intakeFcn();
          delay(startPrimingAfter);

          startPrimingLowGoal = false;
        }
        scoreRoller.stop(hold);
        intakeRollers.spin(forward, 100, percent);
        secondRollers.spin(reverse, 13, percent);

        firstLongGoal = true;
        break;

      case (purgeOne):
        coastAllRollers();
        hoodUp();
        delay(300);
        hoodDown();

        firstLongGoal = true;
        break;

      case (topStageJamInsurance):
        intakeRollers.spin(forward, 100, percent);
        scoreRoller.stop(hold);

        secondRollers.spin(reverse, 100, percent);
        delay(200);
        secondRollers.spin(forward, 100, percent);
        delay(200);

        firstLongGoal = true;
        break;

      case (antiSpill):
        intakeRollers.stop(brake);
        secondRollers.stop(brake);
        scoreRoller.spin(reverse, 50, percent);

        firstLongGoal = true;
        break;

    }

    vex::this_thread::sleep_for(10);
  }
  return 0;
}

// Mag monitoring task
int magMonitorFcn() {
  while (true) {
    //Is there a block otw?
    if (OtwDistSensor.objectDistance(inches) < magDistanceThreshold) {blockOtw = true;} else {blockOtw = false;}

    //What color am I reading?
    if (OtwColorSensor.hue() < redThresholdUpper && OtwColorSensor.hue() > redThresholdLower) {
      readingRed = true;
      readingBlue = false;
    } else if (OtwColorSensor.hue() < blueThresholdUpper && OtwColorSensor.hue() > blueThresholdLower) {
      readingBlue = true;
      readingRed = false;
    } else {
      readingRed = false;
      readingBlue = false;
    }

    //If im reading X color, and I know a block is on a way, then the block on the way must be X colored.
    if (blockOtw && readingRed) {
      blockOtwIsRed = true;
      blockOtwIsBlue = false;
    } else if (blockOtw && readingBlue) {
      blockOtwIsBlue = true;
      blockOtwIsRed = false;
    } else {
      blockOtwIsRed = false;
      blockOtwIsBlue = false;
    }

    //Is there a block in the chamber?
    if (ChamberDistSensor.objectDistance(inches) < chamberDistanceThreshold) {blockInChamber = true;} else {blockInChamber = false;}

    vex::this_thread::sleep_for(10);
  }
  return 0;
}


double raiseIntakeIn = 0;

int delayedIntakeLiftFcn() {
  while (true) {
    if (raiseIntakeIn != 0) {
      delay(raiseIntakeIn);
      midDescoreUp();
      raiseIntakeIn = 0;
    }
  vex::this_thread::sleep_for(10);
  }
}













int trackDisplayTask() {
    while (true) {

        // ---------- Wheel & Gear Constants ----------
        const double wheelDiameter = 3.25;                         // inches
        const double wheelCircumference = wheelDiameter * M_PI;    // π * d

        // Gear ratio: motor → wheel
        // 36T driving 48T = 36/48 = 0.75  (wheel turns 0.75 for every motor turn)
        const double gearRatio = 36.0 / 48.0;

        // ---------- Read encoder ----------
        double motorDegrees = mr.position(vex::deg); // motor deg

        // Convert motor deg → wheel deg
        double wheelDegrees = motorDegrees * gearRatio;

        // Convert wheel deg → inches
        double inchesTracked = (wheelDegrees / 360.0) * wheelCircumference;

        // ---------- Display ----------
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(2, 10);
        Controller.Screen.print("MR Inches: %.3f", inchesTracked);

        vex::this_thread::sleep_for(50);
    }
    return 0;
}

int controllerScreenTask(void) {
  double TX_initial = TX.position(rotationUnits::deg); // store starting position

  while (true) {
    float leftMotorTempAvg = (
      fl.temperature(celsius) +
      ml.temperature(celsius) +
      bl.temperature(celsius)
    ) / 3.0;

    float rightMotorTempAvg = (
      fr.temperature(celsius) +
      mr.temperature(celsius) +
      br.temperature(celsius)
    ) / 3.0;

    float chassisTempAvg = (leftMotorTempAvg + rightMotorTempAvg) / 2.0;

    Controller.Screen.clearScreen();
    
    Controller.Screen.setCursor(1, 1);
    Controller.Screen.print("X:%.3f", chassis.x);
    
    Controller.Screen.setCursor(2, 1);
    Controller.Screen.print("Y:%.3f", chassis.y);
    
    Controller.Screen.setCursor(3, 1); 
    Controller.Screen.print("H:%.3f", chassis.h);

    Controller.Screen.setCursor(1, 10);
    Controller.Screen.print("Chassis:%2.f", chassisTempAvg);

    Controller.Screen.setCursor(2, 10);
    Controller.Screen.print("Sort: %s", colorSortMode == teamRedColorSort ? "Red" : colorSortMode == teamBlueColorSort ? "Blue" : "None");

    Controller.Screen.setCursor(3, 13);
    Controller.Screen.print(currentAuton.c_str());

    vex::this_thread::sleep_for(100);
  }
  return 0;
}


//BRAIN SCREEN STUFF
bool printingDistances = true;
int brainScreenTaskFcn() {
  Brain.Screen.setFont(mono20); // Or monoXL

  while (true) {
    // HUD overlay (distance sensors)
    if (printingDistances) {
    Brain.Screen.printAt(5,   20, "D1: %.5f   ", DistSensor1.objectDistance(inches));
    Brain.Screen.printAt(350, 20, "D2: %.5f   ", DistSensor2.objectDistance(inches));
    Brain.Screen.printAt(5,  235, "D3: %.5f   ", DistSensor3.objectDistance(inches));
    Brain.Screen.printAt(350,235, "D4: %.5f   ", DistSensor4.objectDistance(inches));
    }

    Brain.Screen.printAt(5, 130, "Max IMU Delta: %.3f deg/cycle   ", chassis.maxDeltaRecorded);

    vex::this_thread::sleep_for(50);
  }
  return 0;
}

void vexcodeInit(void) {

  task controllerScreen(controllerScreenTask);
  task controllerButtonMonitoringTask(controllerButtonMonitoringFcn);

  task magControlTask(magControlFcn);
  task magMonitorTask(magMonitorFcn);
  task delayedMatchloaderDropTask(delayedMatchloaderDropFcn);
  task brainScreenTask(brainScreenTaskFcn);
  task delayedIntakeLiftTask(delayedIntakeLiftFcn);

  // task trackDisplayTaskCall(trackDisplayTask);

  OtwColorSensor.setLightPower(100);
  OtwColorSensor.setLight(ledState::on);
  OtwColorSensor.integrationTime(5);

  matchloaderUp();
  hook1Down();
  hoodDown();
}

competition Competition;

PID turnPID;
PID drivePID;
PID cataPID;
PID arcPID;
movement move;
robot chassis;

void pre_auton(void) {
  vexcodeInit();
  task motors(autonMotors);
  task data(collectData);
  task output(dataSend);

  move.autondriveenabled = false;
}

int autonIndex = 3;

void autonomous(void) {
  move.autondriveenabled = true;

  // hkMovement();
  // sixRushSplit();
  // arpitWithInsurance();
  SmallTest();
  // twoMidGoals();
  // sixRush();
  // arpit();
  // tenBRight();
  // rightSplit();
  delay(5000000);
  // SOLO();
  // FastRightSide();

  if (autonIndex == 0) {
    SOLO();

  } else if (autonIndex == 1) {
    arpit();

  } else if (autonIndex == 2) {
    sixRush();

  } else if (autonIndex == 3) {
    sixRushSplit();
  } else if (autonIndex == 4) {
    rightSplit();
  } else if (autonIndex == 5) {
    rightSplit();
  } else if (autonIndex == 6) {
    rightSplit();
  } else if (autonIndex == 7) {
    rightSplit();
  } else if (autonIndex == 8) {
    rightSplit();
  } else if (autonIndex == 9) {
    rightSplit();
  }
    
  // delay(9999999999999); Used to be delay but I think was causing issues

  while (Competition.isAutonomous()) { //This instead to keep the thread running during auton
    wait(20, msec);
  }
}

void arcadeDriveFcn() {
  double turnImportance = 0.2; //0.2 on omnis, 0.13 on grips 
  
  fr.setStopping(coast);
  fl.setStopping(coast);
  mr.setStopping(coast);
  ml.setStopping(coast);
  br.setStopping(coast);
  bl.setStopping(coast);

  double turnVal = Controller.Axis1.position(percent);
  double forwardVal = Controller.Axis3.position(percent);

  //Expo curves, 0: y=x, 1: y=x^3. To see the curve, go to https://www.desmos.com/calculator/slehx2gwxs
  //Turn scaling, tune kTurn between 0 and 1
  double kTurn = 0;
  double turnNorm = turnVal / 100.0;
  turnVal = ((1 - kTurn) * turnNorm + kTurn * turnNorm * turnNorm * turnNorm) * 100.0;

  // Forward scaling
  double kForward = 0;
  double forwardNorm = forwardVal / 100.0;
  forwardVal = ((1 - kForward) * forwardNorm + kForward * forwardNorm * forwardNorm * forwardNorm) * 100.0;

  // Slew rate limiter — prevents tippy direction reversals on Axis3
  static double slewedForward = 0.0;
  double maxSlew = 6.0; // max % change per 10ms loop (~330ms to reverse fully)
  double slewDelta = forwardVal - slewedForward;
  if (slewDelta >  maxSlew) slewDelta =  maxSlew;
  if (slewDelta < -maxSlew) slewDelta = -maxSlew;
  slewedForward += slewDelta;
  forwardVal = slewedForward;

  // Convert to volts
  double turnVolts = turnVal * 0.127;
  double forwardVolts = forwardVal * 0.127 * (1 - (std::abs(turnVolts) / 127.0) * turnImportance);

  double leftVolts  = forwardVolts + turnVolts;
  double rightVolts = forwardVolts - turnVolts;

  fl.spin(forward, leftVolts, voltageUnits::volt);
  ml.spin(forward, leftVolts, voltageUnits::volt);
  bl.spin(forward, leftVolts, voltageUnits::volt);
  fr.spin(forward, rightVolts, voltageUnits::volt);
  mr.spin(forward, rightVolts, voltageUnits::volt);
  br.spin(forward, rightVolts, voltageUnits::volt);
}

// Curvature drive: You can speed up and slow down the chassis without changing what angle you are moving at
void curvatureDriveFcn() {
  fr.setStopping(coast);
  fl.setStopping(coast);
  mr.setStopping(coast);
  ml.setStopping(coast);
  br.setStopping(coast);
  bl.setStopping(coast);

  double throttle = Controller.Axis3.position(percent);
  double turn = Controller.Axis1.position(percent);

  // Deadband
  if (std::abs(throttle) < 5) throttle = 0;
  if (std::abs(turn) < 5) turn = 0;

  // Expo curves
  double kThrottle = 0;
  double tNorm = throttle / 100.0;
  throttle = ((1 - kThrottle) * tNorm + kThrottle * tNorm * tNorm * tNorm) * 100.0;

  // Inverted expo for turn: scales with throttle
  // At low throttle → linear turn. At full throttle → full inverse expo (small stick = big turn).
  double kTurnMax = 0.5; // max inverse expo strength at full throttle
  double kTurn = kTurnMax * (std::abs(throttle) / 100.0);
  double turnNorm = turn / 100.0;
  double cbrtTurn = copysign(cbrt(std::abs(turnNorm)), turnNorm);
  turn = ((1 - kTurn) * turnNorm + kTurn * cbrtTurn) * 100.0;

  double leftPct, rightPct;

  if (std::abs(throttle) < 5) {
    // In-place turning when not moving
    leftPct = turn;
    rightPct = -turn;
  } else {
    // Curvature: turn input controls radius of curvature, not turn rate
    // Flip curvature when going backward so turning direction matches arcade
    double curvature = turn / 100.0;
    if (throttle < 0) curvature = -curvature;
    leftPct = throttle * (1.0 + curvature);
    rightPct = throttle * (1.0 - curvature);
  }

  // Normalize so neither side exceeds 100%
  double maxVal = std::max(std::abs(leftPct), std::abs(rightPct));
  if (maxVal > 100.0) {
    leftPct = leftPct / maxVal * 100.0;
    rightPct = rightPct / maxVal * 100.0;
  }

  // Convert to volts
  double leftVolts = leftPct * 0.127;
  double rightVolts = rightPct * 0.127;

  fl.spin(forward, leftVolts, voltageUnits::volt);
  ml.spin(forward, leftVolts, voltageUnits::volt);
  bl.spin(forward, leftVolts, voltageUnits::volt);
  fr.spin(forward, rightVolts, voltageUnits::volt);
  mr.spin(forward, rightVolts, voltageUnits::volt);
  br.spin(forward, rightVolts, voltageUnits::volt);
}











bool driverActive = false;
bool driverAutonSkills = true;

void usercontrol(void) {

  bool lastButtonLeftState = false;
  static bool lastDown = false;          // rising edge detector  

  // Matchloader toggle variables
  static bool matchloaderState = true;  // toggle state for B
  static bool lastB = false;             // rising edge detector for B
  static uint32_t last_B_time = 0;       // debounce timestamp

  bool hook1Unlocked = false;
  bool hook2Unlocked = false;
  bool justUsedL1Macro = false;

  // R2 tap → midDescore toggle
  static bool lastR2First = false;
  static bool r2WasCombo = false;

  colorSortMode = none;

  hook1Unlocked = false;

  // move.autondriveenabled = true; // enable autonomous chassis control
  move.autondriveenabled = false; // disable autonomous chassis control
  driverActive = true;

  while (driverActive) {
    if (arcadeDrive) {
      arcadeDriveFcn();
    }

    if (curvatureDrive) {
      curvatureDriveFcn();
    }
    
    if (l2Pressed_first) {
      intakeMode = intake;
    }

    if (l1Pressed_first) {
      intakeMode = spit;
    } 

    if (r1Pressed_first) {
      intakeMode = midGoal;
    }

    if (r2Pressed_first) {
      intakeMode = longGoal;
    }

    if (not l2Pressed_first && not r2Pressed_first && not r1Pressed_first) {
      intakeMode = coasting;
    }

    /*if (l1Pressed_first) {
      hook1Down();
      hook1Unlocked = true;
    } else {
      if (hook1Unlocked) {
        hook1Up();
      }
    }*/

    if (Controller.ButtonB.pressing()) {
      hook1.set(!hook1.value());
      wait(150, msec); // debounce
    } 

    if (Controller.Axis2.position(percent) > 90) {
      move.autondriveenabled = true;
      HookProcedure();
      move.autondriveenabled = false;
    }

    if (Controller.ButtonX.pressing()) {
      hook1Down();
      move.autondriveenabled = true;
      leaveGoal();
      move.autondriveenabled = false;
    }

    //Color sort carousel control
    bool currentButtonLeftState = Controller.ButtonLeft.pressing();
    if (currentButtonLeftState && !lastButtonLeftState) { 
      if (colorSortMode == none) {
        colorSortMode = teamRedColorSort;
        Controller.rumble(".");
      } else if (colorSortMode == teamRedColorSort) {
        colorSortMode = teamBlueColorSort;
        Controller.rumble(".");
      } else if (colorSortMode == teamBlueColorSort) {
        colorSortMode = none;
        Controller.rumble("-");
      }
    }
    lastButtonLeftState = currentButtonLeftState;

    // Matchloader toggle (Button Down) - rising edge + 100ms debounce (ts was chatgpt)
    bool b = Controller.ButtonDown.pressing();
    uint32_t now_B = vex::timer::system();
    if (b && !lastB && (now_B - last_B_time > 100)) {
      matchloaderState = !matchloaderState;
      if (matchloaderState) {
        matchloaderUp();
      } else {
        matchloaderDown();
      }
      Controller.rumble("."); // feedback
      last_B_time = now_B;
    }
    lastB = b;

    if (Controller.ButtonLeft.pressing()) {
      if (colorSortMode == teamRedColorSort) {
        colorSortMode = teamBlueColorSort;
      }

      else if (colorSortMode == teamBlueColorSort) {
        colorSortMode = none;
      }

      else if (colorSortMode == none) {
        colorSortMode = teamRedColorSort;
      }

      delay(100);
    }

    if (Controller.ButtonRight.pressing()) {
      autonIndex += 1;
      Controller.rumble(".");
      delay(500);
    }

    if (autonIndex == 0) {
      currentAuton = "SOLO";

    } else if (autonIndex == 1) {
      currentAuton = "L6RUSH";

    } else if (autonIndex == 2) {
      currentAuton = "R6RUSH";

    } else if (autonIndex == 3) {
      currentAuton = "L6SPLIT";

    } else if (autonIndex == 4) {
      currentAuton = "R6SPLIT";

    } else if (autonIndex == 5) {
      currentAuton = "L9SPLIT";

    } else if (autonIndex == 6) {
      currentAuton = "R9SPLIT";

    } else if (autonIndex == 7) {
      currentAuton = "L7CNTR";

    } else if (autonIndex == 8) {
      currentAuton = "R7CNTR";

    } else if (autonIndex == 9) {
      currentAuton = "SKILLS";
    }



    // bounds clamp
    if (autonIndex > 4) {
      autonIndex = 0;
    }

  wait(10, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
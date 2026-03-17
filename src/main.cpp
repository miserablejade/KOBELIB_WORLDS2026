
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
motor intakeRollers = motor(PORT12, ratio6_1, true);
motor upperRollers = motor(PORT11, ratio6_1, false);

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
inertial IMU1 = inertial(PORT5);
inertial IMU2 = inertial(PORT5);

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
digital_out hood = digital_out(Brain.ThreeWirePort.E);
digital_out hook = digital_out(Brain.ThreeWirePort.A);
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

bool l1NewPress = false;
bool l2NewPress = false;
bool r1NewPress = false;
bool r2NewPress = false;
bool upNewPress = false;
bool downNewPress = false;
bool leftNewPress = false;
bool rightNewPress = false;
bool aNewPress = false;
bool bNewPress = false;
bool xNewPress = false;
bool yNewPress = false;

bool l1JustReleased = false;
bool l2JustReleased = false;
bool r1JustReleased = false;
bool r2JustReleased = false;
bool upJustReleased = false;
bool downJustReleased = false;
bool leftJustReleased = false;
bool rightJustReleased = false;
bool aJustReleased = false;
bool bJustReleased = false;
bool xJustReleased = false;
bool yJustReleased = false;

// How long each button has been continuously held (ms), 0 when not pressed
uint32_t l1HeldFor = 0, l2HeldFor = 0, r1HeldFor = 0, r2HeldFor = 0;
uint32_t upHeldFor = 0, downHeldFor = 0, leftHeldFor = 0, rightHeldFor = 0;
uint32_t aHeldFor = 0, bHeldFor = 0, xHeldFor = 0, yHeldFor = 0;

// Snapshot of how long the button was held at the moment it was released (ms).
// Valid for one cycle alongside xxxJustReleased, then resets to 0.
uint32_t l1PressTime = 0, l2PressTime = 0, r1PressTime = 0, r2PressTime = 0;
uint32_t upPressTime = 0, downPressTime = 0, leftPressTime = 0, rightPressTime = 0;
uint32_t aPressTime = 0, bPressTime = 0, xPressTime = 0, yPressTime = 0;

bool l1Exhausted = false, l2Exhausted = false, r1Exhausted = false, r2Exhausted = false;

void controllerButtonMonitoringFcn() {
  static bool prevL1 = false, prevL2 = false, prevR1 = false, prevR2 = false;
  static bool prevUp = false, prevDown = false, prevLeft = false, prevRight = false;
  static bool prevA = false, prevB = false, prevX = false, prevY = false;
  static uint32_t lastTickTime = vex::timer::system();

  bool curL1 = Controller.ButtonL1.pressing();
  bool curL2 = Controller.ButtonL2.pressing();
  bool curR1 = Controller.ButtonR1.pressing();
  bool curR2 = Controller.ButtonR2.pressing();
  bool curUp = Controller.ButtonUp.pressing();
  bool curDown = Controller.ButtonDown.pressing();
  bool curLeft = Controller.ButtonLeft.pressing();
  bool curRight = Controller.ButtonRight.pressing();
  bool curA = Controller.ButtonA.pressing();
  bool curB = Controller.ButtonB.pressing();
  bool curX = Controller.ButtonX.pressing();
  bool curY = Controller.ButtonY.pressing();
  uint32_t now = vex::timer::system();

  // New press detection (true for one cycle only)
  l1NewPress = (curL1 && !prevL1);
  l2NewPress = (curL2 && !prevL2);
  r1NewPress = (curR1 && !prevR1);
  r2NewPress = (curR2 && !prevR2);
  upNewPress = (curUp && !prevUp);
  downNewPress = (curDown && !prevDown);
  leftNewPress = (curLeft && !prevLeft);
  rightNewPress = (curRight && !prevRight);
  aNewPress = (curA && !prevA);
  bNewPress = (curB && !prevB);
  xNewPress = (curX && !prevX);
  yNewPress = (curY && !prevY);

  // Just released detection (true for one cycle only)
  l1JustReleased = (!curL1 && prevL1);
  l2JustReleased = (!curL2 && prevL2);
  r1JustReleased = (!curR1 && prevR1);
  r2JustReleased = (!curR2 && prevR2);
  upJustReleased = (!curUp && prevUp);
  downJustReleased = (!curDown && prevDown);
  leftJustReleased = (!curLeft && prevLeft);
  rightJustReleased = (!curRight && prevRight);
  aJustReleased = (!curA && prevA);
  bJustReleased = (!curB && prevB);
  xJustReleased = (!curX && prevX);
  yJustReleased = (!curY && prevY);

  // Snapshot hold duration on release (valid for one cycle alongside xxxJustReleased)
  l1PressTime = l1JustReleased ? l1HeldFor : 0;
  l2PressTime = l2JustReleased ? l2HeldFor : 0;
  r1PressTime = r1JustReleased ? r1HeldFor : 0;
  r2PressTime = r2JustReleased ? r2HeldFor : 0;
  upPressTime = upJustReleased ? upHeldFor : 0;
  downPressTime = downJustReleased ? downHeldFor : 0;
  leftPressTime = leftJustReleased ? leftHeldFor : 0;
  rightPressTime = rightJustReleased ? rightHeldFor : 0;
  aPressTime = aJustReleased ? aHeldFor : 0;
  bPressTime = bJustReleased ? bHeldFor : 0;
  xPressTime = xJustReleased ? xHeldFor : 0;
  yPressTime = yJustReleased ? yHeldFor : 0;

  // Reset exhausted flags on release
  if (l1JustReleased) l1Exhausted = false;
  if (l2JustReleased) l2Exhausted = false;
  if (r1JustReleased) r1Exhausted = false;
  if (r2JustReleased) r2Exhausted = false;

  // Held-for tracking — accumulates while pressed, resets to 0 on release
  l1HeldFor = curL1 ? (l1HeldFor + (now - lastTickTime)) : 0;
  l2HeldFor = curL2 ? (l2HeldFor + (now - lastTickTime)) : 0;
  r1HeldFor = curR1 ? (r1HeldFor + (now - lastTickTime)) : 0;
  r2HeldFor = curR2 ? (r2HeldFor + (now - lastTickTime)) : 0;
  upHeldFor = curUp ? (upHeldFor + (now - lastTickTime)) : 0;
  downHeldFor = curDown ? (downHeldFor + (now - lastTickTime)) : 0;
  leftHeldFor = curLeft ? (leftHeldFor + (now - lastTickTime)) : 0;
  rightHeldFor = curRight ? (rightHeldFor + (now - lastTickTime)) : 0;
  aHeldFor = curA ? (aHeldFor + (now - lastTickTime)) : 0;
  bHeldFor = curB ? (bHeldFor + (now - lastTickTime)) : 0;
  xHeldFor = curX ? (xHeldFor + (now - lastTickTime)) : 0;
  yHeldFor = curY ? (yHeldFor + (now - lastTickTime)) : 0;

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
  prevUp = curUp; prevDown = curDown; prevLeft = curLeft; prevRight = curRight;
  prevA = curA; prevB = curB; prevX = curX; prevY = curY;
  lastTickTime = now;
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

void hookUp() {
  hook.set(1);
}

void hookDown() {
  hook.set(0);
}

void hoodUp() {
  hood.set(1);
}

void hoodDown() {
  hood.set(0);
}

bool matchloaderIsDown = false;
void matchloaderUp() {
  matchloader.set(0);
  matchloaderIsDown = false;
}

void matchloaderDown() {
  matchloader.set(1);
  matchloaderIsDown = true;
}

void liftOdom() {
  odomLift.set(0);
}

void dropOdom() {
  odomLift.set(1);
}

bool fourBarUp = false;

void raise4bar() {
  intakeLift.set(0);
  fourBarUp = true;
}

void drop4bar() {
  intakeLift.set(1);
  fourBarUp = false;
}







void coastAllRollers() {
  intakeRollers.stop(coast);
  upperRollers.stop(coast);
}

void intakeFcn() {
  intakeRollers.spin(forward, 100, percent);
  upperRollers.spin(forward, 100, percent);
}

void scoreFcn() {
  intakeRollers.spin(forward, 100, percent);
  upperRollers.spin(forward, 100, percent);
}

void spitFcn() {
  intakeRollers.spin(reverse, 100, percent);
  upperRollers.spin(reverse, 100, percent);
}

double lowGoalSpeed = 20;
void lowGoalFcn(bool suspendUpper = false) {
    intakeRollers.spin(reverse, lowGoalSpeed, percent);
    if (suspendUpper) {
      upperRollers.stop(coast);
    } else {
      upperRollers.spin(reverse, 100, percent);
    }
}

//Intake monitor variables
bool blockOtw = false;

bool readingRed = false;
bool readingBlue = false;

bool blockOtwIsRed = false;
bool blockOtwIsBlue = false;

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

bool suspendUpperRollers = false;
bool slowLongGoal = false;

bool justDropped4Bar = true;

bool wasJustIntaking = false;

double score3Time = 500;

bool firstLongGoal = false;
bool dump3 = false;

// Intake Control Task
int magControlFcn() {
  while (true) {
    switch (intakeMode) {
      case (intake):
          intakeFcn();
          wasJustIntaking = true;

        break;

      case (spit):
        spitFcn();

        break;

      case (lowGoal):
        if (suspendUpperRollers) {
          lowGoalFcn(true);
        } else {
          lowGoalFcn(false);
        }

        break;

      case (coasting):
        coastAllRollers();
        if (wasJustIntaking) {
          upperRollers.spin(forward, 100, percent); //make sure the blocks are pressing against the hood, makes scoring more consistent
          delay(500);
        }
        wasJustIntaking = false;

        break;

      case (longGoal):
        scoreFcn();
        
        if (firstLongGoal) { //first long goal bool is controlled in usercontrol
          delay(score3Time); //If this is the first time r2 is being pressed, give it delay so it automatically dumps 3
          firstLongGoal = false;
        }

        if (dump3) { //If preston already let go of r2, that means he only tapped r2 and its going to be a quick dump
          upperRollers.spin(reverse, 100, percent);
          delay(100);
          dump3 = false;
        }

        break;

      case (midGoal):
        scoreFcn();
        break;

      case (midGoalSlow):

        break;

      case (truncatedIntake):
        intakeRollers.spin(forward, 100, percent);
        upperRollers.stop(coast);

        break;

      case (topStageJamInsurance):
        intakeRollers.spin(forward, 100, percent);

        upperRollers.spin(reverse, 100, percent);
        delay(200);
        upperRollers.spin(forward, 100, percent);
        delay(200);

        break;
    }

    if (justDropped4Bar) {
      upperRollers.spin(reverse, 100, percent);
      delay(200);
      justDropped4Bar = false;
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

  task magControlTask(magControlFcn);
  task magMonitorTask(magMonitorFcn);
  task delayedMatchloaderDropTask(delayedMatchloaderDropFcn);
  task brainScreenTask(brainScreenTaskFcn);
  task delayedIntakeLiftTask(delayedIntakeLiftFcn);

  // task trackDisplayTaskCall(trackDisplayTask);

  OtwColorSensor.setLightPower(100);
  OtwColorSensor.setLight(ledState::on);
  OtwColorSensor.integrationTime(5);

  dropOdom();
  matchloaderUp();
  hookDown();

  colorSortMode = none;
  intakeMode = coasting;
  fourBarUp = true;
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

bool didAuton = false;
int autonIndex = 3;

void autonomous(void) {
  move.autondriveenabled = true;

  didAuton = true;

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









void usercontrol(void) {
  
  bool hookUnlocked;

  if (didAuton) {
    hookUnlocked = true;
  } else {
    hookUnlocked = false;
  }

  while (true) {
    //BUTTON MONITOR////////////////////////////////////
    controllerButtonMonitoringFcn();
    ////////////////////////////////////////////////////


    //DRIVE CHOICE//////////////////////////////////////
    if (arcadeDrive) {
      arcadeDriveFcn();
    }

    if (curvatureDrive) {
      curvatureDriveFcn();
    }
    ////////////////////////////////////////////////////

    //INTAKE////////////////////////////////////////////
    if (l2Pressed_first) {
      hoodDown();
      raise4bar();
      midDescoreDown();
      intakeMode = intake;
    }

    /////////////////////////////////////////////////////

    //SPIT///////////////////////////////////////////////
    if (l1Pressed_first) {
      midDescoreUp();
      intakeMode = spit;
    } 

    /////////////////////////////////////////////////////

    //MID GOAL//////////////////////////////////////////
    if (r1Pressed_first) {
      if (fourBarUp) {
        drop4bar();
        justDropped4Bar = true;
        r1Exhausted = true;
      } else if (not r1Exhausted) {
        intakeMode = midGoal;
      }
    }

    if (r1JustReleased) {
      r1Exhausted = false;
    }

    /////////////////////////////////////////////////////

    //LONG GOAL//////////////////////////////////////////
    if (r2Pressed_first) {
      if (not fourBarUp) {
        raise4bar(); // just raise, don't score
        r2Exhausted = true;
      } else if (not r2Exhausted) {
        hoodUp();
        matchloaderUp();
        intakeMode = longGoal;
      }
    }

    if (r2JustReleased) {
      r2Exhausted = false;
    }

    if (r2JustReleased && r2PressTime < score3Time) {
      dump3 = true;
    }

    if (r2NewPress) {
      firstLongGoal = true;
    }

    /////////////////////////////////////////////////////

    //COASTING///////////////////////////////////////////
    if (not l2Pressed_first && not r2Pressed_first && not r1Pressed_first) {
      intakeMode = coasting;
      firstLongGoal = false;
    }

    /////////////////////////////////////////////////////

    //HOOK///////////////////////////////////////////////
    if (Controller.ButtonDown.pressing()) {
      hookDown();
      hookUnlocked = true; 
    } else {
      if (hookUnlocked) {
        hookUp();
      }
    }

    /////////////////////////////////////////////////////

    //HOOK MACRO!////////////////////////////////////////
    if (Controller.Axis2.position(percent) > 90) {
      move.autondriveenabled = true;
      HookProcedure();
      move.autondriveenabled = false;
    }

    /////////////////////////////////////////////////////

    //MID DESCORE////////////////////////////////////////
    if (Controller.ButtonB.pressing()) {
      midDescoreUp();
    } else {
      midDescoreDown();
    }

    /////////////////////////////////////////////////////

    //COLOR SORT CONTROL//////////////////////////////////
    if (leftNewPress) {
      if (colorSortMode == none) {
        colorSortMode = teamRedColorSort;
      } else if (colorSortMode == teamRedColorSort) {
        colorSortMode = teamBlueColorSort;
      } else if (colorSortMode == teamBlueColorSort) {
        colorSortMode = none;
      }

      if (didAuton) {
        colorSortMode = none;
      }

      Controller.rumble(".");
    }

    /////////////////////////////////////////////////////

    //MATCHLOADER CONTROL////////////////////////////////
    if (yNewPress) {
      if (matchloaderIsDown) {
        matchloaderUp();
      } else {
        matchloaderDown();
      }
    }

    /////////////////////////////////////////////////////

    if (rightNewPress) {
      autonIndex += 1;
      Controller.rumble(".");
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
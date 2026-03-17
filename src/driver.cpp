// #include "vex.h"
// #include "movement.h"
// bool boosted_shots = false;

// bool last_L2 = false;
// void chassisOpControl() {
//     float axis3 = Controller.Axis3.position() * 1.15;
//     float axis2 = Controller.Axis2.position() * 1.15;
//     float left = (axis3 * fabs(axis3)) / 100;
//     float right = (axis2 * fabs(axis2)) / 100;
//     bool B = Controller.ButtonB.pressing();
//     bool L2 = Controller.ButtonL2.pressing();
//   if (B) {
//     voltagedrive(3, -3);
//   }
//    else {
//     voltagedrive(left*.12, right*.12);
//   }

//     if (L2 && last_L2 != L2) {
//     intake_lift = !intake_lift;
//     }
//       last_L2 = Controller.ButtonL2.pressing();
// }


// bool last_X = false;
// bool last_Y = false;
// bool last_A = false;
// bool last_B = false;
// bool last_down = false;

// bool disableCata = false;
// bool lastRight = false;
// int counter = 0;


// bool last_R1 = false;
// void intake_op_control() {
// bool R1 = Controller.ButtonR1.pressing();
// bool R2 = Controller.ButtonR2.pressing();
// float pow = 0;
// if (disableCata) pow = R1*12 - R2*12;
// else if (cataRotation.angle() > (move.cataTarg - 20) && cataRotation.angle() < (move.cataTarg + 20)) pow = R1*12 - R2*12;
// else pow = 0;
// intake.spin(fwd, pow, volt);
// }


// void robotOpControl() {
//   chassisOpControl();
//   intake_op_control();

//     if (Controller.ButtonA.pressing() && last_A != Controller.ButtonA.pressing()) {
//    right_expand = true;
//   }
//     if (Controller.ButtonB.pressing() && last_B != Controller.ButtonB.pressing()) {
//    left_expand = true;
//   }
//     if (Controller.ButtonX.pressing() && last_X != Controller.ButtonX.pressing()) {
//    side_expand = true;
//   }
//     if (Controller.ButtonDown.pressing() && last_down != Controller.ButtonDown.pressing()) {
//    band_release = !band_release;
//   }

//   if (Controller.ButtonY.pressing() && last_Y != Controller.ButtonY.pressing()) {
//     if (boosted_shots == false) Controller.rumble("--");
//     if (boosted_shots == true) Controller.rumble("..");

//     boosted_shots = !boosted_shots;
//   }

//   last_X = Controller.ButtonX.pressing();
//   last_Y = Controller.ButtonY.pressing();
//   last_A = Controller.ButtonA.pressing();
//   last_B = Controller.ButtonB.pressing();
//   last_down = Controller.ButtonDown.pressing();
//     if (Controller.ButtonDown.pressing()) {
//     left_expand = false;      
//     right_expand = false;    
//     side_expand = false;      
  
//     }
//     if (Controller.ButtonUp.pressing()) {
//     left_expand = true;      
//     right_expand = true;  
//     side_expand = true;  

//     }
    
// if (Controller.ButtonRight.pressing() && lastRight == false) {
//  disableCata = !disableCata;
//   }
// lastRight = Controller.ButtonRight.pressing();

//   if (disableCata == true ) {
//    cata.stop(coast);
//     move.cataEnabled = false;
//   } else {

//    if (Controller.ButtonL1.pressing()) {
//     if (boosted_shots) {
//     //boostShot(80);
//     boostShot(45);
//     //weak_boost(140);
//     } else {
//       move.shoot = true;
//     }

//       }  else {
//     move.cataEnabled = true;
//   }
// }


// }
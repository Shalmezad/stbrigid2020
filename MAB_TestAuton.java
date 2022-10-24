package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous

public class MAB_TestAuton extends MAB_BaseAuton {
 public void runOpMode(){
   super.runOpMode();
   double FD = 0.0;
   double BD = 0.0;
   double RD = 0.0;
   double LD = 0.0;
   double lHue = 0.0;
   //DriveForwardToXInches(8.0);
   DriveWithinXFeet(DirState.Front, 0.25);
   WaitNSeconds(2.0);
   DriveWithinXFeet(DirState.Back, 0.25);
   WaitNSeconds(2.0);
   DriveWithinXFeet(DirState.Right, 0.25);
   WaitNSeconds(2.0);
   DriveWithinXFeet(DirState.Left, 0.25);
   WaitNSeconds(2.0);
   /*
   FD = getDistInches(DirState.Front);
   telemetry.addData("Front distance:", String.format("%f", FD));
   telemetry.addData("Front ordinal:", String.format("%d", DirState.Front.ordinal()));
   BD = getDistInches(DirState.Back);
   telemetry.addData("Back distance:", String.format("%f", BD));
   telemetry.addData("Back ordinal:", String.format("%d", DirState.Back.ordinal()));
   RD = getDistInches(DirState.Right);
   telemetry.addData("Right distance:", String.format("%f", RD));
   telemetry.addData("Right ordinal:", String.format("%d", DirState.Right.ordinal()));
   LD = getDistInches(DirState.Left);
   telemetry.addData("Left distance:", String.format("%f", LD));
   telemetry.addData("Left ordinal:", String.format("%d", DirState.Left.ordinal()));
   //telemetry.addData("Back distance:", String.format("%f", BD));
   telemetry.update();
   WaitNSeconds(5.0);
   */
   /*
   DriveBackToXInches(4.0);
   FD = getFrontDistanceInches();
   BD = getBackDistanceInches();
   telemetry.addData("Front distance:", String.format("%f", FD));
   telemetry.addData("Back distance:", String.format("%f", BD));
   telemetry.update();
   */
   WaitNSeconds(5.0);
   // lHue = getLeftHue();
   // telemetry.addData("Left Hue:", String.format("%f", lHue));
   /*
   //PositionLeftSkystoneGrabber(1); 
   //PositionLeftSkystoneGrabber(0); 
   //PositionRightSkystoneGrabber(1); 
   //PositionRightSkystoneGrabber(0); 
   
   //FoundationPositionHook(1);
   //FoundationPositionHook(0);
   DriveXFeet(XDist, xPow);
   turnNDegreesAbsolute(0);
   //WaitNSeconds(0.2);
   DriveXFeet(XDist, xPow);
   turnNDegreesAbsolute(0);
   //WaitNSeconds(0.2);
   DriveXFeet(XDist, xPow);
   turnNDegreesAbsolute(0);
   //WaitNSeconds(0.2);
   DriveXFeet(XDist, xPow);
   turnNDegreesAbsolute(0);
   //WaitNSeconds(2);
   //StrafeXFeet(-1.4,.2);
   //turnNDegreesAbsolute(180);
   //DriveXFeet(0.3); 
   WaitNSeconds(30);
   */
   /* turnNDegreesAbsolute(90);
   DriveXFeet(2);
   turnNDegreesAbsolute(90);
   StopDriveTrain();*/
   //StopDriveTrain();
   //DriveXFeet(5);
   //DriveXFeet(-5);
   //StrafeXFeet(5);
   //StrafeXFeet(-5); 
   //turnNDegrees(90); 
   // turnNDegrees(-90); 
   //WaitNSeconds(2);
   //waitForStart();
 }
}

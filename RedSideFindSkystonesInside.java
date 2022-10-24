package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous

public class RedSideFindSkystonesInside extends BaseAuton{
 public void runOpMode(){
 super.runOpMode(); 
 //DriveXFeet(-2/12.0,0.3);
 DriveLeftSideToXInches(2.2);
 turnNDegreesAbsolute(0);
 if(IsLeftSkystone()){
 //Position 3 & 6
 telemetry.addData("Skystones", "3&6");
 telemetry.update();
 //DriveXFeet(-2.0/12,0.4);
 StrafeXFeet(-3.0/12,0.4);
 PositionLeftSkystoneGrabber(1); 
 DriveXFeet(2.0/12,0.3);
 StrafeXFeet(10.0/12,0.4);
 turnNDegreesAbsolute(0); 
 DriveXFeet(2.5+8/12.0, 0.5); 
 PositionLeftSkystoneGrabber(0); 
 //First block done. 
 turnNDegreesAbsolute(0);
 DriveXFeet(-3.4-8/12.0, 0.5);
 turnNDegreesAbsolute(0);
 DriveBackToXInches(18.1); 
 turnNDegreesAbsolute(0);
 DriveLeftSideToXInches(2.0);
 StrafeXFeet(-2.5/12,0.4);
 PositionLeftSkystoneGrabber(1); //grab second block. 
 DriveXFeet(2.0/12,0.3);
 StrafeXFeet(10.0/12,0.4);
 turnNDegreesAbsolute(0); 
 DriveXFeet(4.83+8/12.0,0.4); 
 PositionLeftSkystoneGrabber(0); 
 //Second block done. 
 DriveXFeet(-0.75-8/12.0); 
 StrafeXFeet(-0.9,0.4); 
 }
 else{
 //Position 4 or 5
 DriveXFeet(-8/12.0, 0.2);
 if(IsLeftSkystone()){
 //Position 2 & 5
 telemetry.addData("Skystones", "2&5");
 telemetry.update();
 //DriveXFeet(-2.0/12,0.4);
 StrafeXFeet(-3.0/12,0.4);
 PositionLeftSkystoneGrabber(1); 
 DriveXFeet(2.0/12,0.3);
 StrafeXFeet(10.0/12,0.4);
 turnNDegreesAbsolute(0); 
 DriveXFeet(2.5+16/12.0, 0.5); 
 PositionLeftSkystoneGrabber(0); 
 //First block done. 
 turnNDegreesAbsolute(0);
 DriveXFeet(-3.4-8/12.0, 0.5);
 turnNDegreesAbsolute(0);
 DriveBackToXInches(10.1); 
 turnNDegreesAbsolute(0);
 DriveLeftSideToXInches(2.0);
 StrafeXFeet(-2.5/12,0.4);
 PositionLeftSkystoneGrabber(1); //grab second block. 
 DriveXFeet(2.0/12,0.3);
 StrafeXFeet(10.0/12,0.4);
 turnNDegreesAbsolute(0); 
 DriveXFeet(4.83+16/12.0,0.4); 
 PositionLeftSkystoneGrabber(0); 
 //Second block done. 
 DriveXFeet(-0.75-8/12.0); 
 StrafeXFeet(-0.9,0.4); 
 }
 
 else{
 // Position 1 & 4
 DriveXFeet(-8/12.0, 0.2);
 telemetry.addData("Skystones", "1&4");
 telemetry.update();
 //DriveXFeet(-2.0/12,0.4);
 StrafeXFeet(-3.0/12,0.4);
 PositionLeftSkystoneGrabber(1); 
 DriveXFeet(2.0/12,0.3);
 StrafeXFeet(10.0/12,0.4);
 turnNDegreesAbsolute(0); 
 DriveXFeet(2.5+24/12.0, 0.75); 
 PositionLeftSkystoneGrabber(0); 
 //First block done. 
 turnNDegreesAbsolute(0);
 DriveXFeet(-3.4-8/12.0, 0.75);
 turnNDegreesAbsolute(0);
 DriveBackToXInches(2.1); 
 turnNDegreesAbsolute(0);
 DriveLeftSideToXInches(2.0);
 StrafeXFeet(-2.5/12,0.4);
 PositionLeftSkystoneGrabber(1); //grab second block. 
 DriveXFeet(2.0/12,0.3);
 StrafeXFeet(10.0/12,0.4);
 turnNDegreesAbsolute(0); 
 DriveXFeet(4.83+24/12.0,0.75); 
 PositionLeftSkystoneGrabber(0); 
 //Second block done. 
 DriveXFeet(-0.75-8/12.0, 0.75); 
 StrafeXFeet(-0.9,0.4); 
 
 }
 }
 }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous

public class TestAuton extends BaseAuton {
     public void runOpMode(){
     super.runOpMode();
         PositionLeftSkystoneGrabber(1); 
         PositionLeftSkystoneGrabber(0); 
         PositionRightSkystoneGrabber(1); 
         PositionRightSkystoneGrabber(0); 

     StrafeXFeet(4.4,.4);
     turnNDegreesAbsolute(0);
     WaitNSeconds(20);
     StrafeXFeet(-2.3,.4);
     turnNDegreesAbsolute(0);
     DriveXFeet(1.0); 
     WaitNSeconds(20);

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
    
        
    
    
    
    
    
    
    
    
    
     
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled

public class RedEverythingInside extends BaseAuton {
    public void runOpMode(){
        super.runOpMode();
        DriveRightSideToXInches(2.0); 
        WaitNSeconds(0.25);
        if(IsLeftSkystone()){
             //Position 3 & 6
             StrafeXFeet(-1,0.4);
             turnNDegreesAbsolute(0); 
             DriveXFeet(-3.4-12/12.0, 0.55); 
             //First block done. 
             /*turnNDegreesAbsolute(0);
             DriveXFeet(4.3+12/12.0, 0.7);
             turnNDegreesAbsolute(0);
             DriveForwardToXInches(5.4); 
             turnNDegreesAbsolute(0);
             DriveRightSideToXInches(1.8); 
             PositionHook(1); 
             StrafeXFeet(-1,0.4); 
             DriveXFeet(-5.7-12/12.0,0.55); 
             PositionHook(0.3, false); 
             //Second block done. 
             turnNDegreesAbsolute(0);
             */
             DriveXFeet(-2.8+12/12.0); 
             StrafeXFeet(1.3); 
             FoundationPositionHook(0.55);
             turnNDegrees(-10);
             StrafeXFeet(-3.45,0.75); //speed was 0.55. 
             FoundationPositionHook(0.05);
             DriveXFeet(1.0);
             turnNDegreesAbsolute(0);
             DriveXFeet(2.5); 
        }
        else{
            //Position 4 or 5
            DriveXFeet(8.0/12.0, 0.2);
            WaitNSeconds(0.25);
            if(IsLeftSkystone()){
                 StrafeXFeet(-1,0.4);
                 turnNDegreesAbsolute(0); 
                 DriveXFeet(-3.4-8.0/12-12/12.0, 0.4);
                 //First block done.
                  turnNDegreesAbsolute(0);
                 DriveXFeet(-2.8+12/12.0);
                 turnNDegreesAbsolute(0); 
                 StrafeXFeet(1.3); 
                 FoundationPositionHook(0.55);
                 turnNDegrees(-10);
                 StrafeXFeet(-3.45,0.75); //speed was 0.75
                 FoundationPositionHook(0.05);
                 DriveXFeet(1.0);
                 turnNDegreesAbsolute(0);
                 DriveXFeet(2.5); 
            }
            else{
                // Position 1 & 4
                DriveXFeet(8.0/12.0, 0.2);
                WaitNSeconds(0.25);
                StrafeXFeet(-1,0.4);
                turnNDegreesAbsolute(0); 
                DriveXFeet(-3.4-16.0/12-12/12.0, 0.4);
                turnNDegreesAbsolute(0); 
                //First block done
                DriveXFeet(-3.1+12/12.0);
                turnNDegreesAbsolute(0); 
                StrafeXFeet(1.3);
                turnNDegreesAbsolute(0); 
                FoundationPositionHook(0.55);
                turnNDegrees(-10);
                StrafeXFeet(-3.45,0.55); 
                turnNDegreesAbsolute(0); 
                FoundationPositionHook(0.05);
                DriveXFeet(1.0);
                turnNDegreesAbsolute(0);
                DriveXFeet(2.5); 
            }
        }
       DriveForwardToXFeet(1.5); 
       turnNDegrees(90); 
       DriveXFeet(-0.5); 
       FoundationPositionHook(1); 
       DriveXFeet(2); 
       FoundationPositionHook(0); 
       StrafeXFeet(-3.5); 
    }
}


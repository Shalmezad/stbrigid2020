package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous
@Disabled
public class RedSideFindSkystonesOutside extends BaseAuton{
    public void runOpMode(){
    super.runOpMode(); 
    DriveRightSideToXInches(-1.8); 
    if(IsLeftSkystone()){
         //Position 3 & 6
         DriveXFeet(2.5/12); 
         PositionLeftSkystoneGrabber(1); 
         
         StrafeXFeet(-0.5,0.3);
         turnNDegreesAbsolute(0); 
         DriveXFeet(2.5+8/12.0, 0.4); 
         PositionRightSkystoneGrabber(0); 
         //First block done. 
         turnNDegreesAbsolute(0);
         DriveXFeet(-3.4-8/12.0, 0.4);
         turnNDegreesAbsolute(0);
         DriveBackToXInches(19.1); 
         turnNDegreesAbsolute(0);
         DriveLeftSideToXInches(2.0);
         PositionRightSkystoneGrabber(1); //grab second block. 
         StrafeXFeet(0.5,0.3);
         turnNDegreesAbsolute(0); 
         DriveXFeet(4.83+8/12.0,0.4); 
         PositionRightSkystoneGrabber(0); 
         //Second block done. 
         DriveXFeet(-0.75-8/12.0); 
         StrafeXFeet(-0.3,0.3); 
    }
    else{
        //Position 4 or 5
        DriveXFeet(-8/12.0, 0.2);
        if(IsLeftSkystone()){
            //Position 2 & 5
            DriveXFeet(2.5/12); 
              PositionLeftSkystoneGrabber(1); 
             StrafeXFeet(-0.5,0.3);
           
             turnNDegreesAbsolute(0); 
             DriveXFeet(2.5+12.0/12+8/12.0, 0.4); 
             PositionLeftSkystoneGrabber(0);
             //First block done. 
             turnNDegreesAbsolute(0);
             DriveXFeet(-3.4-12.0/12-8/12.0, 0.55);
             turnNDegreesAbsolute(0);
             DriveBackToXInches(19.1-8.0); 
             turnNDegreesAbsolute(0);
             DriveLeftSideToXInches(2.0); 
             PositionLeftSkystoneGrabber(1); 
             StrafeXFeet(0.5,0.3); 
             turnNDegreesAbsolute(0); 
             DriveXFeet(4.83+8.0/12+8/12.0,0.5); 
             PositionLeftSkystoneGrabber(0); 
             //Second block done. 
             DriveXFeet(-0.75-8/12.0);
             StrafeXFeet(-0.3,0.3); 
            }
    
        else{
           // Position 1 & 4
           DriveXFeet(-8/12.0, 0.2);
                //Position 2 & 5 
                DriveXFeet(2.5/12); 
                PositionLeftSkystoneGrabber(1); 
                 StrafeXFeet(0.5,0.3);
                 
                 turnNDegreesAbsolute(0); 
                 DriveXFeet(2.5+24.0/12+8/12.0, 0.4); 
                 PositionLeftSkystoneGrabber(0); 
                 //First block done. 
                 turnNDegreesAbsolute(0);
                 DriveXFeet(-3.4-24.0/12-8/12.0, 0.55);
                 turnNDegreesAbsolute(0);
                 DriveBackToXInches(19.1-16.0); 
                 turnNDegreesAbsolute(0);
                 DriveLeftSideToXInches(2.0); 
                 PositionLeftSkystoneGrabber(1); 
                 StrafeXFeet(-0.5,0.3); 
                 turnNDegreesAbsolute(0); 
                 DriveXFeet(4.83+16.0/12+8/12.0,0.5); 
                 PositionLeftSkystoneGrabber(0); 
                 //Second block done. 
                 DriveXFeet(-0.75-8/12.0);
                 StrafeXFeet(-0.3,0.3); 
                
            }
        }
    }
}


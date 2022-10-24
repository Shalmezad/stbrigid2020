package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

    @Autonomous

public class RedPlatformInside extends BaseAuton{
    public void runOpMode(){
        super.runOpMode();
        StrafeXFeet(-0.5);
        DriveXFeet(-2.7, 0.3); 
        FoundationPositionHook(1);
        DriveXFeet(1.75);
        turnNDegrees(90);
        
        FoundationPositionHook(0);
        turnNDegreesAbsolute(-90);
        DriveLeftSideToXInches(4);
        StrafeXFeet(2);
        turnNDegreesAbsolute(-90);
        DriveXFeet(3.35, 0.3); 
       }
}


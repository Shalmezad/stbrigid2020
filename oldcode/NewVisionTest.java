package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.vuforia.CameraDevice;
import org.firstinspires.ftc.teamcode.BaseAuton;

@Autonomous
@Disabled
public class NewVisionTest extends BaseAuton {
     public void runOpMode(){
        super.runOpMode();
        while(opModeIsActive()){
             telemetry.addData("Is skystone: %b", IsRightSkystone());
             telemetry.addData("Right Hue: ", getRightHue());
             telemetry.update();
        }
     }
}

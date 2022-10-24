package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class MAB_BaseAuton extends LinearOpMode { 
    double strafespeed = 0.6;
    double drivespeed = 0.6; 
    double turnspeed = 0.30;
    double DriveSpeedMax = 0.9;
    double DriveSpeedMin = 0.2;
    double MaxSpeedDist = 2.0;  //feet, distance to start slowing down from DriveSpeedMax
    double MinSpeedDist = 0.8;  //feet, distance to go to DriveSpeedMin
    double StrafeSpeedMax = 0.35;
    double rdistanceoffset = 0.5; //inch
    double ldistanceoffset = 0.5; //inch
    double bdistanceoffset = 0.5; //inch
    double fdistanceoffset = 0.5; //inch        
    float hsvValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;

    //declare enum for user to select direction of travel.
    enum DirState {
        Front, Back, Right, Left, TurnRight, TurnLeft
    }
    //declare 2 dimensional array to hold the sign of each motor 
    //corresponding to the direction DirState.
    final int MotorSign[][] = {
        //{Order:  frontleft, frontright, backleft, backright}
        { 1, 1, 1, 1},        //corresponds to DirState Front,    index 0
        {-1,-1,-1,-1},        //corresponds to DirState Back,     index 1
        { 1,-1,-1, 1},        //corresponds to DirState Right,    index 2
        {-1, 1, 1,-1},        //corresponds to DirState Left,     index 3
        { 1,-1, 1,-1},        //corresponds to DirState TurnRight,index 4
        {-1, 1,-1, 1}         //corresponds to DirState TurnLeft, index 5
    };

    DcMotor frontleftmotor = null; 
    DcMotor frontrightmotor = null; 
    DcMotor backleftmotor = null;
    DcMotor backrightmotor = null;
    Servo rightfoundationservo = null; 
    Servo leftfoundationservo = null; 
    DistanceSensor RightSideSensor = null; 
    DistanceSensor LeftSideSensor = null;
    DistanceSensor BackSensor = null; 
    DistanceSensor FrontSensor= null; 
    ColorSensor leftcolorSensor;
    ColorSensor rightcolorSensor;
    Servo LeftSkystoneGrabber = null; 
    Servo RightSkystoneGrabber = null; 

    BNO055IMU imu = null;

    // 1, 2, or 3
    private int detectedStone = 2;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "Ae1kn+L/////AAABmRUoKvbYmEJdsWBI5qFhYpMSvUMNOs81Xk6In/3fWY5dIQ6nlXvMP4ywza2a2tZTTAML/8j/yWyrXWkavYK10BJ3uLTDMdTUwM0sK8Ce10okHwUQYNvGRmUS0HV6cTsuK3fLjTi83SKW1tCPL4TBkhDkIT402iU6Bm7eb3/sfwBsnz2sEqsmtmX3NWc+1R1lGwzlRCiXdAcoLQgy0DsRUAGV1uoLBFkif5rX2MkMIJhmpeqX7QuOBwD0y1ZkRF+cnd/a1ljSLmpBxiF/YpbCs1+0qabMmwVJ4Nyzy6QFHKaTSrGNnamJdWEY2t82Xj+nRB5yPUzGr9Nfzsedeso6htc0cwMtJvf7RILvC9ud11Zk";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void DriveXFeet(double X, double drivespeed){
        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        frontleftmotor.setMode (runMode); 
        frontrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        frontrightmotor.setMode (runMode);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        backleftmotor.setMode (runMode); 
        backrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        backrightmotor.setMode (runMode);
        //TODO:get ticks per foot
        if(X<0){
            while (frontleftmotor.getCurrentPosition()>608*X && opModeIsActive()){
                frontleftmotor.setPower(-1*drivespeed); 
                frontrightmotor.setPower(-1*drivespeed); 
                backleftmotor.setPower(-1*drivespeed); 
                backrightmotor.setPower(-1*drivespeed); 
            }
        }
        else{
             while (frontleftmotor.getCurrentPosition()<608*X && opModeIsActive()){ 
                frontleftmotor.setPower(drivespeed); 
                frontrightmotor.setPower(drivespeed); 
                backleftmotor.setPower(drivespeed);
                backrightmotor.setPower(drivespeed); 
            }
        }
        StopDriveTrain(); 
    }

    public void StopDriveTrain(){
        PowerMotors(DirState.Front, 0);
    }

    public void FoundationPositionHook(double X){
        leftfoundationservo.setPosition(X); //One is down.
        rightfoundationservo.setPosition(-X+1); 
        WaitNSeconds(1.8); 
    } 

    public void PositionLeftSkystoneGrabber (double X){
        X = X * 0.62;
        X = 1 - X;
        LeftSkystoneGrabber.setPosition(X);
        WaitNSeconds(1.3); 
    }

    public void PositionRightSkystoneGrabber(double X){
        X = X * 0.65;
        RightSkystoneGrabber.setPosition(X);
        WaitNSeconds(1.3); 
    }

    public double getDistInches(DirState theDirState){
        double theDist;
        switch(theDirState){
            case Front:  theDist = FrontSensor.getDistance(DistanceUnit.INCH);
                break;
            case Back:   theDist = BackSensor.getDistance(DistanceUnit.INCH);
                break;
            case Right:  theDist = RightSideSensor.getDistance(DistanceUnit.INCH);
                break;
            case Left:   theDist = LeftSideSensor.getDistance(DistanceUnit.INCH);
                break;
            default:     theDist = FrontSensor.getDistance(DistanceUnit.INCH);
        }
        return theDist;
    }

    public void DriveRightSideToXInches(double X){
        DriveWithinXFeet(DirState.Right, X/12.0); 
    }

    public void DriveLeftSideToXInches(double X){
        DriveWithinXFeet(DirState.Left, X/12.0); 
    }

    public void DriveBackToXInches(double X){
        DriveWithinXFeet(DirState.Back, X/12.0);
    }

    public void DriveForwardToXInches(double X){
        DriveWithinXFeet(DirState.Front, (X + fdistanceoffset) / 12.0); 
    }
    
    public void PowerMotors(DirState theDirState, double theSpeed){
        int motorIndx;
        //get the motor index corresponding to the direction of travel(theDirState):
        motorIndx = theDirState.ordinal();
        frontleftmotor.setPower(theSpeed * MotorSign[motorIndx][0]);    
        frontrightmotor.setPower(theSpeed * MotorSign[motorIndx][1]);    
        backleftmotor.setPower(theSpeed * MotorSign[motorIndx][2]);    
        backrightmotor.setPower(theSpeed * MotorSign[motorIndx][3]);    
    }
    
    public void DriveWithinXFeet(DirState theDirState, double Xft){
        //all units in feet!
        boolean ReachedDest = false;
        double speed = 0;
        double m = 0.001;  //slope
        double i = 0.0;
        double Distft = 100.0;
        double HighestSpeedDist = Xft + MaxSpeedDist;
        double SlowestSpeedDist = Xft + MinSpeedDist;
        telemetry.setAutoClear(false);
        Telemetry.Item theLoop = telemetry.addData("loop:", 0);
        Telemetry.Item DistItem = telemetry.addData("Dist:", 0);
        Telemetry.Item countItem = telemetry.addData("count:", 0);
        Telemetry.Item speedItem = telemetry.addData("speed:", 0);
        Telemetry.Item calcspeedItem = telemetry.addData("calcspeed:", 0);
        Telemetry.Item mItem = telemetry.addData("m(slope):", 0);
        m = (DriveSpeedMax - DriveSpeedMin) / (MaxSpeedDist - MinSpeedDist);
        mItem.setValue(String.format("%f", m));
        telemetry.update();
        //ElapsedTime timeout = new ElapsedTime();
        //timeout.reset();
        SetMotorModes(true);
        while((!ReachedDest)){   // && opModeIsActive() && (timeout.seconds() < 12)
            //Have not timed out, reached destination and Op mode is active.
            i = i + 1.0;
            //Calculate distance to target:
            Distft = getDistInches(theDirState) / 12.0;   
            countItem.setValue(String.format("%f", i));
            DistItem.setValue(String.format("%f", Distft));
            //determine speed:
            if((Distft >= HighestSpeedDist)){
                //Robot still has a ways to travel.
                speed = DriveSpeedMax;
                calcspeedItem.setValue(String.format("%f", speed));
                theLoop.setValue("far away");
            }
            else{
                if(Distft >= SlowestSpeedDist){
                    theLoop.setValue("calc speed");
                    //Need to calculate speed based on distance to target:
                    //y = mX +b
                    speed = m * (Distft - MinSpeedDist);
                    calcspeedItem.setValue(String.format("%f", speed));
                }else{
                    //Robot is close.
                    if(Distft <= Xft){
                        theLoop.setValue("at target");
                        //Reached target:
                        speed = 0;
                        calcspeedItem.setValue(String.format("%f", speed));
                        ReachedDest = true;
                    }
                    else{
                        theLoop.setValue("min speed");
                        speed = DriveSpeedMin;
                        calcspeedItem.setValue(String.format("%f", speed));
                        speedItem.setValue(String.format("%f", speed));
                    }
                }
            }
            //just in case...
            if(speed > DriveSpeedMax){
                speed = DriveSpeedMax;
                speedItem.setValue(String.format("%f", speed));
            }
            speedItem.setValue(String.format("%f", speed));
            telemetry.update();
            //call procedure to drive te motors with direction and speed:
            PowerMotors(theDirState, speed);
        }
        StopDriveTrain();
    }

    private void SetMotorModes(boolean UseEncoder){
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        frontrightmotor.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        backrightmotor.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        if(UseEncoder){
            frontleftmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER); 
            frontrightmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
            backleftmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER); 
            backrightmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            frontleftmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
            frontrightmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backleftmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
            backrightmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public double getLeftHue(){
        Color.RGBToHSV((int) (leftcolorSensor.red() * SCALE_FACTOR),
                (int) (leftcolorSensor.green() * SCALE_FACTOR),
                (int) (leftcolorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues[0];
    }
    public double getRightHue(){
        Color.RGBToHSV((int) (rightcolorSensor.red() * SCALE_FACTOR),
                (int) (rightcolorSensor.green() * SCALE_FACTOR),
                (int) (rightcolorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        return hsvValues[0];
    }

    public boolean IsLeftSkystone(){
        return getLeftHue()>100.0; 
    } 

    public boolean IsRightSkystone(){
        return getRightHue()>100.0; 
    }

    //Positive is right. Negative is left.
    public void StrafeXFeet(double X, double strafespeed){
        SetMotorModes(false);
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        if(X<0){   //Left:
            while (frontleftmotor.getCurrentPosition()>674*X && opModeIsActive() && timeout.seconds()<4){
                PowerMotors(DirState.Left, strafespeed);
                }
            }
        else{   //Right:
            while (frontleftmotor.getCurrentPosition()<674*X && opModeIsActive() && timeout.seconds()<4){
                PowerMotors(DirState.Right, strafespeed);
            }
        }
        StopDriveTrain();
    }

    public double BearingDiff(double from_a,double to_a ){
        double R;
        R = (to_a - from_a) %360.0; 
        if(R >= 180){
            R = R - 360; 
        }
        return R; 
    }

    public double NormalizeBearing(double a){
        while(a > 180){
            a = a - 360; 
        }
        while(a < -180){
            a = a + 360;
        }
        return a; 
    }
    
    public void WaitNSeconds (double X){
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        while (opModeIsActive() &&
                (timeout.seconds() <X)) {
        }
    }

    public void turnNDegrees(double d){
        turnNDegrees(d, turnspeed); 
    }

    // Positive is clockwise
    public void turnNDegrees(double d, double turnspeed){
        double DesiredBearing=getCurrentAngle()-d; 
        turnNDegreesAbsolute(DesiredBearing, turnspeed);
    } 

    public void turnNDegreesAbsolute(double DesiredBearing){
        turnNDegreesAbsolute(DesiredBearing, turnspeed);    
    }

    public void turnNDegreesAbsolute(double DesiredBearing, double turnspeed){
        double error;
        DesiredBearing=NormalizeBearing(DesiredBearing); 
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        do{ 
            error=BearingDiff(getCurrentAngle(), DesiredBearing);
            double speed = Math.abs(error) * (1/120.0);
            speed = Math.max(speed, turnspeed/2.0); // Limit bottom
            speed = Math.min(speed, turnspeed); // Limit top
            if(error>0){
                //Turn left.
                PowerMotors(DirState.Left, speed);
            }
             else{
                //Turn right. 
                PowerMotors(DirState.Right, speed);
            }
        }   while(Math.abs(error)>2 && opModeIsActive()&& timeout.seconds()<5);  
        StopDriveTrain();
    }
    
    public double getCurrentAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        return currentAngle; 
    }

    public void takePicture(){
        /*
        // Richard will code this
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
              telemetry.addData("# Object Detected", updatedRecognitions.size());

              // step through the list of recognitions and display boundary info.
              int i = 0;
              for (Recognition recognition : updatedRecognitions) {
                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                  recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                if(recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
                    // This is the skystone:
                    double center = (recognition.getLeft() + recognition.getRight())/2;
                    detectedStone = (int)(Math.floor(center / 1024 * 3.0) + 1);
                    telemetry.addData("Detected Stone Pos:", detectedStone);
                }
              }
              telemetry.update();
            }
        }*/
    }
    
    public int getFirstSkystone(){
        // Richard will code this
        // Will either be 1, 2, or 3
        return detectedStone;
    }
    
    public int getSecondSkystone(){
        // Richard will code this
        // Will either be 4, 5, or 6
        return detectedStone+3;
    }
    
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }    
    
    private void initVision(){
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }
    
    public void runOpMode(){
        //initVision();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        frontleftmotor = hardwareMap.get(DcMotor.class, "flm");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frm");
        backleftmotor = hardwareMap.get(DcMotor.class, "blm"); 
        backrightmotor = hardwareMap.get(DcMotor.class, "brm");
        leftfoundationservo=hardwareMap.get(Servo.class, "lfs"); 
        rightfoundationservo=hardwareMap.get(Servo.class, "rfs");
        RightSideSensor=hardwareMap.get(DistanceSensor.class, "rss");
        LeftSideSensor= hardwareMap.get(DistanceSensor.class, "lss"); 
        BackSensor= hardwareMap.get(DistanceSensor.class, "bs"); 
        FrontSensor= hardwareMap.get(DistanceSensor.class, "fs"); 
        leftcolorSensor = hardwareMap.get(ColorSensor.class, "lcs");
        rightcolorSensor = hardwareMap.get(ColorSensor.class, "rcs");
        LeftSkystoneGrabber= hardwareMap.get(Servo.class, "lsg"); 
        RightSkystoneGrabber= hardwareMap.get(Servo.class, "rsg"); 
        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (leftcolorSensor instanceof SwitchableLight) {
          ((SwitchableLight)leftcolorSensor).enableLight(true);
        }
         if (rightcolorSensor instanceof SwitchableLight) {
          ((SwitchableLight)rightcolorSensor).enableLight(true);
        }
        frontrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backrightmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
    } 

    private void finish(){
        if (tfod != null) {
            tfod.shutdown();
        }
    }
}
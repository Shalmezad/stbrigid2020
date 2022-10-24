package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
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


public class BaseAuton extends LinearOpMode { 
    double strafespeed= 1*0.6;
    double drivespeed=1*0.6; 
    double turnspeed=1*0.30;
    double DriveSpeedSensor= 1*0.25;  //??
    double StrafeSpeedSensor= 1*0.35; //??
    double rdistanceoffset=0.5;
    double ldistanceoffset=0.5;
    double bdistanceoffset=0.5;
    double fdistanceoffset=0.5;        
    float hsvValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;

    DcMotor frontleftmotor = null; 
    DcMotor frontrightmotor = null; 
    DcMotor backleftmotor = null;
    DcMotor backrightmotor = null;
    Servo rightfoundationservo=null; 
    Servo leftfoundationservo= null; 
    DistanceSensor RightSideSensor=null; 
    DistanceSensor LeftSideSensor=null;
    DistanceSensor BackSensor=null; 
    DistanceSensor FrontSensor=null; 
    ColorSensor leftcolorSensor;
    ColorSensor rightcolorSensor;
    Servo LeftSkystoneGrabber=null; 
    Servo RightSkystoneGrabber=null; 


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

    public void DriveXFeet(double X){
        DriveXFeet(X, drivespeed);
    }
    
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
                frontleftmotor.setPower(0); 
                frontrightmotor.setPower(0); 
                backleftmotor.setPower(0);
                backrightmotor.setPower(0);   
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
            
    public double getRightDistanceInches(){
        return RightSideSensor.getDistance(DistanceUnit.INCH); 
    }
    
    public double getLeftDistanceInches(){
      return LeftSideSensor.getDistance(DistanceUnit.INCH);  
    }
    
    public double getBackDistanceInches(){
      return BackSensor.getDistance(DistanceUnit.INCH);     
    }
    
    public double getFrontDistanceInches(){
     return FrontSensor.getDistance(DistanceUnit.INCH);        
    }
    
    public void DriveRightSideToXInches( double X){
        DriveRightSideToXFeet(X/12.0); 
    }
    
    public void DriveLeftSideToXInches( double X){
        DriveLeftSideToXFeet(X/12.0); 
    }
    
    public void DriveBackToXInches(double X){
        DriveBackToXFeet(X/12.0);
    }
    
     //public void DriveBackToXInches(double X, double DriveSpeedSensor){
     //   DriveBackToXFeet(X/12.0, DriveSpeedSensor);
     //}
    public void DriveForwardToXInches(double X){
        DriveForwardToXFeet(X/12.0); 
    }
    
    public void DriveRightSideToXFeet(double X){
       frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
         frontleftmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER); 
         frontrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        frontrightmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
         backleftmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER); 
         backrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        backrightmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        
        while (getRightDistanceInches()>(X*12.0)+rdistanceoffset && opModeIsActive()){
                double error = getRightDistanceInches()-12 * X;
                double speed = error * (1/48.0);
                speed = Math.max(speed, StrafeSpeedSensor/1.8); // Limit bottom
                speed = Math.min(speed, StrafeSpeedSensor); // Limit top
              frontleftmotor.setPower(speed); 
              frontrightmotor.setPower(-1*speed);
              backleftmotor.setPower(-1*speed);
              backrightmotor.setPower(speed);
        } 
      
        StopDriveTrain();
    }
    
    public void DriveLeftSideToXFeet(double X){
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
         frontleftmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER); 
         frontrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        frontrightmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
         backleftmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER); 
         backrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        backrightmotor.setMode (DcMotor.RunMode.RUN_USING_ENCODER);
        
        while (getLeftDistanceInches()>(X*12)+ldistanceoffset && opModeIsActive()){
            double error = getLeftDistanceInches()-12 * X ;
                double speed = error * (1/48.0);
                speed = Math.max(speed, StrafeSpeedSensor/1.8); // Limit bottom
                speed = Math.min(speed, StrafeSpeedSensor); // Limit top
            frontleftmotor.setPower(-1*speed); 
            frontrightmotor.setPower(speed); 
            backleftmotor.setPower(speed);
            backrightmotor.setPower(-1*speed);  
        }
        StopDriveTrain();
        
    }
    
    public void DriveForwardToXFeet(double X){
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
         frontleftmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
         frontrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        frontrightmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
         backleftmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
         backrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        backrightmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        while(getFrontDistanceInches()>X*12 + fdistanceoffset && opModeIsActive()){
                double error = getFrontDistanceInches()-12 * X;
                double speed = error * (1/48.0);
                speed = Math.max(speed, DriveSpeedSensor/1.4); // Limit bottom
                speed = Math.min(speed, DriveSpeedSensor); // Limit top
                frontleftmotor.setPower(speed); 
                frontrightmotor.setPower(speed); 
                backleftmotor.setPower(speed);
                backrightmotor.setPower(speed);    
        }
        StopDriveTrain();
        
    }
    //public void DriveBackToXFeet(double X){
    //    DriveBackToXFeet(X, DriveSpeedSensor);     
    //}
    public void DriveBackToXFeet(double X){
        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        frontleftmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
        frontrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        frontrightmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        backleftmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
        backrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        backrightmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        while (getBackDistanceInches()>X*12+bdistanceoffset && opModeIsActive() && timeout.seconds() < 4){
            double error = getBackDistanceInches()-12 * X ;
            double speed = error * (1/48.0);
            speed = Math.max(speed, DriveSpeedSensor/1.4); // Limit bottom
            speed = Math.min(speed, DriveSpeedSensor); // Limit top
            frontleftmotor.setPower(-1*speed); 
            frontrightmotor.setPower(-1*speed); 
            backleftmotor.setPower(-1*speed);
            backrightmotor.setPower(-1*speed);    
        }
        StopDriveTrain();
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
        
    
    public void StrafeXFeet(double X){
        StrafeXFeet(X, strafespeed);
    }
    
    //Positive is right. Negative is left.
    public void StrafeXFeet(double X, double strafespeed){
         frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
         frontleftmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
         frontrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        frontrightmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
         backleftmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
         backrightmotor.setMode (DcMotor.RunMode. STOP_AND_RESET_ENCODER); 
        backrightmotor.setMode (DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        if(X<0){
            while (frontleftmotor.getCurrentPosition()>674*X && opModeIsActive() && timeout.seconds()<4){
            frontleftmotor.setPower(-1*strafespeed); 
            frontrightmotor.setPower(strafespeed); 
            backleftmotor.setPower(strafespeed);
            backrightmotor.setPower(-1*strafespeed);
        }
        
    }
    else{
        while (frontleftmotor.getCurrentPosition()<674*X && opModeIsActive() && timeout.seconds()<4){
            frontleftmotor.setPower(strafespeed); 
            frontrightmotor.setPower(-1*strafespeed);
            backleftmotor.setPower(-1*strafespeed);
            backrightmotor.setPower(strafespeed); 
        }
    }
    StopDriveTrain();
    }
    public double BearingDiff(double from_a,double to_a ){
        double R;
        R=(to_a-from_a)%360.0; 
        if(R>=180){
            R=R-360; 
        }
        return R; 
        
        
    }
    public double NormalizeBearing(double a){
        while(a>180){
        a=a-360; 
        
        }
        while(a<-180){
            a=a+360;
            
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
         DesiredBearing=NormalizeBearing(DesiredBearing); 
       double error;
               ElapsedTime timeout = new ElapsedTime();
         timeout.reset();
       do{ 
        error=BearingDiff(getCurrentAngle(), DesiredBearing);
                double speed = Math.abs(error) * (1/120.0);
                speed = Math.max(speed, turnspeed/2.0); // Limit bottom
                speed = Math.min(speed, turnspeed); // Limit top
        if(error>0){
            //Turn left. 
            frontleftmotor.setPower(-1*speed); 
            frontrightmotor.setPower(speed); 
            backleftmotor.setPower(-1*speed);
            backrightmotor.setPower(speed);
        }
        else{
            //Turn right. 
            frontleftmotor.setPower(speed); 
            frontrightmotor.setPower(-1*speed); 
            backleftmotor.setPower(speed);
            backrightmotor.setPower(-1*speed);
        }
       } while(Math.abs(error)>2 && opModeIsActive()&& timeout.seconds()<5);  
        
            frontleftmotor.setPower(0); 
            frontrightmotor.setPower(0); 
            backleftmotor.setPower(0);
            backrightmotor.setPower(0); 
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
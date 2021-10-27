
package org.firstinspires.ftc.teamcode;
import java.util.List;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Auto with Cam", group="Linear Opmode")

public class AutoWithCam extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    private DcMotor lf,lb,rb,rf;
    private DcMotor arm = null;
    private DcMotor launcher1, intake;
    private TouchSensor touch;
    private double inch = 87.5;
    private DcMotorEx launcher2;
    private Servo grab;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
             "AeOuPaX/////AAABmebKpI9d8EyFsdPTMDjvCkB0wFESJpSWHlnyJ8jXj3U1/pY/+rWqhr36k9GxU+6BuK0+qloTKs7VV1R8l+cRgcooB/qBIqmAUnfH49n3TgXoUXCClsbQ2gYEOeINowutk2vkpurFU8f+QGUuI038Pcb7av2eCMvgi8oAdufjhBMfjQ57s79F3HbqQD758kak4R7x8Fbj3k32cotjSxsziZ7N0aNPUdZy/v2GX7MKu1ZGdqrDzOkR/ClVPnwIXkF27VOseqeHJQavMg2H5M90zABYnS9C5fEDwVYmIwsGLhoYsRSEFHqiio1lVhhtDHFmoEJnQQ3VuDP5xwWpv3SyTNwXOFyV9xnJL6r3Q12itF71";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    Orientation angles;
    @Override
    public void runOpMode() {
       
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0/9.0);
        }
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf  = hardwareMap.get(DcMotor.class, "lf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        intake = hardwareMap.get(DcMotor.class,"intake");

        launcher1 = hardwareMap.get(DcMotor.class,"l1");
        launcher2 = hardwareMap.get(DcMotorEx.class,"l2");
        grab=hardwareMap.get(Servo.class, "grab");
        arm = hardwareMap.get(DcMotor.class,"arm");
        touch = hardwareMap.get(TouchSensor.class, "touch");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        launcher1.setDirection(DcMotor.Direction.REVERSE);
        launcher2.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        //runtime.reset();
        if (opModeIsActive()) {
            int counter = 0;
            String targetZone = "";
            while (opModeIsActive()) {
                counter++;
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 0)
                      {
                          targetZone = "A";
                      }
                      // step through the list of recognitions and display boundary info.
                      int i = 0;
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        if (recognition.getLabel().equals("Quad"))
                        {
                          targetZone = "C";
                        }
                        else if (recognition.getLabel().equals("Single"))
                        {
                          targetZone = "B";
                        }
                        else{
                            targetZone = "UNKNOWN";
                        }
                      }
                      telemetry.addData("Target Zone: ", targetZone);
                      telemetry.update();
                      
                    }
                }
                if (counter >= 5000)
                    break;
                
            }
            //break to here
            if (targetZone.equals("C"))
            {
                //3 powershots:1WG,PARK = 65pts
                    grab.setPosition(1); sleep(500);
                    
                    forward(2750,.5,.7,.9,.4,.333,.333);
                    left(-125,.2,.5,.7,.3,.333,.333);

                    launcher2.setVelocity(-1800);sleep(1500);
                    midtake(150);
                    left(300,.2,.5,.7,.3,.333,.333);
                    //launcher2.setPower(-.9);
                    launcher2.setVelocity(-1850);sleep(500); 
                    midtake(500);
                    left(350,.2,.5,.7,.3,.333,.333);
                    launcher2.setVelocity(-1900);sleep(500);
                    midtake(1000);
                    left(-1900,.5,.7,.9,.7,.333,.333);
                    forward(2200,.2,.5,.9,.9,.333,.333);
                    dropWG();
                    forward(-1500,.2,.5,.9,.9,.333,.333);
                    /*forwardNOE(-2000);
                    left(-200,.5,.7,.9,.7,.333,.333);
                    forwardNOE(-1500);
                    */
                    
                    
                    /*left(-1000,.2,.5,.9,.9,.333,.333);
                    forward(-1500,.2,.5,.9,.9,.333,.333);
                    left(800,.2,.5,.9,.9,.333,.333);*/

            }
            if (targetZone.equals("B"))
            {
                    grab.setPosition(1); sleep(500);
                    
                    forward(2750,.5,.7,.9,.4,.333,.333);
                    left(-150,.2,.5,.7,.3,.333,.333);

                    launcher2.setVelocity(-1800);sleep(1500);
                    midtake(150);
                    left(400,.2,.5,.7,.3,.333,.333);
                    //launcher2.setPower(-.9);
                    launcher2.setVelocity(-1850);sleep(500); 
                    midtake(500);
                    left(250,.2,.5,.7,.3,.333,.333);
                    launcher2.setVelocity(-1900);sleep(500);
                    midtake(1000);
                    left(-850,.5,.7,.9,.7,.333,.333);
                    forward(1000,.2,.5,.9,.5,.333,.333);
                    dropWG();
                    
                    forward(-3000,.2,.5,.9,.5,.333,.333);
                    
                    grab.setPosition(0.5);
                    sleep(1000);
                    arme(-400);
                    left(-1000,.5,.7,.9,.7,.333,.333);
                    pick2WG();
                    left(1350,.5,.7,.9,.7,.333,.333);
                    forward(3300,.2,.5,.9,.7,.333,.333);
                    grab.setPosition(.5);sleep(500);
                    arm.setPower(1);sleep(500);
                    forward(-800,.2,.5,.9,.9,.333,.333);

                    

        }
        if (targetZone.equals("A"))
            {
                    grab.setPosition(1); sleep(500);
                    
                    forward(2750,.5,.7,.9,.4,.333,.333);
                    left(-100,.2,.5,.7,.3,.333,.333);

                    launcher2.setVelocity(-1800);sleep(1500);
                    midtake(150);
                    left(300,.2,.5,.7,.3,.333,.333);
                    //launcher2.setPower(-.9);
                    launcher2.setVelocity(-1850);sleep(500); 
                    midtake(500);
                    left(350,.2,.5,.7,.3,.333,.333);
                    launcher2.setVelocity(-1900);sleep(500);
                    midtake(1000);
                    left(-2000,.5,.7,.9,.7,.333,.333);
                    forward(300,.2,.5,.9,.9,.333,.333);
                    dropWG();
                    
                    left(500,.5,.7,.9,.7,.333,.333);

                    forward(-2400,.2,.5,.9,.5,.333,.333);
                    pickWG();
                    arm.setPower(1);
                    forward(2700,.2,.5,.9,.5,.333,.333);
                    sleep(1000);
                    left(-200,.5,.7,.9,.7,.333,.333);
                    grab.setPosition(0.5);sleep(1000);
                    arm.setPower(.5);sleep(500);
    

        }
        
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    
    
    public void dragWG(double close, int neg)
    {
        arme(neg);
        grab.setPosition(close);

    }
    public void pickWG()
    {
            grab.setPosition(0.5);
            sleep(1000);
            arme(-400);
            left(-250,.2,.5,.9,.9,.333,.333);

            forward(-150,.2,.5,.9,.9,.333,.333);

            grab.setPosition(1);

            sleep(1500);
            
            //arme(500);
    }
    public void pick2WG()
    {
            
            //left(-250,.2,.5,.9,.9,.333,.333);

            forward(-250,.2,.5,.9,.9,.333,.333);

            grab.setPosition(1);

            sleep(1500);
            
            //arme(500);
    }
    public void dropWG()
    {
            
            arme(-500);
            grab.setPosition(0.5);
            sleep(500);
            arme(400);
    }
    public void arme(int amt) {

        arm.setTargetPosition(arm.getCurrentPosition() + amt);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double pow = 0.85;
        long to = System.currentTimeMillis();
        while (arm.isBusy() && opModeIsActive()) {
          arm.setPower(1); 
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void midtake(int amt) {
        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcher1.setTargetPosition(launcher1.getCurrentPosition() + amt);
        launcher1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double pow = 0.85;
        long to = System.currentTimeMillis();
        while (launcher1.isBusy() && opModeIsActive()) {
          launcher1.setPower(1); 
        }
        launcher1.setPower(0);
        launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void forward(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
        
        lf.setTargetPosition(lf.getCurrentPosition() - amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);
        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() - amt);

        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        while ((lb.isBusy() && lf.isBusy() && rf.isBusy() && rb.isBusy()) && opModeIsActive()) {
            double prop = (Math.abs(amt) - Math.abs(lb.getTargetPosition() - lb.getCurrentPosition())) / (double) amt;
            double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);

            lf.setPower(adjPower);
            lb.setPower(adjPower);
            rf.setPower(adjPower);
            rb.setPower(adjPower);
        }
        stopRobot();
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void forwardNOE(int amt) {
        
        lf.setTargetPosition(lf.getCurrentPosition() - amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);
        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() - amt);

        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while ((lb.isBusy() && lf.isBusy() && rf.isBusy() && rb.isBusy()) && opModeIsActive()) {
            //double prop = (Math.abs(amt) - Math.abs(lb.getTargetPosition() - lb.getCurrentPosition())) / (double) amt;
            //double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);

            lf.setPower(1);
            lb.setPower(1);
            rf.setPower(1);
            rb.setPower(1);
        }
        stopRobot();
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void left(int amt, double edge1, double edge2, double max, double min, double p1, double p2) {
        
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);
        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);

        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        while ((lb.isBusy() && lf.isBusy() && rf.isBusy() && rb.isBusy()) && opModeIsActive()) {
            double prop = (Math.abs(amt) - Math.abs(lb.getTargetPosition() - lb.getCurrentPosition())) / (double) amt;
            double adjPower = getPower(prop, edge1, edge2, max, min, p1, p2);

            lf.setPower(adjPower);
            lb.setPower(adjPower);
            rf.setPower(adjPower);
            rb.setPower(adjPower);
        }
        stopRobot();
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void forwardamt(double amt)
    {
        lf.setPower(-amt);
        lb.setPower(-amt);
        rb.setPower(-amt);
        rf.setPower(-amt);
    }
    public void rightamt(double amt)
    {
        lf.setPower(amt);
        lb.setPower(-amt);
        rb.setPower(amt);
        rf.setPower(-amt);
    }
    public void stopRobot()
    {
        lf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        rf.setPower(0);
        sleep(500);
    }
    public double getPower(double amtDone, double edge1, double edge2, double max, double min, double p1, double p2) {

        //Determine power based on an adjustable curve metric inspired by the Normal Distribution
        if (amtDone >= edge1 && amtDone <= 1 - edge2) {

            //We've accelerated and are in the middle of our motion, so we're at max power.
            return max;

        } else {

            if (amtDone > 1 - edge2) {

                //Last edge... what's our power?
                amtDone = 1 - amtDone;
                double amtNormDone = amtDone / edge2;
                return min + evaluateNormal(1, p2, amtNormDone, max - min);

            } else {

                //How much of the way through are we, and what power should we be on?
                double amtNormDone = amtDone / edge1;
                return min + evaluateNormal(1, p1, amtNormDone, max - min);

            }

        }

    }
    public double evaluateNormal(double mu, double sigma, double x, double max) {

        //Return an adjusted Normal Distribution value such that the maximum possible value is "max"
        double exponent = -Math.pow(x - mu, 2) / (2 * Math.pow(sigma, 2));
        return max * Math.pow(Math.E, exponent);

    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}

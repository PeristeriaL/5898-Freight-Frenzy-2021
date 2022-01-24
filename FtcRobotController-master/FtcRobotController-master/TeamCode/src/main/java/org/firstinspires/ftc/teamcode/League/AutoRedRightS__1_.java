package org.firstinspires.ftc.teamcode;
import java.util.List;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import com.qualcomm.robotcore.hardware.DcMotorEx;

//tfod stuff
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
 
 @Autonomous(name="Cam Red Right- use SPEEED", group="Red")
//@Disabled
public class AutoRedRightS__1_ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor lf,lb,rb,rf;
    private DcMotor screw, rotate;
    private DcMotor spin;
    private CRServo in;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite"; // ty ftc for not changing file names year to year <3
    private static final String[] LABELS = {
      "Box",
      "Duck",
      "Ball",
      "Marker"
    };
    
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
            tfod.setZoom(1.30, 16.0/9.0);
        }
        
        telemetry.addData("> ", "wassup, press play to start B)");
        telemetry.update();
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf  = hardwareMap.get(DcMotor.class, "lf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rotate = hardwareMap.get(DcMotor.class, "rotate");
        screw = hardwareMap.get(DcMotor.class,"screw");
        spin = hardwareMap.get(DcMotor.class,"spin");
        in = hardwareMap.get(CRServo.class,"in");
        
        rotate.setDirection(DcMotor.Direction.REVERSE);
        
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        //runtime.reset();
        if (opModeIsActive()) {
            int counter = 0; // loop control variable so thanos doesnt search for ducks forever
            String level = ""; // variable to change what level thanos goes to
            
            while (opModeIsActive()) {
                counter++;
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    
                    if (updatedRecognitions != null) {
                      telemetry.addData("> check it out! i see this many things: ", updatedRecognitions.size());
                                        //"Object Detected : [amount of things seen, should be 1 or 0]"
                                        
                      // step through the list of recognitions and display boundary info.
                      for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", counter), recognition.getLabel());
                                        //"label([counter]) : [thing seen]"
                                        
                        telemetry.addData(String.format("  left,top (%d)", counter), "%.03f , %.03f",
                                          recognition.getLeft(), recognition.getTop());
                                        //"left,top([counter]) : [someFloat1] , [someFloat2]"
                                        // if float1 > float2, the duck is to the left
                                        // else, its to the right
                        

                        if(recognition.getLeft() < recognition.getTop()){ // left side
                            level = "middle";
                        }
                        else if(recognition.getLeft() > recognition.getTop()){ // rigjht side
                            level = "top";
                        }
                        else{
                            level = "bottom";
                        }
                

                      }
                      telemetry.update();
                    }
                
                }
                if (counter >= 5000)
                    break;
                
            }
            //break to here
            
           telemetry.addData("alrighty, ima put this block on shipping hub level ",level);
           telemetry.update();
           
           
           if(level.equals("top")){
               //wobble goal tape 1 code-> [duck   tape    tape]
               telemetry.addData("Placing Block on ",level);
               telemetry.update();
               top();
           }
           else if(level.equals("middle")){
               //wobble goal tape 2 code-> [tape    duck    tape]
               telemetry.addData("Placing Block on ",level);
               telemetry.update();
               mid();
           }
           else{
               //wobble goal tape 3 code-> [tape    tape    duck]
               telemetry.addData("Placing Block on ",level);
               telemetry.update();
               lowest();
           }
    
        
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        
    }
    
    
    
///////// camera init methods //////////////////////////////////////////////////

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "DuckSniper");

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
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    
////////////////////////////////////////////////////////////////////////////////    
////// movement methods ////////////////////////////////////////////////////////
    
    public void up(int amt, double power){
    
        screw.setTargetPosition(screw.getCurrentPosition() + amt);
        screw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
        while(opModeIsActive() && (screw.isBusy())){
            telemetry.addData("Mode" , "running");
            telemetry.update();  
            screw.setPower(power);
        }
    
        screw.setPower(0);
        screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    
    public void rotate(int amt, double power){
        rotate.setTargetPosition(rotate.getCurrentPosition() - amt);
        rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    
        while(opModeIsActive() && (rotate.isBusy())){
            telemetry.addData("Mode" , "running");
            telemetry.update();  
            rotate.setPower(power);
        }
    
        rotate.setPower(0);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public void outtake(){
        in.setPower(1.0);
        sleep(2500);
        
        //stop the intake
        in.setPower(0.0);
        sleep(200);
    }
    
    public void warehouse(){
        //back up slightly to not hit the goal
        backAmt(400,.5);
        sleep(700);
        stopRobot();
        
        //turn right to go into the warehouse
        rightAmt(1000,.4);
        stopRobot();
        
        //go into warehouse
        forwardAmt(2500,.9);
    }
    
    public void approachGoal(){
        forwardAmt(90,.6);
        strafe(-1200,.4);
        forwardAmt(750,.55);
        sleep(250);
    }
    

    public void forwardAmt(int amt, double power){
        
        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);
        
        runToPosition();
        
        
        while(opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy() )){
            telemetry.addData("Mode" , "running");
            telemetry.update();  
            allPower(power);
        }
        
        stopRobot();
        runUsingEncoder();
    }
    
    public void backAmt(int amt, double power){
        
        rf.setTargetPosition(rf.getCurrentPosition() - amt);
        rb.setTargetPosition(rb.getCurrentPosition() - amt);
        lf.setTargetPosition(lf.getCurrentPosition() - amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);
        
        runToPosition();
        
        while(opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy() )){
        
            telemetry.addData("Mode" , "running");
            telemetry.update();  
            allPower(power);
        
        }
        stopRobot();
        runUsingEncoder();
    }
    
    public void rightAmt(int amt, double power){
        
        rf.setTargetPosition(rf.getCurrentPosition() - amt);
        rb.setTargetPosition(rb.getCurrentPosition() - amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() + amt);
        
        runToPosition();
        
        while(opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy() )){
        
            telemetry.addData("Mode" , "running");
            telemetry.update();  
            allPower(power);
        
        }
        stopRobot();
        runUsingEncoder();
    }
    
    
    public void leftAmt(int amt, double power){
        
        rf.setTargetPosition(rf.getCurrentPosition() + amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() - amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);
        
        runToPosition();
        
        while(opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy() )){
        
            telemetry.addData("Mode" , "running");
            telemetry.update();  
            allPower(power);
        
        }
        stopRobot();
        runUsingEncoder();
    }
    
    public void strafe(int amt, double power){
        rf.setTargetPosition(rf.getCurrentPosition() - amt);
        rb.setTargetPosition(rb.getCurrentPosition() + amt);
        lf.setTargetPosition(lf.getCurrentPosition() + amt);
        lb.setTargetPosition(lb.getCurrentPosition() - amt);
        
        runToPosition();
        
        while(opModeIsActive() && (rf.isBusy() && rb.isBusy() && lf.isBusy() && lb.isBusy() )){
        
            telemetry.addData("Mode" , "running");
            telemetry.update();  
            allPower(power);
        
        }
        stopRobot();
        runUsingEncoder();
    }
    
    
    
    //set mode for all motors
    public void runUsingEncoder(){
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    public void runToPosition(){
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    
    //stop all movement 
    public void stopRobot(){
        rf.setPower(0);
        lf.setPower(0);
        rb.setPower(0);
        lb.setPower(0);
    }
    
    //set all power at once
    public void allPower(double power){
        rf.setPower(power);
        rb.setPower(power);
        lf.setPower(power);
        lb.setPower(power);
    }
    
    
    
    
// find out the randomizations so that the duck code goes to the right level

    public void lowest(){
        approachGoal();
        outtake();
        warehouse();
    }
    
    public void mid(){
        forwardAmt(90,.6); //get away from the wall
        strafe(-1100,.4); // strafe to the goal
        up(1500,.6); //go up        
        forwardAmt(750,.55); //approach goal        
        rotate(440,.3); //rotate to the top goal
        outtake(); 
        forwardAmt(-100, .3); //backup so it wont hit the stupid goal   
        
        //if possible - run multiple things at once so that speeeeeeeeeeeeed
        
        rotate(-100,.3); //rotate back
        up(-1500,.6);
            
        warehouse();
    }
    
    public void top(){
        forwardAmt(90,.6);//get off the wall
        strafe(-1100,.4);//strafe to be in front of the goal
        rotate(440,.3);//rotate from start position to be upright
        up(2630,.6); //bring the bucket
        forwardAmt(750,.55); //go forward to the goal
        rotate(100,.3);//rotate for drop position
        outtake();        
        rotate(-200,.3);//rotate back to upright
        backAmt(100,.2);//back up a little
        sleep(500);
        up(-2700,.6);//bring the bucket back down
        
        warehouse();
    }

}
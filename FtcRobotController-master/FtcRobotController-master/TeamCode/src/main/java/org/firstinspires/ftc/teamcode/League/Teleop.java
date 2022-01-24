package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Lead Screw", group="Iterative Opmode")

public class Teleop extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lb, lf, rb, rf, spin, screw, rotate;
    private CRServo in;
    private int counter = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        int soundID = hardwareMap.appContext.getResources().getIdentifier("imposters among", "raw", hardwareMap.appContext.getPackageName());

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        in = hardwareMap.get(CRServo.class, "in");
        spin = hardwareMap.get(DcMotor.class, "spin");
        screw = hardwareMap.get(DcMotor.class, "screw");
        rotate = hardwareMap.get(DcMotor.class,"rotate");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lb.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.REVERSE);


        //rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Setup a variable for each drive wheel to save power level for telemetry


        double r = Math.hypot(gamepad1.right_stick_x - gamepad1.left_stick_x, gamepad1.right_stick_y - gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.right_stick_y - gamepad1.left_stick_y, -gamepad1.right_stick_x + gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_trigger - gamepad1.left_trigger;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        lf.setPower(-v1);
        rf.setPower(-v2);
        lb.setPower(-v3);
        rb.setPower(-v4);
        
        //duck wheel on a / b press
        if (gamepad2.a) {
            spin.setPower(.5); //.3 way too fast. .2 enough but .25 is speed with safety
        }                       
        else if (gamepad2.b) {
            spin.setPower(-.5);
        }
        else {
            spin.setPower(0.0);
        }
        
       // if (gamepad2.x){
         //   SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        //}
        
        
        //intake
        if (gamepad2.x) {
            in.setPower(1.0);
        }
        else if (gamepad2.y) {
            in.setPower(-1.0);
        }
        else {
            in.setPower(0.0);
        }
        
        
        if (gamepad2.dpad_down) {
           rotate.setPower(-.5);
        }
        else if (gamepad2.dpad_up) {
          rotate.setPower(.5);
        }
        else {
            rotate.setPower(0.0);
        }
        
        if (gamepad2.right_bumper) {
            screw.setPower(1);
        }
        else if (gamepad2.left_bumper) {
            screw.setPower(-1);
        }
        else {
            screw.setPower(0.0);
        }
        
      
    }

}

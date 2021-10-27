package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name="Teleop4Bennett", group="Iterative Opmode")

public class Teleop4Bennett extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lb, lf, rb, rf, arm,launcher1, launcher2;
    private Servo grab;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        arm = hardwareMap.get(DcMotor.class,"arm");
        launcher1 = hardwareMap.get(DcMotor.class,"l1");
        launcher2 = hardwareMap.get(DcMotor.class,"l2");

        grab=hardwareMap.get(Servo.class, "grab");
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        launcher1.setDirection(DcMotor.Direction.REVERSE);
        launcher2.setDirection(DcMotor.Direction.FORWARD);
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
        double launcher1power;
        double l1p = -gamepad2.left_trigger;
        launcher1power   = Range.clip(l1p, -1.0, 1.0) ;
        double launcher2power;
        double l2p = -gamepad2.right_trigger;
        launcher2power   = Range.clip(l2p, -1.0, 1.0) ;
    
        // Setup a variable for each drive wheel to save power level for telemetry


        double r = Math.hypot(gamepad1.left_stick_x - gamepad1.right_stick_x, gamepad1.left_stick_y - gamepad1.right_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y - gamepad1.right_stick_y, -gamepad1.left_stick_x + gamepad1.right_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_trigger - gamepad1.left_trigger;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        lf.setPower(-v1);
        rf.setPower(-v2);
        lb.setPower(-v3);
        rb.setPower(-v4);
        double armp;
        arm.setPower(0);
        
        if(gamepad2.y)
        {
            arm.setPower(1);

        }
        if(gamepad2.x)
        {
            arm.setPower(-.5);
        }
        
        if(gamepad2.right_bumper)
        {
            grab.setPosition(1);
        }
        if(gamepad2.left_bumper)
        {
            grab.setPosition(0.5);
        }
        if(gamepad2.left_bumper)
        {
            grab.setPosition(0.5);
        }
        if(gamepad2.b)
        {
            launcher1.setPower(1);
        }
        launcher1.setPower(launcher2power);
        launcher2.setPower(launcher1power);
        //else
        //{
        //    grab.setPosition(1);
        //}
        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels


        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("arm working", "%.2f", arm);
        //telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}

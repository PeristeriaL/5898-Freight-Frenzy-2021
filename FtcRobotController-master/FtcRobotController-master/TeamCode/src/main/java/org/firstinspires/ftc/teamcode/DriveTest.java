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

@TeleOp(name="DriveTest", group="Iterative Opmode")

public class DriveTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lb, lf, rb, rf, spin;
    private DcMotor arm1, arm2;
    private CRServo in;
    private int counter = 0;

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
        spin = hardwareMap.get(DcMotor.class, "spin");
        
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");

        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);


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
        
        if (gamepad2.a) {
            lf.setPower(-.5);
            rf.setPower(.5);
            lb.setPower(-.5);
            rb.setPower(.5);
        }
        else if (gamepad2.b) {
            lf.setPower(.5);
            lb.setPower(-.5);
            rb.setPower(.5);
            rf.setPower(-.5);
        }
        else if (gamepad2.x) {
            lf.setPower(-.5);
            lb.setPower(-.5);
            rb.setPower(-.5);
            rf.setPower(-.5);
        }
        else if (gamepad2.y) {
            lf.setPower(.5);
            lb.setPower(.5);
            rb.setPower(.5);
            rf.setPower(.5);
        }
        else {
            lf.setPower(0.0);
            lb.setPower(0.0);
            rb.setPower(0.0);
            rf.setPower(0.0);
        }
        
        
      
    }

  
}


package org.firstinspires.ftc.teamcode;

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
@Autonomous(name="LEFT_START_POSITION", group="Linear Opmode")

public class NOPOWERSHOTS extends LinearOpMode {

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
    Orientation angles;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO55IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        
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
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        

            // Setup a variable for each drive wheel to save power level for telemetry
            //launcher2.setVelocity(-1800);sleep(1500);
            
            //forward(150);
            //forward(-30);
            //13.33
            grab.setPosition(1); sleep(500);
            forward(2750,.5,.7,.9,.4,.333,.333);
            launcher2.setVelocity(-1800);sleep(1500);
            midtake(150);
            left(225,.2,.5,.7,.3,.333,.333);
            //launcher2.setPower(-.9);
            launcher2.setVelocity(-1850);sleep(500); 
            midtake(500);
            left(350,.2,.5,.7,.3,.333,.333);
            launcher2.setVelocity(-1900);sleep(500);
            midtake(1000);
            left(-2000,.5,.7,.9,.7,.333,.333);
            forward(200,.2,.5,.9,.9,.333,.333);
            dropWG();
            

            left(1400,.5,.7,.9,.5,.333,.333);
            forward(-2200,.7,.9,.9,.5,.333,.333);
            arme(-400);
            left(-500,.5,.7,.9,.5,.333,.333);
            forward(-150,.5,.7,.9,.7,.333,.333);
            grab.setPosition(1);sleep(1000);
            arm.setPower(1);sleep(1000);
            left(-550,.5,.7,9.,.5,.333,.333);

            forward(2600,.5,.7,.9,.7,.333,.333);
            
            left(300,.5,.7,.9,.7,.333,.333);
            grab.setPosition(0);
            
            //launcher2.setPower(-.9);
            //sleep(1000);
            //midtake(400);
            //launcher2.setPower(1);
            //sleep(1000);
            //launcher1.setPower(.3);
            //sleep(1000);
            
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "leftDf (%.2f),leftDb (%.2f),rightDf (%.2f) rightDb (%.2f)", leftDf,leftDb,rightDf,rightDb);
            //telemetry.update();
        
    }
    public void thirteen()
    {
            grab.setPosition(1);
            forwardamt(1);
            sleep(1200);//
            stopRobot();
            rightamt(-.2);
            sleep(1000);
            stopRobot();
            dropWG();
            rightamt(.2);
            sleep(800);
            forwardamt(-1);
            sleep(950);//
            stopRobot();
            rightamt(-.5);
            sleep(1050);
            stopRobot();
            dragWG(0,-500);
            forwardamt(-.15);
            sleep(500);
            dragWG(1,500);
            rightamt(-.4);//right after grab second
            sleep(1000);
            arm.setPower(1);//lift
            forwardamt(1);
            sleep(1000);
            stopRobot();
            rightamt(.2);//
            sleep(1000);
            arm.setPower(-1);
            grab.setPosition(0);
            sleep(500);
            arm.setPower(1);
            sleep(500);
            forwardamt(-.1);
            sleep(1000);
            rightamt(.325);//aim for powershots
            sleep(1000);
            stopRobot();
            launcher2.setPower(-1);
            sleep(2000);
            launcher2.setPower(-.9);
            launcher1.setPower(-1);
            sleep(1000);
            launcher1.setPower(0);
            launcher2.setPower(0);
            sleep(1000);
            forwardamt(.3);
            sleep(1000);
            
    }
    public void twelve()
    {
            grab.setPosition(1);
            forwardamt(1);
            sleep(1300);
            stopRobot();
            rightamt(-.2);
            sleep(1000);
            stopRobot();
            dropWG();
            forwardamt(-1);
            sleep(1000);
            stopRobot();
            rightamt(-.4);
            sleep(1050);
            stopRobot();
            dragWG(0,-500);
            forwardamt(-.2);
            sleep(500);
            dragWG(1,500);
            rightamt(-.3);//
            sleep(1000);
            arm.setPower(1);
            forwardamt(1);
            sleep(1000);
            stopRobot();
            rightamt(-.2);
            dragWG(0,0);
            forwardamt(-.2);
            sleep(1000);
            rightamt(.7);
            launcher2.setPower(-1);
            sleep(1000);
            launcher1.setPower(.2);
            sleep(1000);
            //new addition: with powershots
            /*rightamt(.3);
            sleep(1000);
            stopRobot();
            launcher2.setPower(-1);
            sleep(2000);
            launcher2.setPower(-1);
            launcher1.setPower(-1);
            sleep(1000);
            launcher1.setPower(0);
            launcher2.setPower(0);
            sleep(1000);
            */
    }
    
    public void dragWG(double close, int neg)
    {
        grab.setPosition(close);
        sleep(1000);
        arme(neg);
    }
    public void pickWG()
    {
            grab.setPosition(0);
            sleep(1000);
            arme(-500);
            grab.setPosition(1);
            sleep(1000);
            arme(500);
    }
    public void dropWG()
    {
            
            arme(-500);
            grab.setPosition(0);
            sleep(500);
            arme(500);
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

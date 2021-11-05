/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



//imports all the packages. dont change this, its got all u need to do stuff
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


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

//tells driver hub what to name the thing on the screen. it also tell it that its op mode instead of auto.
@Autonomous(name="auto red center sleep", group="Linear Opmode")


// makes a class to run ur swag code
public class RedAutoCenterSleep extends LinearOpMode {

    // Declare OpMode members. this just creates 'objects' for you to do stuff with
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor lf = null;
    private DcMotor lb = null;
    private DcMotor rf = null;
    private DcMotor rb = null;
    private DcMotor arm1 = null;
    private DcMotor arm2 = null;
    private DcMotor spin = null;

    private CRServo in = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf  = hardwareMap.get(DcMotor.class, "lf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        spin = hardwareMap.get(DcMotor.class,"spin");
        in = hardwareMap.get(CRServo.class,"in");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)

        //wait 10 milliseconds so our alliance can do their thing.
        //this is the portion that is different than the other red auto center
        sleep(10000);



        //go forward to the goal
        forwardamt(.43);
        sleep(950);
        stopRobot();

        //outtake block like cubes in minecraft
        sleep(500);
        in.setPower(-1.0);
        sleep(3000);

        //stop the intake
        in.setPower(0.0);
        sleep(500);

        //back up slightly to not hit the goal
        backamt(.2);
        sleep(700);
        stopRobot();

        //turn right to go into the warehouse
        turnrightamt(.4);
        sleep(1250);
        stopRobot();

        //go into warehouse
        forwardamt(.8);
        sleep(1700);
        stopRobot();

        //back up a tad public void stop()
        //backamt(.4);
        //sleep(500);
        //stopRobot();


        // Setup a variable for each drive wheel to save power level for telemetry

    }



    public void backamt(double amt)
    {
        lf.setPower(-amt);
        lb.setPower(-amt);
        rb.setPower(-amt);
        rf.setPower(-amt);
    }

    public void forwardamt(double amt)
    {
        lf.setPower(amt);
        lb.setPower(amt);
        rb.setPower(amt);
        rf.setPower(amt);
    }

    public void turnleftamt(double amt)
    {
        lf.setPower(-amt);
        lb.setPower(-amt);
        rb.setPower(amt);
        rf.setPower(amt);
    }

    public void turnrightamt(double amt)
    {
        lf.setPower(amt);
        lb.setPower(amt);
        rb.setPower(-amt);
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


}

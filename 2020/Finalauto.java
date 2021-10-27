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


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
@Autonomous(name="Autonomousv2", group="Linear Opmode")

public class Finalauto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDf = null;
    private DcMotor leftDb = null;
    private DcMotor rightDf = null;
    private DcMotor rightDb = null;
    private DcMotor arm = null;
    private DcMotor launcher1, launcher2;

    private Servo grab;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDf  = hardwareMap.get(DcMotor.class, "lf");
        leftDb  = hardwareMap.get(DcMotor.class, "lb");
        rightDf = hardwareMap.get(DcMotor.class, "rf");
        rightDb = hardwareMap.get(DcMotor.class, "rb");
        launcher1 = hardwareMap.get(DcMotor.class,"l1");
        launcher2 = hardwareMap.get(DcMotor.class,"l2");
        grab=hardwareMap.get(Servo.class, "grab");
        arm = hardwareMap.get(DcMotor.class,"arm");



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        launcher1.setDirection(DcMotor.Direction.REVERSE);
        launcher2.setDirection(DcMotor.Direction.FORWARD);
        leftDf.setDirection(DcMotor.Direction.REVERSE);
        leftDb.setDirection(DcMotor.Direction.FORWARD);
        rightDf.setDirection(DcMotor.Direction.REVERSE);
        rightDb.setDirection(DcMotor.Direction.FORWARD);
        
        rightDf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        

            // Setup a variable for each drive wheel to save power level for telemetry
            grab.setPosition(1);
            leftDf.setPower(-.7);
            leftDb.setPower(-.7);
            rightDb.setPower(-.7);
            rightDf.setPower(-.7);
            sleep(1400);
            leftDf.setPower(0);
            leftDb.setPower(0);
            rightDb.setPower(0);
            rightDf.setPower(0);
            sleep(1400);
            leftDf.setPower(-.5);
            leftDb.setPower(.5);
            rightDb.setPower(-.5);
            rightDf.setPower(.5);
            sleep(1400);
            leftDf.setPower(0);
            leftDb.setPower(0);
            rightDb.setPower(0);
            rightDf.setPower(0);
            arm.setPower(-0.4);
            sleep(500);
            arm.setPower(0);
            grab.setPosition(0);
            sleep(400);
            grab.setPosition(1);
            arm.setPower(0.5);
            sleep(600);
            leftDf.setPower(.2);
            leftDb.setPower(.2);
            rightDb.setPower(.2);
            rightDf.setPower(.2);
            launcher2.setPower(-1);
            sleep(1400);
            //left position
            leftDf.setPower(.7);
            leftDb.setPower(-.7);
            rightDb.setPower(.7);
            rightDf.setPower(-.7);
            sleep(1150);
            leftDf.setPower(0);
            leftDb.setPower(0);
            rightDb.setPower(0);
            rightDf.setPower(0);
            sleep(1000);
            forwardamt(.15);
            sleep(1000);
            launcher2.setPower(-.75);
            launcher1.setPower(-.25);
            sleep(1000);
            //middle position
            leftDf.setPower(-.33);
            leftDb.setPower(.33);
            rightDb.setPower(-.33);
            rightDf.setPower(.33);
            sleep(1000);
            leftDf.setPower(0);
            leftDb.setPower(0);
            rightDb.setPower(0);
            rightDf.setPower(0);
            sleep(1400);
            launcher2.setPower(-.85);
            launcher1.setPower(-.2);
            sleep(1000);
            launcher1.setPower(0);
            //RIGHT position
            leftDf.setPower(-.2);
            leftDb.setPower(.2);
            rightDb.setPower(-.2);
            rightDf.setPower(.2);
            sleep(1000);
            leftDf.setPower(0);
            leftDb.setPower(0);
            rightDb.setPower(0);
            rightDf.setPower(0);
            sleep(1400);
            launcher2.setPower(-.85);
            launcher1.setPower(-.5);
            sleep(1000);
            forwardamt(.3);
            sleep(1000);
            
            //leftDf.setPower(-0.3);
            //leftDb.setPower(-0.3);
            //rightDb.setPower(-0.3);
            //rightDf.setPower(-0.3);
            //sleep(1000);
            //move backwards to get the second wobble goal
            
            
            

            
            
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "leftDf (%.2f),leftDb (%.2f),rightDf (%.2f) rightDb (%.2f)", leftDf,leftDb,rightDf,rightDb);
            //telemetry.update();
        
    }
    public void forwardamt(double amt)
    {
        leftDf.setPower(-amt);
        leftDb.setPower(-amt);
        rightDb.setPower(-amt);
        rightDf.setPower(-amt);
    }
    public void left(double amt)
    {
        leftDf.setPower(amt);
        leftDb.setPower(-amt);
        rightDb.setPower(amt);
        rightDf.setPower(-amt);
        sleep(1000);
    }
    public void forward(int amt)
    {
        rightDf.setTargetPosition(rightDf.getCurrentPosition()+amt);
        rightDb.setTargetPosition(rightDb.getCurrentPosition()+amt);
        leftDf.setTargetPosition(leftDf.getCurrentPosition()+amt);
        leftDb.setTargetPosition(leftDb.getCurrentPosition()+amt);
        
        rightDf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(rightDf.isBusy()&&rightDb.isBusy()&&leftDf.isBusy()&&leftDb.isBusy()&&opModeIsActive())
        {
            rightDf.setPower(.6);
            rightDb.setPower(.6);
            leftDf.setPower(.6);
            leftDb.setPower(.6);
            telemetry.addData("rightDf pos", rightDf.getCurrentPosition());
            telemetry.update();
            
        }
        rightDf.setPower(0);
        rightDb.setPower(0);
        leftDf.setPower(0);
        leftDb.setPower(0);
        
        rightDf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
    }
}


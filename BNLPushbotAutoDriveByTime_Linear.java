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

import android.app.backup.FullBackupDataOutput;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot02;

import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot02.FULL_SERVO;
import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot02.START_SERVO;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Auto Drive By Time", group="BNL")

//@Disabled
public class BNLPushbotAutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot02       robot   = new HardwarePushbot02();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     RightDrvTurn_Unhook  = -.4; //constant is Neg value for reverse
    static final double     LeftDrvTurn_Unhook  = .45; //constant is Pos value for forward
    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     FULL_SPEED    = 1;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 00 initialize Wrist & mineral arm
        robot.mineralArm.setPower(0);
        robot.rightwrist.setPosition(0);
        robot.leftwrist.setPosition(1);

        // Step 0:  Drive Descend from Lander 2 seconds
        robot.landerElevator.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.2)) {
            telemetry.addData("Path", "Leg 0: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Step 0.1 Pause after decending to let robot to settle first before moving
        while (opModeIsActive() && (runtime.seconds() < 4.5)) {
            telemetry.addData("Path", "Leg 1.1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 1:  1.1 straight into lander, 1.2 Turn right to disconnect HOOK
        // Straight first
        robot.leftDrive.setPower(LeftDrvTurn_Unhook);
        robot.rightDrive.setPower(LeftDrvTurn_Unhook);
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 1.1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.leftDrive.setPower(.6); //constant above is POS+ value for forward
        robot.rightDrive.setPower(-.6); //constant above is NEG- value for reverse
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.7)) {
            telemetry.addData("Path", "Leg 1.2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Drive Forwards for .8 Seconds
        robot.leftDrive.setPower(FULL_SPEED);
        robot.rightDrive.setPower(FULL_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < .8)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Spin right for 1.0 seconds
        robot.leftDrive.setPower(TURN_SPEED);
        robot.rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

     /*   Remove Last 2 steps
        // Step 4:  Drive Backwards for 1 Second
        robot.leftDrive.setPower(-FORWARD_SPEED);
        robot.rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 5:  Stop and close the claw.
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftClaw.setPosition(1.0);
        robot.rightClaw.setPosition(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
     */

    }
}

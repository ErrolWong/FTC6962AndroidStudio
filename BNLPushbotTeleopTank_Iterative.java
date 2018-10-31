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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot02;

import static java.lang.Math.tanh;
import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot02.FULL_SERVO;
import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot02.START_SERVO;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware02 class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="BNLPushbot: Teleop Tank", group="BNL")
//@Disabled
public class BNLPushbotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot02 robot     = new HardwarePushbot02();    // use the class created to define a Pushbot's hardware
                                                              // could also use HardwarePushbotMatrix class.
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.mineralpup.setPower(0);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
         //robot.mineralpup.setPower(0);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double mineralpower;

     // Run wheels in tank mode(note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

     // Use gamepad1 trigger buttons to move the landerElevator Gamepad 1 Lefttrigger(Y) and down (A)
       if (gamepad1.left_trigger > .9) robot.landerElevator.setPower(-.45);
        // NEG value Raise arm UP
        else if (gamepad1.right_trigger > .9) robot.landerElevator.setPower(1.0);
          // POS value Lower arm DOWN .45 to .7 more 1.0 FULL power
          else robot.landerElevator.setPower(0.0);
           // no imput STOP movement

     // USE Gamepad2.Right bumper collect .Left bumper release
        if (gamepad2.dpad_up) {
            robot.rightwrist.setPosition(0);
            robot.leftwrist.setPosition(1);
            }
             else if (gamepad2.dpad_down) {
              robot.rightwrist.setPosition(1);
              robot.leftwrist.setPosition(0);
              }

     // Use gamepad2 left stick to move the (MINERAL) RIGHT arm up UP and DOWN
        mineralpower = -gamepad2.left_stick_y;
        robot.mineralArm.setPower(mineralpower*.65);
        //else robot.mineralArm.setPower(0.0);

     // USE Gamepad2 A Intake and B Dispense 360 deg servo X to stop
        if (gamepad2.a) robot.mineralpup.setPower(.99);
          else if (gamepad2.b) robot.mineralpup.setPower(-.99);
             if (gamepad2.x) robot.mineralpup.setPower(.0);

     // Send telemetry message to signify robot running;
       // telemetry.addData("MineralArm",  "et = %.2f", robot.mineralArm.getCurrentPosition());
        // telemetry.addData("teamHand",  "%.2f", robot.teamHand.getPosition());
       // telemetry.addData("rightwrist", "%.2f", robot.rightwrist.getPosition());
       // telemetry.addData("leftwrist", "%.2f", robot.leftwrist.getPosition());
     // telemetry.addData("mineralpup", "%.2f", robot.mineralpup.getPower());
    }

     // Code to run ONCE after the driver hits STOP
     //
    @Override
    public void stop() {
    }
}

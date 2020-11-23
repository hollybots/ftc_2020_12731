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

package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOpModesBase;
import org.firstinspires.ftc.teamcode.Components.WheelPower;

/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Test Mecanum", group="3")
@Disabled
public class TestMecanumWheelsPropulsion extends TeleOpModesBase
{
    static final double     AUTONOMOUS_SPEED            = 0.6;
    static final double     K                           = 0.2;
    private double          theta                       = 0;   // gyro angle.  For field centric autonomous mode we will use this to orient the robot

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing the base...");
        super.init();
        telemetry.update();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("Status", "Make sure motors are free to move");
        telemetry.update();
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
        /*
        Read gamepad value
         */
        // ... or for two 2-axis joysticks do this (Halo):
        // push joystick1 forward to go forward
        // push joystick1 to the right to strafe right
        // push joystick2 to the right to rotate clockwise
        double forward      = -gamepad1.right_stick_y;
        double right        = gamepad1.right_stick_x;
        double clockwise    = gamepad1.left_stick_x;


        // Now add a tuning constant K for the rotate axis sensitivity.
        // Start with K=0, and increase it very slowly (do not exceed K=1)
        // to find the right value after you’ve got fwd/rev and strafe working:
        clockwise = K*clockwise;

        // if "theta" is measured CLOCKWISE from the zero reference:
        double temp = forward* Math.cos(theta) + right* Math.sin(theta);
        right = -forward* Math.sin(theta) + right* Math.cos(theta);
        forward = temp;

// if "theta" is measured COUNTER-CLOCKWISE from the zero reference:
//        temp = forward*Math.cos(theta) - right*Math.sin(theta);
//        right = forward*Math.sin(theta) + right*Math.cos(theta);
//        forward = temp;


        // Now apply the inverse kinematic tranformation
        // to convert your vehicle motion command
        // to 4 wheel speed commands:
        WheelPower wheels = null;
        wheels.front_left = forward + clockwise + right;
        wheels.front_right = forward - clockwise - right;
        wheels.rear_left = forward + clockwise - right;
        wheels.rear_right = forward - clockwise + right;
        wheels.normalize();

        // Send calculated power to wheels
        botBase.driveTrain.setPower(wheels);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), rear left (%.2f), rear right (%.2f)", wheels.front_left, wheels.front_right, wheels.rear_left, wheels.rear_right);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }
}

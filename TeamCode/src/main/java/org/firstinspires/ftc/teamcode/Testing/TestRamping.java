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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOpModesBase;

/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Testing Ramping", group="1")
//@Disabled
public class TestRamping extends TeleOpModesBase
{

    private class WheelPower {
        double front_left;
        double front_right;
        double rear_left;
        double rear_right;
    };

    private double[] ramp = {
            -1,
            -0.86,
            -0.73,
            -0.61,
            -0.51,
            -0.42,
            -0.34,
            -0.27,
            -0.22,
            -0.17,
            -0.13,
            -0.09,
            -0.06,
            -0.04,
            -0.03,
            -0.02,
            -0.01,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0.01,
            0.02,
            0.03,
            0.04,
            0.06,
            0.09,
            0.13,
            0.17,
            0.22,
            0.27,
            0.34,
            0.42,
            0.51,
            0.61,
            0.73,
            0.86,
            1
    };

    static final int CONTROL_FORWARD = 0;
    static final int CONTROL_RIGHT = 1;


    // This limits the power change to an 0.1 increment every 200ms 0,00005 power/s^2
    static final double  DELTA_T                        = 100; // in ms ) {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    static final double     K                           = 0.6;
    private double          theta                       = 0;   // gyro angle.  For field centric autonomous mode we will use this to orient the robot

    double lastTimeWeCheckedSpeed                       = 0.0;
    int currentRampNumberForward                        = 20;
    int currentRampNumberRight                          = 20;

    double previousPowerForward                             = 0;
    double previousPowerRight                               = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        // Send telemetry message to indicate successful Encoder reset
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

        boolean moving = false;


        double forward                  = -gamepad1.right_stick_y;
        double right                    = gamepad1.right_stick_x;
        WheelPower wheels               = null;
        double clockwise                = gamepad1.left_stick_x;

        double now                          = runtime.milliseconds();
        double deltaT                       = now - lastTimeWeCheckedSpeed;

        if (deltaT > DELTA_T ) {
//            forward                     = rampUp(CONTROL_FORWARD, forward);
//            right                       = rampUp(CONTROL_RIGHT, right);
            wheels                      = calcWheelPower(K, clockwise, forward, right);
            lastTimeWeCheckedSpeed      = now;
        }


        /**
         * OUTPUT PROPULSION
         */
        if (deltaT > DELTA_T && wheels != null) {
            dbugThis(String.format("%.1f,%.1f,%.02f,%.02f,%.02f,%.02f", forward, right, wheels.front_left,wheels.front_right,wheels.rear_left,wheels.rear_right));
        }

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private WheelPower calcWheelPower(double K, double clockwise, double forward, double right) {

        WheelPower wheels = new WheelPower();
        clockwise = K * clockwise;

        // if "theta" is measured CLOCKWISE from the zero reference:
        double temp = forward * Math.cos(theta) + right * Math.sin(theta);
        right = -forward * Math.sin(theta) + right * Math.cos(theta);
        forward = temp;

        wheels.front_left = forward + clockwise + right;
        wheels.front_right = forward - clockwise - right;
        wheels.rear_left = forward + clockwise - right;
        wheels.rear_right = forward - clockwise + right;

        double calculatedPropulsionCommand = Math.abs(wheels.front_left);
        if (Math.abs(wheels.front_right) > calculatedPropulsionCommand) {
            calculatedPropulsionCommand = Math.abs(wheels.front_right);
        }
        if (Math.abs(wheels.rear_left) > calculatedPropulsionCommand) {
            calculatedPropulsionCommand = Math.abs(wheels.rear_left);
        }
        if (Math.abs(wheels.rear_right) > calculatedPropulsionCommand) {
            calculatedPropulsionCommand = Math.abs(wheels.rear_right);
        }

        if (calculatedPropulsionCommand > 1.0) {
            wheels.front_left /= calculatedPropulsionCommand;
            wheels.front_right /= calculatedPropulsionCommand;
            wheels.rear_left /= calculatedPropulsionCommand;
            wheels.rear_right /= calculatedPropulsionCommand;
        }

        return wheels;
    }



    private double rampUp(int which, double setPoint) {

        if (which == CONTROL_FORWARD) {

            if ( setPoint < previousPowerForward) {
                currentRampNumberForward = Math.min(Math.max(0, currentRampNumberForward - 1), 40);
            }
            else if (setPoint > previousPowerForward) {
                currentRampNumberForward = Math.min(Math.max(0, currentRampNumberForward + 1), 40);
            }

            previousPowerForward = ramp[currentRampNumberForward];
            return ramp[currentRampNumberForward];
        }
        else if (which == CONTROL_RIGHT) {

            if ( setPoint < previousPowerRight) {
                currentRampNumberRight = Math.min(Math.max(0, currentRampNumberRight - 1), 40);
            }
            else if (setPoint > previousPowerRight) {
                currentRampNumberRight = Math.min(Math.max(0, currentRampNumberRight + 1), 40);
            }

            previousPowerRight = ramp[currentRampNumberForward];
            return ramp[currentRampNumberForward];
        }

        return 0;
    }
}

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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.WheelPower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Components.LedPatterns;


/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="TeleOp Main", group="1")
@Disabled
public class TeleOpMode_12731 extends TeleOpModesBase
{

    // This limits the power change to an 0.1 increment every 200ms 0,00005 power/s^2
    static final double  DELTA_T                        = 200; // in ms ) {
    static final double  MAX_CHANGE_IN_POWER_IN_DELTA_T = 0.1;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // State variables
    boolean isClamping                                  = false;
    boolean waitForClampingButtonRelease                = false;


    boolean isFastSpeedMode                             = false;
    boolean waitForSpeedButtonRelease                   = false;


    int resetState                                      = 0;
    int readyState                                      = 0;
    int dodgeState                                      = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing the base...");
        telemetry.update();

        // Init Botbase and Bottop
        super.init();

        botBase.setBling(LedPatterns.LED_OFF);

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
        botBase.setBling(LedPatterns.LED_TEAM_COLORS4);
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        /**
         * INPUT GAMEPAD
         */
        // ... or for two 2-axis joysticks do this (Halo):
        // push joystick1 forward to go forward
        // push joystick1 to the right to strafe right
        // push joystick2 to the right to rotate clockwise

        double forward                  = -gamepad1.left_stick_y;
        double right                    = gamepad1.left_stick_x;
        double clockwise                = gamepad1.right_stick_x;

        /**
         * Use this to command DC Motor Power
         */
        WheelPower wheels               = null;

        double now                      = runtime.milliseconds();
        double arm                      = -gamepad2.right_stick_y;
        double linearMotion             = -gamepad2.left_stick_y;

        boolean isPressedClampingButton     = gamepad2.right_stick_button;
        boolean toggledClamp                = false;

        boolean isPressedSpeedButton        = gamepad1.right_stick_button;
        boolean toggledSpeed                = false;

        /**
         * LED BLINK MODES TRIGGER
         */
        boolean blinkinOff                  =  gamepad1.dpad_up;
        boolean blinkin1                    =  gamepad1.dpad_down;
        boolean blinkin2                    =  gamepad1.dpad_left;
        boolean blinkin3                    =  gamepad1.dpad_right;

        boolean onHeading0                  =  gamepad1.y;
        boolean onHeading90                 =  gamepad1.x;

        /**
         * MOVEMENT COMBINATIONS
         */
        boolean reset                       =  gamepad2.x;
        boolean ready                       =  gamepad2.y;
        boolean dodge                       =  gamepad2.back;


        /**
         * GET UPDATES ON POSITION
         */
        if (botBase.hasOdometry()) {
            botBase.odometer.globalCoordinatePositionUpdate();
        }

        /**
         * GET UPDATES ON PROPULSTION COMMANDS
         */
        wheels                              =  calcWheelPower(clockwise, forward, right);

        /**
         * GET UPDATES ON LIMIT SWITCHES
         */
        botTop.checkAllLimitSwitches();

        /**
         * SEQUENCE MONITORING - When you have to initiate and end a sequence
         */
        if (resetState > 0) {
            maybeEndResetSequence();
        }

        // If no other sequences are playing, we can start a new one
        if ( noSequenceIsCurrentlyExecuting() ) {

            // If we ask for reset, we start the sequence here
            if ( reset ) {
                startResetSequence();
            }
        }

        // If in one of those pre-set sequence, ignore all the BotTop commands
        if ( noSequenceIsCurrentlyExecuting() ) {

            /**
             * TOGGLE CLAMP MECHANISM - Servo positions
             */
            if (isPressedClampingButton) {
                waitForClampingButtonRelease = true;
            } else if (waitForClampingButtonRelease) {
                waitForClampingButtonRelease = false;
                toggledClamp = true;
            }

            if (toggledClamp && isClamping) {
                isClamping = false;
            } else if (toggledClamp && !isClamping) {
                isClamping = true;
            }
            if (isClamping) {
                // Go to position 1
            } else {
                // Go to position 2
            }

            /**
             * TOGGLE SPEED MECHANISM - Speed toggle
             */
            if (isPressedSpeedButton) {
                waitForSpeedButtonRelease = true;
            } else if (waitForSpeedButtonRelease) {
                waitForSpeedButtonRelease = false;
                toggledSpeed = true;
            }

            if (toggledSpeed && isFastSpeedMode) {
                isFastSpeedMode = false;
            } else if (toggledSpeed && !isFastSpeedMode) {
                isFastSpeedMode = true;
            }
            if (!isFastSpeedMode) {
                wheels.front_left /= 2;
                wheels.front_right /= 2;
                wheels.rear_left /= 2;
                wheels.rear_right /= 2;
            }
        }






        /**
         * OUTPUT PROPULSION
         */
        // Send calculated power to wheels
        botBase.getFrontLeftDrive().setPower(wheels.front_left);
        botBase.getFrontRightDrive().setPower(wheels.front_right);
        botBase.getRearLeftDrive().setPower(wheels.rear_left);
        botBase.getRearRightDrive().setPower(wheels.rear_right);




        /**
         * OUTPUT BLING
         */
        if (blinkinOff) {
            botBase.setBling(LedPatterns.LED_OFF);
        }
        else if (blinkin1) {
            botBase.setBling(LedPatterns.LED_TEAM_COLORS1);
        }
        else if (blinkin2) {
            botBase.setBling(LedPatterns.LED_TEAM_COLORS4);
        }
        else if (blinkin3) {
            botBase.setBling(LedPatterns.LED_TEAM_COLORS3);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addLine("Motor Speed")
                .addData("FR", String.format("%.3f", wheels.front_right))
                .addData("FL", String.format("%.3f", wheels.front_left))
                .addData("RR", String.format("%.3f", wheels.rear_right))
                .addData("RL", String.format("%.3f", wheels.rear_left));

        telemetry.addLine("Odometer")
                .addData("X", String.format("%.3f", botBase.odometer.getCurrentXPos()))
                .addData("Y", String.format("%.3f", botBase.odometer.getCurrentYPos()));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
        botBase.setBling(LedPatterns.LED_OFF);
    }


    /**
     * Typical sequence states
     * @return
     */
    private boolean noSequenceIsCurrentlyExecuting() {

        return resetState == 0 && readyState == 0 && dodgeState == 0;
    }

    /**
     * Starts a Movement combination
     */
    private void startResetSequence() {
        // Start the movement - make sure there are no loops in these methods

        // Set the state variable
        resetState = 1;
    }


    private void maybeEndResetSequence(){

        // Check for an End sequence Request, if so, set the flag
        if ( /* botTop.isCoilLimitDown() && */ resetState == 1 ) {
            resetState = 2;
            return;
        }

        // Check for a Completion State.  If so, set the flag
        if ( /* botTop.isSwingLimitDown() && */ resetState == 2 ) {
            resetState = 0;
        }
    }
}

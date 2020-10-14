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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.WheelPower;


/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="TeleOp Rename please", group="1")
@Disabled
public class TeleOpMode_sofia extends TeleOpModesBase
{

    private void justWait(int milliseconds){

        double currTime = getRuntime();
        double waitUntil = currTime + (double)(milliseconds/1000);
        while (getRuntime() < waitUntil){
        }

    }

    static final double  LED_OFF                        = 0.7745;   // off
    static final double  LED_TEAM_COLORS1               = 0.6545;  // Sinelon, Color 1 and 2
    static final double  LED_TEAM_COLORS2               = 0.6295;  // End to End Blend
    static final double  LED_TEAM_COLORS3               = 0.6045;  // Sparkle, Color 1 on Color 2
    static final double  LED_TEAM_COLORS4               = 0.6195;  // Beats per Minute, Color 1 and 2

    static final int INITIATE_COLLECTING_STATE = 1;
    static final int LOAD_STATE = 2;
    static final int COLLECTING_STATE = 3;
    static final int LAUNCHING_STATE = 4;
    private int currentState = INITIATE_COLLECTING_STATE;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // State variables
    boolean isFastSpeedMode                             = false;
    boolean waitForSpeedButtonRelease                   = false;

    boolean wasPressedLaunchingButton                   = false;
    boolean wasPressedLoadingButton                     = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing the base...");
        telemetry.update();

        // Init Botbase and Bottop
        super.init();

        // Turn off the LEDs
        botBase.setBling(LED_OFF);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        // Send telemetry message
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        // Resets the OpMode timer
        runtime.reset();
        // Fire up the LEDs
        botBase.setBling(LED_TEAM_COLORS4);
    }


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /**
         * Use this to command DC Motor Power
         */
        WheelPower wheels               = null;

        /*******************************
         * 1) GAMEPAD INPUT SECTION
         *
         */

        /**
         * Input drivetrain commands
         */
        // push joystick1 forward to go forward
        // push joystick1 to the right to strafe right
        // push joystick2 to the right to rotate clockwise
        // I'm remapping the joystick2 right stick so we can use it for the claw thingy
        // couldn't find a way to deactivate "clockwise" without causing an error so i mapped it to gamepad 2

        double forward                  = -gamepad1.left_stick_y;
        double right                    = gamepad1.left_stick_x;
        double clockwise                = gamepad2.right_stick_x;
        double clawextend               = gamepad1.right_stick_y;
        double clawretract              = -gamepad1.right_stick_y;


        /**
         * Input speed toggle
         */
        float isPressedForwardButton        = gamepad1.right_trigger;
        float isPressedBackwardButton        = gamepad1.left_trigger;
        boolean toggledSpeed                = false;

        /* Other gamepad inputs
         */
        // To initiate loaading state
        boolean isPressedLoadingButton     = gamepad1.x;  // a and b are already used by the Robot Controller to select the gamepad
        // To initiate collecting state
        boolean isPressedResetButton       = gamepad1.y;
        // To launch rings
        boolean isPressedLaunchingButton   = gamepad1.right_bumper;

        /**
         * Input LED trigger
         */
        boolean blinkinOff                  =  gamepad1.dpad_up;
        boolean blinkin1                    =  gamepad1.dpad_down;
        boolean blinkin2                    =  gamepad1.dpad_left;
        boolean blinkin3                    =  gamepad1.dpad_right;


        /*******************************
         * 2) SUBSYSTEMS UPDATES
         *
         */

        //pick up the wobble goal
        if (clawextend >= 0.1) {
            botTop.clawMotorOn(0.3);
        }
        else if (clawretract >= 0.1) {
            botTop.clawMotorOn(-0.3);
        }
        else if (clawextend == 0.0 && clawretract == 0.0){
            botTop.clawMotorOff();
        }

        // I might've messed up a bunch of stuff but there's my progress for today

        if (currentState == INITIATE_COLLECTING_STATE) {
            botTop.lowerMagazine();
            botTop.retractArm();
            botTop.intakeMotorOn(0.5);
            botTop.launchMotorOff();

            if (isPressedLoadingButton && !wasPressedLoadingButton) {
                currentState = LOAD_STATE;
            }
        }

        else if (currentState == LOAD_STATE) {
            botTop.intakeMotorOff(0.0);
            botTop.launchMotorOn();
            botTop.liftMagazine();
            currentState = LAUNCHING_STATE;
        }
        else if (currentState == LAUNCHING_STATE) {
            // Here we are trying to send the servo command ony only once, so we detect one the change on the button state

            // The button is being pressed
            if (isPressedLaunchingButton && !wasPressedLaunchingButton) {
                botTop.extendArm();
                justWait(1000);
                wasPressedLaunchingButton = true;
            }
            // look coach i fixed it :))))))))
            // the button is being released
            else if (!isPressedLaunchingButton && wasPressedLaunchingButton) {
                botTop.retractArm();
                wasPressedLaunchingButton = false;
            }

            else if (isPressedResetButton){
                currentState = INITIATE_COLLECTING_STATE;
            }
        }

        /**
         * Updates from Odometer
         */
        botBase.odometer.globalCoordinatePositionUpdate();

        /**
         * Updates from  critical limit switches
         */
        botTop.checkAllLimitSwitches();


        /*******************************
         * 3) CALCULATIONS
         *
         */

        /**
         * Power calculation for drivetrain
         */

        // making myself a note to change these

        wheels =  calcWheelPower(clockwise, forward, right);
        if (isPressedForwardButton >= 0.2) {
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


        /*******************************
         * 4) OUTPUT SECTION
         *
         */


        /**
         * Output drivetrain
         */
        botBase.getFrontLeftDrive().setPower(wheels.front_left);
        botBase.getFrontRightDrive().setPower(wheels.front_right);
        botBase.getRearLeftDrive().setPower(wheels.rear_left);
        botBase.getRearRightDrive().setPower(wheels.rear_right);

        dbugThis(String.format("%.02f,%.02f,%.02f,%.02f", wheels.front_left,wheels.front_right,wheels.rear_left,wheels.rear_right));


        /**
         * Output LED
         */
        if (blinkinOff) {
            botBase.setBling(LED_OFF);
        }
        else if (blinkin1) {
            botBase.setBling(LED_TEAM_COLORS1);
        }
        else if (blinkin2) {
            botBase.setBling(LED_TEAM_COLORS4);
        }
        else if (blinkin3) {
            botBase.setBling(LED_TEAM_COLORS3);
        }

        /**
         * Output Telemetry
         */
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
        botBase.setBling(LED_OFF);
    }
}

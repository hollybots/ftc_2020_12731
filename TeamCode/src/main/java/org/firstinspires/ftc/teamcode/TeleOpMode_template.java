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
public class TeleOpMode_template extends TeleOpModesBase
{

    static final double  LED_OFF                        = 0.7745;   // off
    static final double  LED_TEAM_COLORS1               = 0.6545;  // Sinelon, Color 1 and 2
    static final double  LED_TEAM_COLORS2               = 0.6295;  // End to End Blend
    static final double  LED_TEAM_COLORS3               = 0.6045;  // Sparkle, Color 1 on Color 2
    static final double  LED_TEAM_COLORS4               = 0.6195;  // Beats per Minute, Color 1 and 2

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // State variables
    boolean isFastSpeedMode                             = false;
    boolean waitForSpeedButtonRelease                   = false;

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

        double forward                  = -gamepad1.left_stick_y;
        double right                    = gamepad1.left_stick_x;
        double clockwise                = gamepad1.right_stick_x;


        /**
         * Input speed toggle
         */
        boolean isPressedSpeedButton        = gamepad1.right_stick_button;
        boolean toggledSpeed                = false;

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
        wheels =  calcWheelPower(clockwise, forward, right);
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

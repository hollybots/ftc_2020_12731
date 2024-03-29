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
import org.firstinspires.ftc.teamcode.Components.LedPatterns;

import org.firstinspires.ftc.teamcode.Components.WheelPower;
import org.firstinspires.ftc.teamcode.Components.AdWidget;


/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="TeleOp sofia lol", group="1")
//@Disabled
public class TeleOpMode_sofia extends TeleOpModesBase
{

    static final double CLAW_POWER                      = 0.4;
//    static final double LAUNCH_VELOCITY                    = 0.75;

    static final double LAUNCH_VELOCITY_POWER_SHOT_BACK    = 1700;
    static final double LAUNCH_VELOCITY_POWER_SHOT_FRONT   = 1550;
    static final double LAUNCH_VELOCITY_TOWER_RING         = 1700;
    static final double INTAKE_POWER                    = 0.9;
    static final int SERVO_TIMEOUT                      = 220;     // ms before the arms retracts.  Should be the interval defined by the servo manufacturer for 60 degrees
    static final int PIVOTING_TIMEOUT                   = 90;
    static final int COLLECTOROUT                       = 0;
    static final int COLLECTORIN                        = 1;

    static final int INITIATE_COLLECTING_STATE          = 1;
    static final int LOAD_STATE                         = 2;
    static final int COLLECTING_STATE                   = 3;
    static final int LAUNCHING_STATE                    = 4;
    static final int REJECTING_STATE                    = 5;
    static final int ENDGAME_STATE                      = 6;
    static final int LOAD_STATE_2                       = 7;
    static final int TURNING_STATE                      = 8;

    static final int POWER_MODE_TOWER                   = 1;
    static final int POWER_MODE_FRONT_SHOT              = 2;
    static final int POWER_MODE_BACK_SHOT               = 3;

    private int currentState = INITIATE_COLLECTING_STATE;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // State variables
    boolean isFastSpeedMode                             = false;
    boolean waitForSpeedButtonRelease                   = false;
    boolean isReverseMode                               = false;

    boolean wasPressedLaunchingButton                   = false;
    boolean wasPressedCollectWobbleGoalButton           = false;
    double servoTimeout                                 = 0.0;
    boolean wasPressedResetButton                       = false;
    boolean wasPressedPivotButton                       = false;
    double pivotingTimeout                              = 0;
    int collectorPosition                            = COLLECTOROUT;

    // Defining launch power
    double launchVelocity                                  = LAUNCH_VELOCITY_TOWER_RING;
    int launchVelocityMode                                 = POWER_MODE_TOWER;

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
        botBase.setBling(LedPatterns.LED_OFF);
//        AdWidget.displayLogo(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Set the collector position to out
        botTop.collectOut();
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

        double forward                  = isReverseMode ? gamepad1.left_stick_y : -gamepad1.left_stick_y;
        double right                    = isReverseMode ? -gamepad1.left_stick_x : gamepad1.left_stick_x;
        double clockwise                = gamepad1.right_stick_x;

        /**
         * Input speed toggle
         */
        boolean isPressedSpeedButton        = gamepad1.right_stick_button;
        boolean toggledSpeed                = false;

        boolean isPressedReverseButton        = gamepad1.left_stick_button;
        boolean toggledReverse                = false;

        /* Other gamepad inputs
         */
        // To initiate loading state
        boolean isPressedLoadingButton          = gamepad1.x;  // a and b are already used by the Robot Controller to select the gamepad
        // To initiate collecting state
        boolean isPressedResetButton            = gamepad1.y;
        // To start end game mode
        boolean isPressedEndGameButton          = gamepad1.a;
        // To start end game mode
        boolean isPressedSecondEndGameButton    = gamepad1.b;
        // To launch rings
        boolean isPressedLaunchingButton        = gamepad1.right_bumper;
        // To Collect wobble goal
        boolean isPressedCollectWobbleGoalButton   = gamepad1.right_bumper;
        // To unhook wobble goal
        boolean isPressedHookWobbleGoalButton = gamepad1.right_bumper;
        // To unhook wobble goal
        boolean isPressedUnhookWobbleGoalButton = gamepad1.left_bumper;
        // To lift wobble goal
        boolean isPressedLiftWobbleGoalButton   = gamepad1.right_trigger > 0.2;
        // To lower wobble goal
        boolean isPressedLowerWobbleGoalButton  = gamepad1.left_trigger > 0.2;
        // To launch power tower ring
        boolean isPressedLaunchPowerTowerRing  = gamepad1.dpad_right;
        // To launch power tower ring
        boolean isPressedLaunchPowerPowerShot  = gamepad1.dpad_left;
        // To launch power tower ring
        boolean isPressedLaunchPowerBackPowerShot  = gamepad1.dpad_up;
        // To turn auto
        boolean isPressedPivotButton  = gamepad1.left_trigger > 0.2;


        /*******************************
         * 2) SUBSYSTEMS UPDATES
         *
         */

        if (currentState == INITIATE_COLLECTING_STATE) {
            isReverseMode = false;
            botBase.setBling(LedPatterns.LED_SOLID_COLOR_ORANGE);
            botTop.lowerMagazine();
            botTop.retractArm();
            botTop.intakeMotorOn(INTAKE_POWER);
            botTop.launchMotorOff();
            currentState = COLLECTING_STATE;
        }

        else if (currentState == COLLECTING_STATE) {
            isReverseMode = false;
            if (isPressedLoadingButton) {
                currentState = LOAD_STATE;
            }
            if (isPressedResetButton) {
                wasPressedResetButton = true;
            }
            else if (wasPressedResetButton) {
                wasPressedResetButton = false;
                currentState = REJECTING_STATE;
            }
            if (isPressedEndGameButton){
                currentState = ENDGAME_STATE;
            }

            else if (isPressedLaunchingButton && !wasPressedLaunchingButton) {
                botTop.extendArm();
                wasPressedLaunchingButton = true;
                servoTimeout = runtime.milliseconds() + SERVO_TIMEOUT;
            }
            // the button is being released after servoTimeout
            else if (runtime.milliseconds() > servoTimeout && wasPressedLaunchingButton) {
                botTop.retractArm();
                wasPressedLaunchingButton = false;
            }
        }

        else if (currentState == REJECTING_STATE) {
            isReverseMode = false;
            botBase.setBling(LedPatterns.LED_SOLID_COLOR_RED_ORANGE);
            botTop.intakeMotorOn(-INTAKE_POWER/2);
            if (isPressedResetButton) {
                wasPressedResetButton = true;
            }
            else if (wasPressedResetButton) {
                wasPressedResetButton = false;
                currentState = INITIATE_COLLECTING_STATE;
            }
        }

        else if (currentState == LOAD_STATE_2) {
            botTop.launchMotorOn(LAUNCH_VELOCITY_POWER_SHOT_BACK);
            // same as following but with a slower lau
            botBase.setBling(LedPatterns.LED_SOLID_COLOR_RED);
            isReverseMode = false;
            botTop.intakeMotorOff();
            botTop.liftMagazine();
            currentState = LAUNCHING_STATE;
        }

        else if (currentState == LOAD_STATE) {
            botBase.setBling(LedPatterns.LED_SOLID_COLOR_BLUE);
            isReverseMode = false;
            botTop.intakeMotorOff();
            botTop.launchMotorOn(launchVelocity);
            botTop.liftMagazine();
            currentState = LAUNCHING_STATE;
        }

        else if (currentState == LAUNCHING_STATE) {
            isReverseMode = false;
            botTop.launchMotorOn(launchVelocity);
//            botBase.setBling(LedPatterns.LED_SOLID_COLOR_BLUE);
            // The launch button is being pressed
            if (isPressedLaunchingButton && !wasPressedLaunchingButton) {
                botTop.extendArm();
                wasPressedLaunchingButton = true;
                servoTimeout = runtime.milliseconds() + SERVO_TIMEOUT;
            }
            // the button is being released after servoTimeout
            else if (runtime.milliseconds() > servoTimeout && wasPressedLaunchingButton) {
                botTop.retractArm();
                wasPressedLaunchingButton = false;
            }
            if (isPressedResetButton) {
                wasPressedResetButton = true;
            }
            else if (wasPressedResetButton) {
                wasPressedResetButton = false;
                currentState = INITIATE_COLLECTING_STATE;
            }
            if (isPressedEndGameButton){
                currentState = ENDGAME_STATE;
            }
            if (isPressedPivotButton) {
                wasPressedPivotButton = true;
            }
            else if (wasPressedPivotButton) {
                clockwise = 0.5;
                wasPressedPivotButton = false;
                pivotingTimeout = runtime.milliseconds() + PIVOTING_TIMEOUT;
                currentState = TURNING_STATE;
            }
        }

        else if (currentState == TURNING_STATE) {
            if (runtime.milliseconds() > pivotingTimeout) {
                clockwise = 0;
                currentState = LAUNCHING_STATE;
            } else  {
                clockwise = 0.5;
            }
        }

        else if (currentState == ENDGAME_STATE) {
            isReverseMode = true;
            botBase.setBling(LedPatterns.LED_SOLID_COLOR_GREEN);
            botTop.intakeMotorOff();
            botTop.launchMotorOff();
            /**
             * Control wobble goal collector
             */
            if (isPressedCollectWobbleGoalButton) {
                wasPressedCollectWobbleGoalButton = true;
            }
            else if (wasPressedCollectWobbleGoalButton) {
                if (collectorPosition == COLLECTOROUT) {
                    botTop.collectIn();
                    collectorPosition = COLLECTORIN;
                }
                else if (collectorPosition == COLLECTORIN) {
                    botTop.collectOut();
                    collectorPosition = COLLECTOROUT;
                }
                wasPressedCollectWobbleGoalButton = false;
            }

//            /**
//             * Enable Wobble Goal hook through bumpers
//             */
//            if (isPressedHookWobbleGoalButton) {
//                botTop.wobbleGoalHookMotorOn(0.5);
//            }
//            else if (isPressedUnhookWobbleGoalButton) {
//                botTop.wobbleGoalHookMotorOn(-0.5);
//            }
//            if (!isPressedHookWobbleGoalButton && !isPressedUnhookWobbleGoalButton) {
//                botTop.wobbleGoalHookMotorOff();
//            }
            /**
             * Enable Wobble Goal Lift through trigger
             */
            if (isPressedLiftWobbleGoalButton) {
                botTop.clawMotorOn(-CLAW_POWER);
            }
            else if (isPressedLowerWobbleGoalButton) {
                botTop.clawMotorOn(CLAW_POWER);
            }
            if (!isPressedLiftWobbleGoalButton && !isPressedLowerWobbleGoalButton) {
                botTop.clawMotorOff();
            }

            if (isPressedResetButton) {
                wasPressedResetButton = true;
            }
            else if (wasPressedResetButton) {
                wasPressedResetButton = false;
                currentState = INITIATE_COLLECTING_STATE;
            }

            if (isPressedLoadingButton) {
                currentState = LOAD_STATE;
            }

            if (isPressedSecondEndGameButton) {
                currentState = LOAD_STATE_2;
            }
        }

//        /**
//         * Updates from Odometer - WE DO NOT USE ODOMETRY IN TELE OP. LET'S SAVE SOME TIME
//         */
//        if (botBase.hasOdometry()) {
//            botBase.odometer.globalCoordinatePositionUpdate();
//        }
//
//        /**
//         * Updates from  critical limit switches
//         */
//        botTop.checkAllLimitSwitches();


        /*******************************
         * 3) CALCULATIONS
         *
         */


        if (isPressedLaunchPowerTowerRing){
            launchVelocity = LAUNCH_VELOCITY_TOWER_RING;
            launchVelocityMode = POWER_MODE_TOWER;
        }
        else if (isPressedLaunchPowerPowerShot){
            launchVelocity = LAUNCH_VELOCITY_POWER_SHOT_FRONT;
            launchVelocityMode = POWER_MODE_FRONT_SHOT;
        }
        else if (isPressedLaunchPowerBackPowerShot){
            launchVelocity = LAUNCH_VELOCITY_POWER_SHOT_BACK;
            launchVelocityMode = POWER_MODE_BACK_SHOT;
        }

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
         * Output Telemetry
         */
        telemetry.addLine("Launcher")
        .addData("velocity", botTop.getLaunchMotor().getVelocity())
        .addData("target", launchVelocity)
        .addData("Mode", (launchVelocityMode == POWER_MODE_TOWER) ? "Tower" : (launchVelocityMode == POWER_MODE_BACK_SHOT) ? "Back Power" : (launchVelocityMode == POWER_MODE_FRONT_SHOT)  ? "Front Power" : "Unknown");
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
}

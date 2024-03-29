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

import org.firstinspires.ftc.teamcode.Components.ObjectIdentificationInterface;
import org.firstinspires.ftc.teamcode.Components.TensorFlowObjectIdentification;
import org.firstinspires.ftc.teamcode.Components.VuMarkIdentification;
import org.firstinspires.ftc.teamcode.Components.WheelPower;
import org.firstinspires.ftc.teamcode.OpenCV.RingDetector;
import org.firstinspires.ftc.teamcode.Components.LedPatterns;
import org.firstinspires.ftc.teamcode.OpenCV.RingPosition;
import org.firstinspires.ftc.teamcode.Components.AdWidget;

import java.io.FileWriter;
import java.io.PrintWriter;


/**
 * This OpMode was developed to help position the robot in the pre-start portion of the game.
 * It enables the drivetrain and provides the controls to pre-load the wobble goal onto the
 * robot prior to the game.
 *
 * Additionally, it also initialize the ring detector
 *
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Positioning Tool", group="1")
//@Disabled
public class PositioningTool extends TeleOpModesBase
{

    // Needed for TENSORFLOW object Identification
    protected String TFOD_MODEL_ASSET                           = "Skystone.tflite";
    protected String [] TFOD_MODEL_ASSETS_LABEL                 = {"Stone", "Skystone"};
    protected String TFOD_TARGET_LABEL                          = "Skystone";

    static final int COLLECTORIN                                = 0;
    static final int COLLECTOROUT                               = 1;

    protected String IDENTIFICATION_SYSTEM         = "TSF"; // can be VUFORIA, TSF, or NONE
    protected String CAMERA_SYSTEM                 = "WEBCAM";  // can be PHONE or WEBCAM

    static final double CLAW_POWER                      = 0.4;

    static final double MIN_LAUNCH_VELOCITY                = 1400;
    static final double MAX_LAUNCH_VELOCITY                = 5000;  // Max for this motor 5400

    static final double LAUNCH_VELOCITY_BIG_INCREMENT      = 100;
    static final double LAUNCH_VELOCITY_SMALL_INCREMENT    = 10;

    static final int SERVO_TIMEOUT                      = 220;     // ms before the arms retracts.  Should be the interval defined by the servo manufacturer for 60 degrees
    static final int RPM_CALC_DURATION                  = 500; // in ms
    static final double COUNTS_PER_MOTOR_REV            = 28.0;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // State variables
    boolean isFastSpeedMode                             = false;
    boolean waitForSpeedButtonRelease                   = false;
    boolean isReverseMode                               = true;

    // State variables
    RingPosition ringPosition                           = RingPosition.UNKNOWN;
    protected String ringLabel                          = "";
    double launchVelocity                               = MIN_LAUNCH_VELOCITY;

    boolean wasPressedPowerBigIncrement                 = false;
    boolean wasPressedPowerSmallIncrement               = false;
    boolean wasPressedPowerBigReduction                 = false;
    boolean wasPressedPowerSmallReduction               = false;
    boolean wasPressedCollectWobbleGoalButton           = false;

    boolean wasPressedLaunchingButton                   = false;
    double servoTimeout                                 = 0.0;
    boolean wasPressedResetButton                       = false;
    double rpm                                          = 0;
    double startSampling                                = 0;
    double startPosition                                = 0;

    int collectorPosition                                   = COLLECTOROUT;


    static final int JUST_CHILLING                      = 1;
    static final int LAUNCHING_STATE                    = 2;

    private int currentState                            = JUST_CHILLING;


    /* Object detection sub system
     */
    protected ObjectIdentificationInterface searchableTarget      = null;

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
        AdWidget.displayLogo(hardwareMap);

        TFOD_MODEL_ASSET                            = "UltimateGoal.tflite";
        TFOD_MODEL_ASSETS_LABEL                     = new String[] {"Quad", "Single"};
        TFOD_TARGET_LABEL                           = "";
        IDENTIFICATION_SYSTEM                       = "TSF"; // can be VUFORIA, TSF, or NONE
        CAMERA_SYSTEM                               = "WEBCAM";  // can be PHONE or WEBCAM

        searchableTarget  = new TensorFlowObjectIdentification(
                hardwareMap,
                telemetry,
                TFOD_MODEL_ASSET,
                TFOD_MODEL_ASSETS_LABEL,
                TFOD_TARGET_LABEL,
                CAMERA_SYSTEM,
                true
        );

        // Set the collector position to out
        botTop.collectOut();

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
        botTop.retractArm();
        botTop.liftMagazine();
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
         * Input laucher tuning commands
         */
        boolean isPressedPowerBigIncrement       = gamepad1.dpad_up;
        boolean isPressedPowerSmallIncrement     = gamepad1.dpad_right;
        boolean isPressedPowerBigReduction       = gamepad1.dpad_down;
        boolean isPressedPowerSmallReduction     = gamepad1.dpad_left;


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

        /* Other gamepad inputs
         */
        // To initiate loading state

        boolean isPressedLoadingButton          = gamepad1.x;  // a and b are already used by the Robot Controller to select the gamepad
        // To initiate chilling state
        boolean isPressedResetButton            = gamepad1.y;
        // To Hook wobble goal
        boolean isPressedHookWobbleGoalButton   = gamepad1.right_bumper;
        // To unhook wobble goal
        boolean isPressedUnhookWobbleGoalButton = gamepad1.left_bumper;
        // To Collect wobble goal
        boolean isPressedCollectWobbleGoalButton   = gamepad1.right_bumper;
        // To lift wobble goal
        boolean isPressedLiftWobbleGoalButton   = gamepad1.right_trigger > 0.2;
        // To lower wobble goal
        boolean isPressedLowerWobbleGoalButton  = gamepad1.left_trigger > 0.2;
        // To launch rings
        boolean isPressedLaunchingButton        = gamepad1.right_bumper;

        /*******************************
         * 2) SUBSYSTEMS UPDATES
         *
         */

        if (currentState == JUST_CHILLING) {

            botTop.launchMotorOff();
//            /**
//             * Enable Wobble Goal hook through bumpers
//             */
//            if (isPressedHookWobbleGoalButton) {
//                botTop.wobbleGoalHookMotorOn(0.5);
//            } else if (isPressedUnhookWobbleGoalButton) {
//                botTop.wobbleGoalHookMotorOn(-0.5);
//            }
//            if (!isPressedHookWobbleGoalButton && !isPressedUnhookWobbleGoalButton) {
//                botTop.wobbleGoalHookMotorOff();
//            }

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

            /**
             * Enable Wobble Goal Lift through trigger
             */
            if (isPressedLiftWobbleGoalButton) {
                botTop.clawMotorOn(-CLAW_POWER);
            } else if (isPressedLowerWobbleGoalButton) {
                botTop.clawMotorOn(CLAW_POWER);
            }
            if (!isPressedLiftWobbleGoalButton && !isPressedLowerWobbleGoalButton) {
                botTop.clawMotorOff();
            }

            if (isPressedLoadingButton) {
                currentState = LAUNCHING_STATE;
            }
        }

        /**
         * Updates from launcher commands
         */
        else if (currentState == LAUNCHING_STATE) {

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

            /**
             * Updates from launcher power commands
             */

            if (isPressedPowerBigIncrement) {
                wasPressedPowerBigIncrement = true;

            } else if (wasPressedPowerBigIncrement) {
                wasPressedPowerBigIncrement = false;
                launchVelocity += LAUNCH_VELOCITY_BIG_INCREMENT;
                launchVelocity = Math.min(MAX_LAUNCH_VELOCITY, launchVelocity);

            } else if (isPressedPowerSmallIncrement) {
                wasPressedPowerSmallIncrement = true;

            } else if (wasPressedPowerSmallIncrement) {
                wasPressedPowerSmallIncrement = false;
                launchVelocity += LAUNCH_VELOCITY_SMALL_INCREMENT;
                launchVelocity = Math.min(MAX_LAUNCH_VELOCITY, launchVelocity);

            } else if (isPressedPowerBigReduction) {
                wasPressedPowerBigReduction = true;

            } else if (wasPressedPowerBigReduction) {
                wasPressedPowerBigReduction = false;
                launchVelocity -= LAUNCH_VELOCITY_BIG_INCREMENT;
                launchVelocity = Math.max(MIN_LAUNCH_VELOCITY, launchVelocity);

            } else if (isPressedPowerSmallReduction) {
                wasPressedPowerSmallReduction = true;

            } else if (wasPressedPowerSmallReduction) {
                wasPressedPowerSmallReduction = false;
                launchVelocity -= LAUNCH_VELOCITY_SMALL_INCREMENT;
                launchVelocity = Math.max(MIN_LAUNCH_VELOCITY, launchVelocity);
            }

            /**
             * Calc RPM for telemetry
             */
            rpm =  botTop.getLaunchMotor().getVelocity();

            /**
             * Go back to chilling
             */
            if (isPressedResetButton) {
                wasPressedResetButton = true;
            }
            else if (wasPressedResetButton) {
                wasPressedResetButton = false;
                currentState = JUST_CHILLING;
            }



            botTop.getLaunchMotor().setVelocity(launchVelocity);
        }

        /**
         * Updates from Odometer
         */
        if (botBase.hasOdometry()) {
            botBase.odometer.globalCoordinatePositionUpdate();
        }

        /*******************************
         * 3) CALCULATIONS
         *
         */

        double rpm = botTop.getLaunchMotor().getVelocity();

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

        /**
         * Output Telemetry
         */
        telemetry.addLine("Launcher")
        .addData("Current RPM", String.format("%.3f", rpm))
        .addData("target RPM", launchVelocity);

        telemetry.addLine("Odometry")
        .addData("x", botBase.odometer.getCurrentXPos())
        .addData("y", botBase.odometer.getCurrentYPos());

        // Power
        telemetry.addLine("Power")
                .addData("FL", String.format("%.3f", wheels.front_left))
                .addData("FR", String.format("%.3f", wheels.front_right))
                .addData("RL", String.format("%.3f", wheels.rear_left))
                .addData("RR", String.format("%.3f", wheels.rear_right));

        // Object detection
        telemetry.addLine("Object Detection")
                .addData("Object Detected (label)", searchableTarget != null && searchableTarget.getTargetLabel() != null ? searchableTarget.getTargetLabel() : "none")
                .addData("Target Position", searchableTarget != null && searchableTarget.getTargetRelativePosition() != null ? searchableTarget.getTargetRelativePosition().toString() : "none");
        telemetry.update();
    }

//
//    private double getRPMs(double rpm) {
//        double now;
//        double counts;
//        now = runtime.milliseconds();
//        // waiting for rmp calculations
//        if ((now - startSampling) > RPM_CALC_DURATION) {
//            counts = Math.abs(botTop.getLaunchMotor().getCurrentPosition() - startPosition);
//            rpm = counts * RPM_CALC_DURATION * 60 / COUNTS_PER_MOTOR_REV / (now - startSampling);
//            startSampling = now;
//            startPosition = botTop.getLaunchMotor().getCurrentPosition();
//        }
//        return rpm;
//    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
        if (searchableTarget != null) {
            searchableTarget.stop();
        }
        botBase.setBling(LedPatterns.LED_OFF);
    }
}

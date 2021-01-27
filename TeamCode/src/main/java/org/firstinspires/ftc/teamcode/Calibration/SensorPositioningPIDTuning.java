package org.firstinspires.ftc.teamcode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpModesBase;
import org.firstinspires.ftc.teamcode.Components.LedPatterns;
import org.firstinspires.ftc.teamcode.Components.RevInputs;
import org.firstinspires.ftc.teamcode.Components.TravelDirection;

@Autonomous(name="PID Tuning - Position Sensor", group="1")
//@Disabled

public class SensorPositioningPIDTuning extends AutonomousOpModesBase {

    double COMMAND_VALUE_BIG_INCREMENT                  = 0.01;
    double COMMAND_VALUE_SMALL_INCREMENT                = 0.001;
    double MAX_COMMAND_VALUE                            = 1.0;
    double MIN_COMMAND_VALUE                            = 0.0001;
    boolean wasPressedPowerBigIncrement                 = false;
    boolean wasPressedPowerSmallIncrement               = false;
    boolean wasPressedPowerBigReduction                 = false;
    boolean wasPressedPowerSmallReduction               = false;
    boolean wasPressedStartButton                       = false;

    boolean wasPressedForward                           = false;
    boolean wasPressedRight                             = false;
    boolean wasPressedBackward                          = false;
    boolean wasPressedLeft                              = false;

    double[] positions                                   = {10.00, 20.00, 24.0, 4.0};
    int currentPositionIndex                             = 0;

    double DRIVE_TRAIN_TRAVELING_POWER                   = 0.7;

    TravelDirection currentDirection                    = TravelDirection.FORWARD;

    @Override
    public void initAutonomous() {
        IDENTIFICATION_SYSTEM = "NONE";
        super.initAutonomous();
        botBase.setBling(LedPatterns.LED_OFF); //off
        DEBUG = true;
    }

    @Override
    public void runOpMode() {

        initAutonomous();
        telemetry.addLine("Tuning Kp for Sensor position PID:");
        telemetry.addLine("---------------------------");
        telemetry.addLine("Use Dpad Up/Right    + Big/Small increments Kp");
        telemetry.addLine("Use Dpad Down/Left   - Big/Small increments Kp");
        telemetry.addLine("Use Y + Dpad Up/Right/Down/Left to change direction");
        telemetry.addLine("Press X to move to next position:");
        telemetry.update();

        /*********************************************
         * WAIT FOR START
         * *******************************************/

        waitForStart();
        telemetry.clearAll();
        runtime.reset();

        /*********************************************
         * GAME IS ON !!
         * *******************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            P_SENSOR_POS_COEFF = tuningCommands(P_SENSOR_POS_COEFF);
            currentDirection = selectTravelDirection(currentDirection);
            boolean isPressedStartButton          = gamepad1.x;

            if (isPressedStartButton) {
                wasPressedStartButton = true;
            }
            else if (wasPressedStartButton) {
                wasPressedStartButton = false;
                gotoHeadings(positions);
            }

            telemetry.addLine("PID")
            .addData("Kp", String.format("%.3f", P_SENSOR_POS_COEFF));
            telemetry.addLine("Sensor " + currentDirection + " ")
            .addData("Commanded", String.format("%.3f", positions[currentPositionIndex]))
            .addData("Distance", String.format("%.3f", getDistance(currentDirection)));
            telemetry.addLine("Press X to move to next position...");
            telemetry.update();

        }
        terminateAutonomous();
    }

    protected void gotoHeadings(double[] headings) {
        currentPositionIndex++;
        if (currentPositionIndex >= headings.length) {
            currentPositionIndex = 0;
        }
        moveDistanceFromObject(currentDirection, positions[currentPositionIndex], DRIVE_TRAIN_TRAVELING_POWER);
        justWait(800);
    }


    protected double tuningCommands(double commandValue) {

        /**
         * Input laucher tuning commands
         */
        boolean isPressedPowerBigIncrement = gamepad1.dpad_up;
        boolean isPressedPowerSmallIncrement = gamepad1.dpad_right;
        boolean isPressedPowerBigReduction = gamepad1.dpad_down;
        boolean isPressedPowerSmallReduction = gamepad1.dpad_left;
        boolean isPressedModeButton           = gamepad1.y;

        if (isPressedModeButton) {
            return commandValue;
        }

        if (isPressedPowerBigIncrement) {
            wasPressedPowerBigIncrement = true;

        } else if (wasPressedPowerBigIncrement) {
            wasPressedPowerBigIncrement = false;
            commandValue += COMMAND_VALUE_BIG_INCREMENT;
            commandValue = Math.min(MAX_COMMAND_VALUE, commandValue);

        } else if (isPressedPowerSmallIncrement) {
            wasPressedPowerSmallIncrement = true;

        } else if (wasPressedPowerSmallIncrement) {
            wasPressedPowerSmallIncrement = false;
            commandValue += COMMAND_VALUE_SMALL_INCREMENT;
            commandValue = Math.min(MAX_COMMAND_VALUE, commandValue);

        } else if (isPressedPowerBigReduction) {
            wasPressedPowerBigReduction = true;

        } else if (wasPressedPowerBigReduction) {
            wasPressedPowerBigReduction = false;
            commandValue -= COMMAND_VALUE_BIG_INCREMENT;
            commandValue = Math.max(MIN_COMMAND_VALUE, commandValue);

        } else if (isPressedPowerSmallReduction) {
            wasPressedPowerSmallReduction = true;

        } else if (wasPressedPowerSmallReduction) {
            wasPressedPowerSmallReduction = false;
            commandValue -= COMMAND_VALUE_SMALL_INCREMENT;
            commandValue = Math.max(MIN_COMMAND_VALUE, commandValue);
        }

        return commandValue;

    }

    protected TravelDirection selectTravelDirection(TravelDirection direction) {

        /**
         * Input selection tuning commands
         */
        boolean isPressedForward = gamepad1.dpad_up;
        boolean isPressedRight = gamepad1.dpad_right;
        boolean isPressedBackward = gamepad1.dpad_down;
        boolean isPressedLeft = gamepad1.dpad_left;
        boolean isPressedModeButton           = gamepad1.y;

        if (!isPressedModeButton) {
            return direction;
        }

        if (isPressedForward) {
            wasPressedForward = true;

        } else if (wasPressedForward) {
            wasPressedForward = false;
            direction = TravelDirection.FORWARD;

        } else if (isPressedRight) {
            wasPressedRight = true;

        } else if (wasPressedRight) {
            wasPressedRight = false;
            direction = TravelDirection.RIGHT;

        } else if (isPressedBackward) {
            wasPressedBackward = true;

        } else if (wasPressedBackward) {
            wasPressedBackward = false;
            direction = TravelDirection.BACKWARD;

        } else if (isPressedLeft) {
            wasPressedLeft = true;

        } else if (wasPressedLeft) {
            wasPressedLeft = false;
            direction = TravelDirection.LEFT;
        }

        return direction;

    }
}
package org.firstinspires.ftc.teamcode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AutonomousOpModesBase;
import org.firstinspires.ftc.teamcode.Components.LedPatterns;

@Autonomous(name="PID Tuning - Turning", group="1")
//@Disabled

public class TurnPIDTuning extends AutonomousOpModesBase {

    double COMMAND_VALUE_BIG_INCREMENT                  = 0.01;
    double COMMAND_VALUE_SMALL_INCREMENT                = 0.001;
    double MAX_COMMAND_VALUE                            = 1.0;
    double MIN_COMMAND_VALUE                            = 0.0001;
    boolean wasPressedPowerBigIncrement                 = false;
    boolean wasPressedPowerSmallIncrement               = false;
    boolean wasPressedPowerBigReduction                 = false;
    boolean wasPressedPowerSmallReduction               = false;
    boolean wasPressedStartButton                       = false;
    boolean wasPressedModeIMUButton                     = false;
    boolean wasPressedModeGyroButton                    = true;

    double[] headings                                   = {0.0, 90.0, 0.0, 180.0, 178.0, 180.0, 5.0, 10.0, 15.0, 20.0, 30.0, 90.0, 180.0, 256.0, 0.0};
    int currentHeadingIndex                             = 0;

    @Override
    public void initAutonomous() {
        IDENTIFICATION_SYSTEM = "NONE";
        DEBUG = true;
        super.initAutonomous();
        botBase.setBling(LedPatterns.LED_OFF); //off
    }

    @Override
    public void runOpMode() {

        initAutonomous();
        telemetry.addLine("Tuning Kp for rotation PID:");
        telemetry.addLine("---------------------------");
        telemetry.addLine("Use Dpad Up/Right    + Big/Small increments Kp");
        telemetry.addLine("Use Dpad Down/Left   - Big/Small increments Kp");
        telemetry.addLine("Press X to turn");
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

            P_TURN_COEFF = tuningCommands(P_TURN_COEFF);
            boolean isPressedStartButton          = gamepad1.x;

            if (isPressedStartButton) {
                wasPressedStartButton = true;
            }
            else if (wasPressedStartButton) {
                wasPressedStartButton = false;
                gotoHeadings(headings);
//                moveForwardByTime(500, 0.5);
            }

            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

            telemetry.addLine("Press X to turn");
            telemetry.addLine("PID")
            .addData("Kp", String.format("%.3f", P_TURN_COEFF));
            telemetry.addLine("Angle")
            .addData("Commanded", String.format("%.3f", headings[currentHeadingIndex]))
            .addData("IMU", String.format("%.3f", actualAngle))
            .addData("Odometry", String.format("%.3f", botBase.odometer.getHeading()));
            telemetry.update();

        }
        terminateAutonomous();
    }

    protected void gotoHeadings(double[] headings) {
        currentHeadingIndex++;
        if (currentHeadingIndex >= headings.length) {
            currentHeadingIndex = 0;
        }
        gotoHeading(headings[currentHeadingIndex]);
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
}
package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutonomousOpModesBase;
import org.firstinspires.ftc.teamcode.Components.RevInputs;

@Autonomous(name="Test Sensors", group="1")
//@Disabled

public class TestSensors extends AutonomousOpModesBase {

    protected static final double DRIVE_TRAIN_TRAVELING_POWER           = 0.2;

    @Override
    public void initAutonomous() {
        TFOD_MODEL_ASSET                            = "UltimateGoal.tflite";
        TFOD_MODEL_ASSETS_LABEL                     = new String[] {"Quad", "Single"};
        TFOD_TARGET_LABEL                           = "";
        IDENTIFICATION_SYSTEM                       = "TSF"; // can be VUFORIA, TSF, or NONE
        CAMERA_SYSTEM                               = "WEBCAM";  // can be PHONE or WEBCAM
        DEBUG = true;
        super.initAutonomous();
    }

    @Override
    public void runOpMode() {

        initAutonomous();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*********************************************
         * WAIT FOR START
         * *******************************************/

        waitForStart();
        runtime.reset();

        /*********************************************
         * GAME IS ON !!
         * *******************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            autonomousIdleTasks(RevInputs.ALL);
            displayTelemetry();
            telemetry.update();
        }
        terminateAutonomous();
    }

    protected void displayTelemetry() {

        // Odometry
        telemetry.addLine("Odometry")
                .addData("X", botBase.hasOdometry() ? botBase.odometer.getCurrentXPos() : "n/a")
                .addData("Y", botBase.hasOdometry() ? botBase.odometer.getCurrentYPos() : "n/a");

        // Distance sensor
        telemetry.addLine("Positioning")
                .addData("Left", botBase.hasSensorPositioningLeft() && botBase.distanceLeft.isValidDistance() ? String.format("%.3f", botBase.distanceLeft.getDistance()) : "n/a")
                .addData("Right", botBase.hasSensorPositioningRight() && botBase.distanceRight.isValidDistance() ? String.format("%.3f", botBase.distanceRight.getDistance()) : "n/a")
                .addData("Front", botBase.hasSensorPositioningFront() && botBase.distanceFront.isValidDistance() ? String.format("%.3f", botBase.distanceFront.getDistance()): "n/a")
                .addData("Back", botBase.hasSensorPositioningBack() && botBase.distanceBack.isValidDistance() ? String.format("%.3f", botBase.distanceBack.getDistance()) : "n/a");

        // Collision switches
        telemetry.addLine("Collision")
                .addData("Front", botBase.hasCollisionFront() ? botBase.collisionFront.isColliding() : "n/a")
                .addData("Back", botBase.hasCollisionBack() ? botBase.collisionBack.isColliding()  : "n/a");

        // Object detection
        telemetry.addLine("Object Detection")
                .addData("Object Detected", searchableTarget != null && searchableTarget.getTargetLabel() != null ? searchableTarget.getTargetLabel() : "none")
                .addData("Target Position", searchableTarget != null && searchableTarget.getTargetRelativePosition() != null ? searchableTarget.getTargetRelativePosition().toString() : "none");

        float hsvValues[] = {0F, 0F, 0F};

        if (bottomColor != null) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (bottomColor.red() * 255),
                    (int) (bottomColor.green() * 255),
                    (int) (bottomColor.blue() * 255),
                    hsvValues);
        }

        // Color sensor
        int validColor = getValidColor(bottomColor);
        telemetry.addLine("Color Sensors")
                .addData("Red", bottomColor != null ? bottomColor.red() :"none")
                .addData("Green", bottomColor != null ? bottomColor.green() :"none")
                .addData("Blue", bottomColor != null ? bottomColor.blue() :"none")
                .addData("Hue", hsvValues[0]);


        telemetry.addLine("GetValidColor()")
                .addData("Color Detected", validColor == Color.RED ? "Red" : validColor == Color.BLUE ? "Blue" : validColor == Color.YELLOW ? "Yellow" : validColor == Color.WHITE ? "White" : validColor == Color.BLACK ? "Black" : "Unknown");
    }

}
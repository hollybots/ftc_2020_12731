package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.BotSounds;
import org.firstinspires.ftc.teamcode.Components.FieldPlacement;

@Autonomous(name="Test Suite", group="1")
//@Disabled

public class Autonomous_test extends AutonomousOpModesBase {
    // I am adding this comment

    protected static final int TEST_idle                       = 0;
    protected static final int TEST_done                       = 1;

    protected static final int TEST_moveLaterally               = 5;


    protected int currentTest                                   = TEST_idle;

    protected FieldPlacement stoneRelativePlacement             = null;

    // Sounds
    protected BotSounds botSounds = null;

    // alignment camera from center toward the the left is negative (with the selfie side forward)
    protected static final double CAMERA_TO_CENTER               = -1.8;

    protected static final int MAX_CYCLES_FOR_FINDING_STONE      = 3;

    protected static final double BLING_MODE_CLAMP               = LED_TEAM_COLORS4;
    protected static final double DISTANCE_TO_STONEWALL          = 12.0;


    @Override
    public void initAutonomous() {
        IDENTIFICATION_SYSTEM = "NONE";
        DEBUG = true;
        super.initAutonomous();

        /* **********************************
           LIGHTS
        */
        botBase.setBling(0.7745);
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

            autonomousIdleTasks();

            if ( currentTest == TEST_done ) {
                 break;
            }

            switch (currentTest) {

                case TEST_idle:
                    displayTelemetry();
                    break;

                case TEST_done:
                    stopMoving();
                    break;

                case TEST_moveLaterally:
                    moveFrontAndLaterally();
                    break;
            }
            telemetry.update();
        }

        terminateAutonomous();
    }

    protected void moveFrontAndLaterally() {

    }
    protected void displayTelemetry() {

        telemetry.addLine("Positioning")
                .addData("Left", botBase.hasSensorPositioningLeft() ? String.format("%.3f", botBase.distanceLeft.getDistance()) : "n/a")
                .addData("Right", botBase.hasSensorPositioningRight() ? String.format("%.3f", botBase.distanceRight.getDistance()) : "n/a")
                .addData("Front", botBase.hasSensorPositioningFront() ? String.format("%.3f", botBase.distanceFront.getDistance()): "n/a")
                .addData("Back", botBase.hasSensorPositioningBack() ? String.format("%.3f", botBase.distanceBack.getDistance()) : "n/a");

        telemetry.addLine("Collision")
                .addData("Front", botBase.hasCollisionFront() ? botBase.collisionFront.isColliding() : "n/a")
                .addData("Back", botBase.hasCollisionBack() ? botBase.collisionBack.isColliding()  : "n/a");

        telemetry.addLine("Object Detection")
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
        int validColor = getValidColor(bottomColor);
        telemetry.addLine("Color Sensors")
                .addData("Red", bottomColor != null ? bottomColor.red() :"none")
                .addData("Green", bottomColor != null ? bottomColor.green() :"none")
                .addData("Blue", bottomColor != null ? bottomColor.blue() :"none")
                .addData("Hue", hsvValues[0]);


        telemetry.addLine("GetValidColor()")
                .addData("Color Detected", validColor == Color.RED ? "Red" : validColor == Color.BLUE ? "Blue" : validColor == Color.YELLOW ? "Yellow" : validColor == Color.BLACK ? "Black" : "Unknown");
    }

}

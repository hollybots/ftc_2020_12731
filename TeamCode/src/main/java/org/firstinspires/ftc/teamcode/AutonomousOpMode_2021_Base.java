package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Components.FieldPlacement;
import org.firstinspires.ftc.teamcode.Components.LedPatterns;

@Autonomous(name="Autonomous 2021", group="none")
@Disabled

public class AutonomousOpMode_2021_Base extends AutonomousOpModesBase {


    protected static final double DRIVE_TRAIN_TRAVELING_POWER           = 0.68;
    protected static final double LAUNCH_POWER                          = 0.8;

    protected static final double WOBBLE_GOAL_DELIVERY_POWER            = 0.4; // lifting is negative, lowering is positive
    protected static final double TIME_TO_DELIVER                       = 2500;

    protected static final int TIME_TO_EXTEND                           = 300; //ms
    protected static final int TIME_TO_RETRACT                          = 300; //ms

    protected static final int NB_RINGS_POWER_SHOT                      = 1;

    protected static final double LAUNCHER_X                            = 24.0;  // position in inches from the robot
    protected static final double LAUNCHER_Y                            = 0.5;

    /**
     * All possible states
     */
    protected static final int STATE_idle                       = 0;

    protected static final int STATE_STARTER_STACK              = 1;
    protected static final int STATE_MOVE_TO_TARGET_ZONE        = 2;
    protected static final int STATE_DELIVER_WOBBLE_GOAL        = 3;
    protected static final int STATE_TRAVEL_TO_RING_LAUNCHER    = 4;
    protected static final int STATE_TOWER_SHOT                 = 5;
    protected static final int STATE_TRAVEL_TO_POWER_SHOT       = 6;
    protected static final int STATE_POWER_SHOT                 = 7;
    protected static final int STATE_TRAVEL_TO_LAUNCH_LINE      = 8;


    protected static final int STATE_done                       = 50;

    // State variables
    int remainingRings                      = 3;

    /**
     * State your Opmode is currently in
     */
    protected int currentState              = STATE_idle;

    // TSF
    private FieldPlacement ringPlacement        = null;
    protected String ringLabel                  = "None";


    @Override
    public void initAutonomous() {

        DEBUG = true;
        super.initAutonomous();

        botTop.liftMagazine();
        botTop.retractArm();
        botTop.intakeMotorOff();

    }

    @Override
    public void runOpMode() {
        TFOD_MODEL_ASSET                            = "UltimateGoal.tflite";
        TFOD_MODEL_ASSETS_LABEL                     = new String[] {"Quad", "Single"};
        TFOD_TARGET_LABEL                           = "";
        IDENTIFICATION_SYSTEM                       = "TSF"; // can be VUFORIA, TSF, or NONE
        CAMERA_SYSTEM                               = "WEBCAM";  // can be PHONE or WEBCAM
        currentState                                = STATE_STARTER_STACK;

        initAutonomous();

        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Initial Rings", "" + ringDetector.getPosition());
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
            switch (currentState) {

                case STATE_done:
                    break;

                case STATE_idle:
                    stopMoving();
                    break;

                case STATE_STARTER_STACK:
                    detectInitialStack(1000);
                    break;

                case STATE_MOVE_TO_TARGET_ZONE:
                    moveToTargetZone();
                    break;

                case STATE_DELIVER_WOBBLE_GOAL:
                    deliverWobbleGoal();
                    break;

                case STATE_TRAVEL_TO_RING_LAUNCHER:
                    travelToRingLauncher();
                    break;

                case STATE_TOWER_SHOT:
                    towerShot();
                    break;

                case STATE_TRAVEL_TO_POWER_SHOT:
                    travelToPowerShot();
                    break;

                case STATE_POWER_SHOT:
                    powerShot();
                    break;

                case STATE_TRAVEL_TO_LAUNCH_LINE:
                    travelToLaunchLine();
                    break;

            }
//            telemetry.addData("Actual Rings", "" + ringPosition);
            telemetry.update();
        }
    }


    protected void detectInitialStack(double ms) {

        double limit = runtime.milliseconds() + ms;
        while (
                opModeIsActive() &&
                runtime.milliseconds() < limit  && ringPlacement == null
        ) {
            autonomousIdleTasks();
            if (searchableTarget != null) {
                ringPlacement = searchableTarget.getTargetRelativePosition();
                ringLabel = searchableTarget.getTargetLabel();
            }
        }
        searchableTarget.stop();
        currentState = STATE_MOVE_TO_TARGET_ZONE;
    }

    protected void moveToTargetZone() {
        /***
         * Implemented in base class
         */
    }

    protected void deliverWobbleGoal() {
        botTop.clawMotorOn(WOBBLE_GOAL_DELIVERY_POWER);
        justWait(TIME_TO_DELIVER);
        botTop.clawMotorOff();
        currentState = STATE_TRAVEL_TO_RING_LAUNCHER;
        return;
    }

    protected void travelToRingLauncher() {
        /***
         * Implemented in base class
         */
    }

    protected void towerShot() {
        double timeLeft = 30 - runtime.seconds();
        while (timeLeft > 5.0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
        }
        currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
    }

    protected void travelToPowerShot() {
        /***
         * Implemented in base class
         */
    }

    protected void powerShot() {

        double timeLeft = 30 - runtime.seconds();
        while (timeLeft > 3.0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
        }
        currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
    }

    protected void travelToLaunchLine() {
        moveForwardToColor(Color.WHITE, 0.3);
        botBase.setBling(LedPatterns.LED_TEAM_COLORS3);
        botTop.stopAll();
        super.terminateAutonomous();
        currentState = STATE_done;
        return;
    }
}
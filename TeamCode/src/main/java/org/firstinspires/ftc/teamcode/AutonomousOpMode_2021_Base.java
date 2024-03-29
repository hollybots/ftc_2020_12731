package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Components.FieldPlacement;
import org.firstinspires.ftc.teamcode.Components.LedPatterns;
import org.firstinspires.ftc.teamcode.Components.RevInputs;

@Autonomous(name="Autonomous 2021", group="none")
@Disabled

public class AutonomousOpMode_2021_Base extends AutonomousOpModesBase {


    protected static final double DRIVE_TRAIN_TRAVELING_POWER              = 0.7;
    protected static final double LAUNCH_VELOCITY                          = 1700;
    protected static final double LAUNCH_VELOCITY_NONE                     = LAUNCH_VELOCITY;
    protected static final double LAUNCH_VELOCITY_SINGLE                   = LAUNCH_VELOCITY;
    protected static final double LAUNCH_VELOCITY_QUAD                     = LAUNCH_VELOCITY;
    protected static final double LAUNCH_VELOCITY_POWER_SHOT_FRONT         = 1550;
    protected static final double INTAKE_MOTOR                          = 0.9;

    protected static final double WOBBLE_GOAL_DELIVERY_POWER            = 0.6; // lifting is negative, lowering is positive
    protected static final double TIME_TO_DELIVER                       = 1800;

    protected static final int TIME_TO_EXTEND                           = 300; //ms
    protected static final int TIME_TO_RETRACT                          = 350; //ms
//    protected static final int TIME_TO_RETRACT                          = 300; //ms

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
    protected static final int STATE_PICKUP_RINGS               = 9;
    protected static final int STATE_MOVE_XTRA_WOBBLE_GOAL      = 10;


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

            autonomousIdleTasks(RevInputs.ALL);
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

                case STATE_PICKUP_RINGS:
                    pickupExtraRings();
                    break;

                case STATE_MOVE_XTRA_WOBBLE_GOAL:
                    moveExtraWobbleGoal();
                    break;
            }
        }
    }


    protected void detectInitialStack(double ms) {

        double limit = runtime.milliseconds() + ms;
        while (
                opModeIsActive() &&
                runtime.milliseconds() < limit  && ringPlacement == null
        ) {
            autonomousIdleTasks(RevInputs.OBJECT_RECON);
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
         * Implemented in extended class
         */
        currentState = STATE_DELIVER_WOBBLE_GOAL;
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
         * Implemented in extended class
         */
        currentState = STATE_TOWER_SHOT;
    }

    protected void towerShot() {
        int t = 4;
        while (t > 0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
            t = t - 1;
        }
        if ((ringLabel == "Single") || (ringLabel == "Quad"))  {
            currentState = STATE_PICKUP_RINGS;
        }
        else if (ringLabel == "None") {
            currentState = STATE_MOVE_XTRA_WOBBLE_GOAL;
        } else {
            currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
        }
    }

    protected void travelToPowerShot() {
        /***
         * Implemented in extended class
         */
        currentState = STATE_POWER_SHOT;
    }

    protected void powerShot() {
        double timeLeft = 30.0 - runtime.seconds();
        while (timeLeft > 3.0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
        }
        currentState = STATE_PICKUP_RINGS;
    }

    protected void pickupExtraRings() {
        /***
         * Implemented in extended class
         */
        currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
    }

    protected void moveExtraWobbleGoal() {
        /***
         * Implemented in extended class
         */
        currentState = STATE_done;
    }

    protected void travelToLaunchLine() {
        botTop.launchMotorOff();
        moveForwardToColor(Color.WHITE, 0.3);
        botTop.stopAll();
        botBase.setBling(LedPatterns.LED_TEAM_COLORS3);
        super.terminateAutonomous();
        currentState = STATE_done;
        return;
    }
}
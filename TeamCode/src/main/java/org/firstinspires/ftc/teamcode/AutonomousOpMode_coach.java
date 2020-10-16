package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.OpenCV.RingDetector;
import org.firstinspires.ftc.teamcode.OpenCV.RingPosition;

@Autonomous(name="Autonomous Coach", group="none")
//@Disabled

public class AutonomousOpMode_coach extends AutonomousOpModesBase {


    protected static final double DRIVE_TRAIN_TRAVELING_POWER           = 0.75;

    protected static final double WOBBLE_GOAL_DELIVERY_POWER            = 0.3;
    protected static final double TIME_TO_DELIVER                       = 2000;

    /**
     * All possible states
     */
    protected static final int STATE_idle                       = 0;

    protected static final int STATE_STARTER_STACK              = 1;
    protected static final int STATE_MOVE_TO_TARGET_ZONE        = 2;
    protected static final int STATE_DELIVER_WOBBLE_GOAL        = 3;
    protected static final int STATE_TRAVEL_TO_RING_LAUNCHER    = 4;
    protected static final int STATE_TOWER_SHOT                 = 5;
    protected static final int STATE_POWER_SHOT                 = 6;
    protected static final int STATE_TRAVEL_TO_START_LINE       = 7;

    protected static final int STATE_done                       = 50;

    RingDetector ringDetector                                   = null;


    // State variables
    RingPosition ringPosition               = null;
    /**
     * State your Opmode is currently in
     */
    protected int currentState              = STATE_idle;


    @Override
    public void initAutonomous() {

        DEBUG = true;
        super.initAutonomous();
        ringDetector =  RingDetector.init(hardwareMap, "WEBCAM", true);
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

            if (currentState == STATE_done) {
                break;
            }

            switch (currentState) {

                case STATE_idle:
                    stopMoving();
                    break;

                case STATE_STARTER_STACK:
                    detectInitialStack();
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

                case STATE_POWER_SHOT:
                    powerShot();
                    break;

                case STATE_TRAVEL_TO_START_LINE:
                    travelToStartLine();
                    break;
            }
            telemetry.update();
        }

        super.terminateAutonomous();
    }


    protected void detectInitialStack() {
        // <CODE HERE>
        currentState = STATE_MOVE_TO_TARGET_ZONE;
    }

    protected void moveToTargetZone() {
        switch (ringPosition) {
            case NONE:
                moveBackward(48.0, 0.5);
                gotoHeading(180);
                break;
            case ONE:
                // <CODE HERE>
                break;
            case FOUR:
                // <CODE HERE>
                break;
        }
        currentState = STATE_DELIVER_WOBBLE_GOAL;
        return;
    }

    protected void deliverWobbleGoal() {
        justWait(500);
        botTop.clawMotorOn(WOBBLE_GOAL_DELIVERY_POWER);
        justWait(TIME_TO_DELIVER);
        botTop.clawMotorOff();
        currentState = STATE_TRAVEL_TO_RING_LAUNCHER;
        return;
    }

    protected void travelToRingLauncher() {
        gotoHeading(0);
        moveLeft(24.0, 0.5);
        currentState = STATE_TOWER_SHOT;
        return;
    }

    protected void towerShot() {
        botTop.launchMotorOn(0.75);
        botTop.extendArm();
        justWait(500);
        botTop.retractArm();
        return;
    }

    protected void powerShot() {
        // <CODE HERE>
        currentState = STATE_TRAVEL_TO_START_LINE;
        return;
    }

    protected void travelToStartLine() {
        // <CODE HERE>
        currentState = STATE_done;
        return;
    }

}
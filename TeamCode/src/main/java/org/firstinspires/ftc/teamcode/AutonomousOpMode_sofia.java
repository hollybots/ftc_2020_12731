package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpenCV.RingDetector;
import org.firstinspires.ftc.teamcode.OpenCV.RingPosition;

@Autonomous(name="Autonomous Sofia", group="none")
@Disabled

public class AutonomousOpMode_sofia extends AutonomousOpModesBase {

    protected static final double DRIVE_TRAIN_TRAVELING_POWER           = 0.75;
    protected static final double LAUNCH_POWER                          = 0.75;

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
    protected int currentState = STATE_idle;

    @Override
    public void initAutonomous() {

        //

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

    // how do i do this
    // i don't see anything in the base component to detect how many rings are on the marker
    protected void detectInitialStack() {
        // <CODE HERE>
        currentState = STATE_MOVE_TO_TARGET_ZONE;
    }

    protected void moveToTargetZone() {
        switch (ringPosition) {
            case NONE:
                moveLeft(12, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(48.0, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(180);
                break;
            case ONE:
                moveRight(12, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(72.0, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(180);
                break;
            case FOUR:
                moveLeft(12, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(96.0, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(180);
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
        moveLeft(24.0, DRIVE_TRAIN_TRAVELING_POWER);
        currentState = STATE_TOWER_SHOT;
        return;
    }

    protected void towerShot() {
        botTop.launchMotorOn(LAUNCH_POWER);
        botTop.extendArm();
        justWait(500);
        botTop.retractArm();
        return;
    }

    // I'm making it so that the robot moves a bit to the side after shooting each power shot target, don't know if the distance between each target is right, though.
    protected void powerShot() {
        // shoot once
        botTop.launchMotorOn(LAUNCH_POWER);
        botTop.extendArm();
        justWait(100);
        botTop.retractArm();
        moveRight(6, DRIVE_TRAIN_TRAVELING_POWER);
        // shoot again
        botTop.launchMotorOn(LAUNCH_POWER);
        botTop.extendArm();
        justWait(100);
        botTop.retractArm();
        moveRight(6, DRIVE_TRAIN_TRAVELING_POWER);
        // and again
        botTop.launchMotorOn(LAUNCH_POWER);
        botTop.extendArm();
        justWait(100);
        botTop.retractArm();
        moveRight(6, DRIVE_TRAIN_TRAVELING_POWER);
        currentState = STATE_TRAVEL_TO_START_LINE;
        return;
    }

    protected void travelToStartLine() {
        moveXInchesFromFrontObject(1, 3, DRIVE_TRAIN_TRAVELING_POWER);
        moveXInchesFromLeftObject(12, 5, DRIVE_TRAIN_TRAVELING_POWER);
        currentState = STATE_done;
        return;
    }

}
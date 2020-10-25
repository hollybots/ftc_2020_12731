package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.LedPatterns;
import org.firstinspires.ftc.teamcode.OpenCV.RingDetector;
import org.firstinspires.ftc.teamcode.OpenCV.RingPosition;

@Autonomous(name="Autonomous Sofia", group="none")
//@Disabled

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
    protected static final int STATE_TRAVEL_TO_LAUNCH_LINE       = 7;

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

        currentState = STATE_STARTER_STACK;

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

                case STATE_TRAVEL_TO_LAUNCH_LINE:
                    travelToLaunchLine();
                    break;

            }
            telemetry.update();
        }

        super.terminateAutonomous();

    }

    protected void detectInitialStack() {
        ringPosition = ringDetector.getPosition();
        currentState = STATE_MOVE_TO_TARGET_ZONE;
    }

    protected void moveToTargetZone() {
        botBase.setBling(LedPatterns.LED_SOLID_COLOR_ORANGE);
        switch (ringPosition) {
            default:
            case NONE:
                moveXInchesFromLeftObject(6, 4000, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(48.0, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            case ONE:
                moveXInchesFromLeftObject(6, 4000, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(72.0, DRIVE_TRAIN_TRAVELING_POWER);
                moveRight(18, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            case FOUR:
                moveXInchesFromLeftObject(6, 4000, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(96.0, DRIVE_TRAIN_TRAVELING_POWER);
                break;
        }
        currentState = STATE_DELIVER_WOBBLE_GOAL;
        return;
    }

    protected void deliverWobbleGoal() {
        botBase.setBling(LedPatterns.LED_SOLID_COLOR_ORANGE);
        justWait(500);
        botTop.clawMotorOn(WOBBLE_GOAL_DELIVERY_POWER);
        justWait(TIME_TO_DELIVER);
        botTop.clawMotorOff();
        gotoHeading(180);
        currentState = STATE_TRAVEL_TO_RING_LAUNCHER;
        return;
    }

    // make distance relative to position
    protected void travelToRingLauncher() {
        botTop.launchMotorOn(LAUNCH_POWER);
        botBase.setBling(LedPatterns.LED_SOLID_COLOR_ORANGE);
        switch (ringPosition) {
            default:
            case NONE:
                moveLeft(12.0, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(24.0, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            case ONE:
                moveLeft(12.0, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(48.0, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            case FOUR:
                moveLeft(12.0, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(72.0, DRIVE_TRAIN_TRAVELING_POWER);
                break;
        }
        currentState = STATE_TOWER_SHOT;
        return;
    }

    protected void towerShot() {
        botBase.setBling(LedPatterns.LED_SOLID_COLOR_RED);
        botTop.extendArm();
        justWait(500);
        botTop.retractArm();
        currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
        return;
    }

    // I'm making it so that the robot moves a bit to the side after shooting each power shot target, don't know if the distance between each target is right, though.
    protected void powerShot() {
        botBase.setBling(LedPatterns.LED_SOLID_COLOR_RED);
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
        currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
        return;
    }

    // make a white LED pattern so we can use it here
    protected void travelToLaunchLine() {
        botTop.launchMotorOff();
        moveForwardToColor(Color.WHITE, DRIVE_TRAIN_TRAVELING_POWER);
        currentState = STATE_done;
        return;
    }

}
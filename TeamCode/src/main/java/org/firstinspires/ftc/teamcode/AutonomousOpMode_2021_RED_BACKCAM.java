package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This Opmode is extended from the Season's Autonomous main Opmode
 * In this extended class, we account for
 * 1) the webcam positioned at the very back of the robot
 * 2) the RED alliance starting position
 * so the robot start the autonomous mode backward, and eventually turns in order to shoot the rings.
 */

@Autonomous(name="Autonomous RED - BACKCAM", group="none")
//@Disabled

public class AutonomousOpMode_2021_RED_BACKCAM extends AutonomousOpMode_2021_Base {

    public void initAutonomous() {
        super.initAutonomous();
        // the robot is starting facing back, so all directions are reversed
        if (botBase.hasOdometry()) {
            botBase.odometer.reverseLeftEncoder();
            botBase.odometer.reverseRightEncoder();
            botBase.odometer.reverseHorizontalEncoder();
        }
    }

    protected void moveToTargetZone() {
        if (ringLabel == null) {ringLabel = "None";}
        moveBackward(2.0, DRIVE_TRAIN_TRAVELING_POWER/1.25);
        gotoHeading(0);
        moveXInchesFromLeftObject(10.0, 5000, DRIVE_TRAIN_TRAVELING_POWER/1.25);
        switch (ringLabel) {
            case "Quad":
                moveBackward(48, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                moveBackward(48, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                break;
            case "Single":
                moveBackward(74.0, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                moveRight(-18, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                break;
            case "None":
            default:
                moveBackward(48, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(2);
                break;
        }
        currentState = STATE_DELIVER_WOBBLE_GOAL;
        return;
    }

    protected void travelToRingLauncher() {
        if (ringLabel == null) {ringLabel = "None";}
        switch (ringLabel) {
            case "Quad":
                moveForward(-41, DRIVE_TRAIN_TRAVELING_POWER);
                moveRight(-15, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            case "Single":
                moveForward(-20, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            case "None":
            default:
                moveRight(-12, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(5, DRIVE_TRAIN_TRAVELING_POWER);
            break;
        }
        botTop.launchMotorOn(LAUNCH_POWER);
        gotoHeading(180);
        currentState = STATE_TOWER_SHOT;
        return;
    }

    protected void pickupExtraRings() {
        currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
    }
}
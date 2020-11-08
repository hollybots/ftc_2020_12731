package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This Opmode is extended from the Season's Autonomous main Opmode
 * In this extended class, we account for
 * 1) the webcam positioned at the front of the robot
 * 2) the RED alliance starting position
 * so the robot start the autonomous mode facing forward, and eventually turns if target position is C.
 */

@Autonomous(name="Autonomous RED - FRONTCAM", group="none")
//@Disabled

public class AutonomousOpMode_2021_RED_FRONTCAM extends AutonomousOpMode_2021_Base {

    protected void moveToTargetZone() {

        switch (ringLabel) {
            case "Single":
                moveXInchesFromRightObject(10.0, 5000, DRIVE_TRAIN_TRAVELING_POWER);
                moveForward(105, DRIVE_TRAIN_TRAVELING_POWER);
                moveLeft(-10, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            case "Quad":
                moveForward(6, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(180);
                moveXInchesFromLeftObject(10.0, 5000, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(92, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            default:
                moveXInchesFromRightObject(10.0, 5000, DRIVE_TRAIN_TRAVELING_POWER);
                moveForward(80, DRIVE_TRAIN_TRAVELING_POWER);
                break;
        }
        justWait(500);
        currentState = STATE_DELIVER_WOBBLE_GOAL;
    }

    protected void travelToRingLauncher() {
        switch (ringLabel) {
            case "Single":
                moveBackward(-52, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            case "Quad":
                moveForward(-48, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                moveLeft(-24, DRIVE_TRAIN_TRAVELING_POWER);
                break;
            default:
                moveBackward(-30, DRIVE_TRAIN_TRAVELING_POWER);
                moveLeft(-24, DRIVE_TRAIN_TRAVELING_POWER);
            break;
        }
        botTop.launchMotorOn(LAUNCH_POWER);
        currentState = STATE_TOWER_SHOT;
        return;
    }

    protected void travelToPowerShot() {
        gotoHeading(-30.0);
    }
}
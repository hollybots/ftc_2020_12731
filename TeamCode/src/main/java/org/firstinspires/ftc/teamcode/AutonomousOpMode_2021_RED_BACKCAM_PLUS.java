package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This Opmode is extended from the Season's Autonomous main Opmode
 * In this extended class, we account for
 * 1) the webcam positioned at the very back of the robot
 * 2) the RED alliance starting position
 * so the robot start the autonomous mode backward, and eventually turns in order to shoot the rings.
 */

@Autonomous(name="Autonomous RED MOTION PLUS(TM)", group="none")
//@Disabled

public class AutonomousOpMode_2021_RED_BACKCAM_PLUS extends AutonomousOpMode_2021_Base {

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
                moveRight(-21, DRIVE_TRAIN_TRAVELING_POWER);
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
                botTop.launchMotorOn(LAUNCH_POWER);
                gotoHeading(177);
                break;
            case "Single":
                moveForward(-23, DRIVE_TRAIN_TRAVELING_POWER);
                botTop.launchMotorOn(LAUNCH_POWER);
                gotoHeading(170);
                break;
            case "None":
            default:
                moveRight(-12, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(2, DRIVE_TRAIN_TRAVELING_POWER);
                botTop.launchMotorOn(LAUNCH_POWER);
                gotoHeading(170);
            break;
        }
        currentState = STATE_TOWER_SHOT;
        return;
    }

    protected void pickupExtraRings() {
        botTop.intakeMotorOn(LAUNCH_POWER/1.5);
        gotoHeading(0);
        botTop.lowerMagazine();
        moveForward(-20.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveBackward(12.0, DRIVE_TRAIN_TRAVELING_POWER);
        botTop.liftMagazine();
        gotoHeading(200);
        int t = 2;
        while (t > 0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
            t--;
        }
        currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
        return;
    }
}
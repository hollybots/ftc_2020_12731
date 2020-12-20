package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This Opmode is extended from the Season's Autonomous main Opmode
 * In this extended class, we account for
 * 1) the webcam positioned at the very back of the robot
 * 2) the RED alliance starting position
 * so the robot start the autonomous mode backward, and eventually turns in order to shoot the rings.
 *
 * NOTE:  Directions FORWARD, BACKWARD, LEFT and RIGHT are given in the robots coordinate the positioning values are
 * vectored values given in field coordinates: positive Y is moving toward the tower goal and positive X is moving toward
 * the red alliance wall.
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
        moveBackward(1.0, DRIVE_TRAIN_TRAVELING_POWER/1.75);
        gotoHeading(0);
        moveLeftXInchesFromLeftObject(10.0, 5000, DRIVE_TRAIN_TRAVELING_POWER/1.25);
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
                moveRight(-17, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                break;
            case "None":
            default:
                moveBackward(54, DRIVE_TRAIN_TRAVELING_POWER);
                moveLeftXInchesFromLeftObject(5.0, 5000, DRIVE_TRAIN_TRAVELING_POWER/1.25);
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
                moveForward(-36, DRIVE_TRAIN_TRAVELING_POWER);
                moveRightXInchesFromLeftObject(20, 10000, DRIVE_TRAIN_TRAVELING_POWER);
                botTop.launchMotorOn(LAUNCH_POWER_QUAD);
                gotoHeading(176);
                break;
            case "Single":
                moveForward(-13, DRIVE_TRAIN_TRAVELING_POWER);
                botTop.launchMotorOn(LAUNCH_POWER_SINGLE);
                gotoHeading(173);
                break;
            case "None":
            default:
                moveRightXInchesFromLeftObject(20, 10000, DRIVE_TRAIN_TRAVELING_POWER);
//                moveRight(-15, DRIVE_TRAIN_TRAVELING_POWER);
                moveBackward(2, DRIVE_TRAIN_TRAVELING_POWER);
                botTop.launchMotorOn(LAUNCH_POWER_NONE);
                gotoHeading(178);
                break;

        }
        currentState = STATE_TOWER_SHOT;
        return;
    }

    protected void pickupExtraRings() {
        if (ringLabel == "Single") {
            pickupExtraOneRing();
        } else if (ringLabel == "Quad") {
            pickupExtraFourRings();
        }
        currentState = STATE_TRAVEL_TO_LAUNCH_LINE;
    }

    protected void pickupExtraOneRing() {
        botTop.intakeMotorOn(INTAKE_MOTOR);
        botTop.launchMotorOn(LAUNCH_POWER_POWER_SHOT_FRONT);
        gotoHeading(0);
        botTop.lowerMagazine();
        moveForward(-25.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveBackward(10.0, DRIVE_TRAIN_TRAVELING_POWER);
        botTop.liftMagazine();
        gotoHeading(198);
        int t = 2;
        while (t > 0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
            t--;
        }
    }


    protected void moveExtraWobbleGoal() {
        botTop.launchMotorOff();
        botTop.clawMotorOn(-WOBBLE_GOAL_DELIVERY_POWER);
        justWait(TIME_TO_DELIVER/2.5);
        botTop.clawMotorOff();
        autonomousIdleTasks();
        moveRight(4.0, DRIVE_TRAIN_TRAVELING_POWER/1.5);
        moveXInchesFromBackObject(8.0, 10000, DRIVE_TRAIN_TRAVELING_POWER);
        moveXInchesFromBackObject(2.0, 10000, DRIVE_TRAIN_TRAVELING_POWER/2.0);
        moveLeft(-29.0, DRIVE_TRAIN_TRAVELING_POWER/1.5);
        gotoHeading(150);
        moveForward(5, DRIVE_TRAIN_TRAVELING_POWER/2.0);
        moveForward(51, DRIVE_TRAIN_TRAVELING_POWER/1.5);
        currentState = STATE_done;
    }


    protected void pickupExtraFourRings() {
        botTop.launchMotorOn(LAUNCH_POWER_POWER_SHOT_FRONT);
        gotoHeading(0);
        botTop.lowerMagazine();
        moveRight(-2.0, DRIVE_TRAIN_TRAVELING_POWER/1.25);
        moveForward(-12.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveBackward(2.0, DRIVE_TRAIN_TRAVELING_POWER/2.0);
        botTop.intakeMotorOn(INTAKE_MOTOR);
        moveForward(-3.0, DRIVE_TRAIN_TRAVELING_POWER);
        justWait(500);
        gotoHeading(200);
        moveForward(12.0, DRIVE_TRAIN_TRAVELING_POWER);
        botTop.liftMagazine();
        justWait(450);
        int t = 2;
        while (t > 0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
            moveRight(2.0, DRIVE_TRAIN_TRAVELING_POWER/3.0);
            t--;
        }
    }
}
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Components.TravelDirection;


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
    }

    protected void moveToTargetZone() {
        if (ringLabel == null) {ringLabel = "None";}
        moveBackward(2.0, DRIVE_TRAIN_TRAVELING_POWER);
        gotoHeading(0);
//        moveLeftXInchesFromLeftObject(10.0, 5000, DRIVE_TRAIN_TRAVELING_POWER/1.25);
        moveDistanceFromObject(TravelDirection.LEFT, 8.0, DRIVE_TRAIN_TRAVELING_POWER);
//        moveDistanceFromObject(TravelDirection.LEFT, 10.0, DRIVE_TRAIN_TRAVELING_POWER);
        switch (ringLabel) {
            case "Quad":
                gotoHeading(0);
                moveBackward(80, DRIVE_TRAIN_TRAVELING_POWER);
                moveDistanceFromObject(TravelDirection.BACKWARD,24.0, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                break;
            case "Single":
                moveBackward(76.0, DRIVE_TRAIN_TRAVELING_POWER*1.1);
//                moveBackward(74.0, DRIVE_TRAIN_TRAVELING_POWER*1.1);
                gotoHeading(0);
//                moveRight(17, DRIVE_TRAIN_TRAVELING_POWER);
                moveDistanceFromObject(TravelDirection.LEFT,25.0, DRIVE_TRAIN_TRAVELING_POWER);
//                moveRight(22, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                break;
            case "None":
            default:
                moveBackward(56, DRIVE_TRAIN_TRAVELING_POWER);
//                moveLeftXInchesFromLeftObject(5.0, 5000, DRIVE_TRAIN_TRAVELING_POWER/1.25);
                moveDistanceFromObject(TravelDirection.LEFT,3.0, DRIVE_TRAIN_TRAVELING_POWER);
                gotoHeading(0);
                break;
        }
        currentState = STATE_DELIVER_WOBBLE_GOAL;
        return;
    }

    protected void travelToRingLauncher() {
        if (ringLabel == null) {ringLabel = "None";}
        switch (ringLabel) {
            case "Quad":
//                moveForward(33, DRIVE_TRAIN_TRAVELING_POWER);
//                moveForward(34, DRIVE_TRAIN_TRAVELING_POWER);
                moveForward(36, DRIVE_TRAIN_TRAVELING_POWER);
                moveDistanceFromObject(TravelDirection.LEFT, 26.0, DRIVE_TRAIN_TRAVELING_POWER);
                botTop.launchMotorOn(LAUNCH_POWER_QUAD);
                gotoHeading(181);
//                gotoHeading(180);
                break;
            case "Single":
                moveForward(17, DRIVE_TRAIN_TRAVELING_POWER*1.1);
//                moveForward(18, DRIVE_TRAIN_TRAVELING_POWER*1.1);
//                moveForward(16, DRIVE_TRAIN_TRAVELING_POWER*1.1);
//                moveForward(14, DRIVE_TRAIN_TRAVELING_POWER*1.1);
                botTop.launchMotorOn(LAUNCH_POWER_SINGLE);
//                gotoHeading(173);
//                moveDistanceFromObject(TravelDirection.LEFT, 26.0, DRIVE_TRAIN_TRAVELING_POWER*1.1);
                moveDistanceFromObject(TravelDirection.LEFT, 27.0, DRIVE_TRAIN_TRAVELING_POWER*1.1);
                gotoHeading(181);

                break;
            case "None":
            default:
                moveDistanceFromObject(TravelDirection.LEFT,27.0, DRIVE_TRAIN_TRAVELING_POWER);
                botTop.launchMotorOn(LAUNCH_POWER_NONE);
                gotoHeading(181);
                moveForward(3.5, DRIVE_TRAIN_TRAVELING_POWER);
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
        moveDistanceFromObject(TravelDirection.LEFT, 28.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveForward(25.0, DRIVE_TRAIN_TRAVELING_POWER*1.1);

        // pick up goal and push forward
        moveDistanceFromObject(TravelDirection.LEFT, 22.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveDistanceFromObject(TravelDirection.FORWARD, 2.0, DRIVE_TRAIN_TRAVELING_POWER);
        botTop.intakeMotorOff();
        moveRight(14.0, DRIVE_TRAIN_TRAVELING_POWER);
//        gotoHeading(-8);
        gotoHeading(-5);
//        gotoHeading(-3);
//        moveBackward(84.0, DRIVE_TRAIN_TRAVELING_POWER/1.5);
        moveBackward(83.0, DRIVE_TRAIN_TRAVELING_POWER/1.2);

        // shoot the ring
//        moveForward(30.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveForward(32.0, DRIVE_TRAIN_TRAVELING_POWER);
        botTop.liftMagazine();
        // counter clockwise from 180 is positive
        //        gotoHeading(200);
        // counter clockwise from 180 is positive
//        gotoHeading(198);
        gotoHeading(198);
//        gotoHeading(195);
        // hits the middle one
        int t = 2;
        while (t > 0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
            t--;
        }
    }


    protected void pickupExtraOneRingOld() {
        botTop.intakeMotorOn(INTAKE_MOTOR);
        botTop.launchMotorOn(LAUNCH_POWER_POWER_SHOT_FRONT);
        gotoHeading(0);
        botTop.lowerMagazine();
        moveDistanceFromObject(TravelDirection.LEFT, 28.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveForward(25.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveBackward(10.0, DRIVE_TRAIN_TRAVELING_POWER);
        justWait(500);
        botTop.liftMagazine();
        // counter clockwise from 180 is positive
        //        gotoHeading(200);
        // counter clockwise from 180 is positive
//        gotoHeading(198);
        gotoHeading(195);
        // hits the middle one
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
        moveRight(8.0, DRIVE_TRAIN_TRAVELING_POWER);
        gotoHeading(180);
        moveDistanceFromObject(TravelDirection.BACKWARD,2.0, DRIVE_TRAIN_TRAVELING_POWER);
        gotoHeading(180);
        moveDistanceFromObject(TravelDirection.RIGHT,24.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveLeft(20.0, DRIVE_TRAIN_TRAVELING_POWER/1.5);
        gotoHeading(150);
        moveForward(5, DRIVE_TRAIN_TRAVELING_POWER/2.0);
        moveForward(67, DRIVE_TRAIN_TRAVELING_POWER/1.5);
        currentState = STATE_done;
    }


    protected void pickupExtraFourRings() {
        botTop.launchMotorOn(LAUNCH_POWER_POWER_SHOT_FRONT);
        gotoHeading(0);
        botTop.lowerMagazine();
        moveRight(2.0, DRIVE_TRAIN_TRAVELING_POWER);

        // Knock off the stack
        moveForward(16.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveBackward(2.0, DRIVE_TRAIN_TRAVELING_POWER);
        botTop.intakeMotorOn(INTAKE_MOTOR);

        // pick up one ring
        moveForward(4.0, DRIVE_TRAIN_TRAVELING_POWER);

        // @todo added this next line to pick up 2 rings from the stack.  Does not work all the time
        // ******************
//        moveForward(1.0, DRIVE_TRAIN_TRAVELING_POWER/2.0);
        // **********************
        justWait(500);
        moveForward(2.0, DRIVE_TRAIN_TRAVELING_POWER/2.0);

        // move toward the launch line
        moveBackward(12.0, DRIVE_TRAIN_TRAVELING_POWER);
//        moveBackward(15.0, DRIVE_TRAIN_TRAVELING_POWER);
        moveRight(12.0, DRIVE_TRAIN_TRAVELING_POWER);
        // counter clockwise from 180 is positive
        //        gotoHeading(200);
        // counter clockwise from 180 is positive
        gotoHeading(195);
        botTop.liftMagazine();
        justWait(450);
        int t = 2;
        while (t > 0) {
            botTop.extendArm();
            justWait(TIME_TO_EXTEND);
            botTop.retractArm();
            justWait(TIME_TO_RETRACT);
            gotoHeading(188);
            t--;
        }
    }
}
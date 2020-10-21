package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpModesBase;

@Autonomous(name="Test Movements", group="1")
//@Disabled

public class TestMovements extends AutonomousOpModesBase {

    protected static final double DRIVE_TRAIN_TRAVELING_POWER           = 0.2;

    @Override
    public void initAutonomous() {
        IDENTIFICATION_SYSTEM = "NONE";
        DEBUG = true;
        super.initAutonomous();
        botBase.setBling(0.7745); //off
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
            moveSquare();
            gotoHeading(0);
            moveSquare2();
            gotoHeading(0);
            moveTriangle();
            gotoHeading(0);
            break;
        }

        terminateAutonomous();
    }

    protected void moveUsingSensors() {
    }

    protected void moveSquare() {
        moveForward(10, DRIVE_TRAIN_TRAVELING_POWER);
        moveRight(10, DRIVE_TRAIN_TRAVELING_POWER);
        moveBackward(10, DRIVE_TRAIN_TRAVELING_POWER);
        moveLeft(10, DRIVE_TRAIN_TRAVELING_POWER);
    }

    protected void moveSquare2() {
        justWait(500);
        moveToRelative(10, 0, DRIVE_TRAIN_TRAVELING_POWER);
        justWait(500);
        moveToRelative(10, 10, DRIVE_TRAIN_TRAVELING_POWER);
        justWait(500);
        moveToRelative(0, 10, DRIVE_TRAIN_TRAVELING_POWER);
        justWait(500);
        moveToRelative(0, 0, DRIVE_TRAIN_TRAVELING_POWER);
        justWait(500);
    }

    protected void moveTriangle() {
        moveDiagonally(150, 10, DRIVE_TRAIN_TRAVELING_POWER);
        moveDiagonally(270, 10, DRIVE_TRAIN_TRAVELING_POWER);
        moveDiagonally(30, 10, DRIVE_TRAIN_TRAVELING_POWER);
    }

    protected void gotoHeadings() {
        gotoHeading(90);
        justWait(800);
        gotoHeading(180);
        justWait(800);
        gotoHeading(270);
        justWait(800);
        gotoHeading(0);
    }
}
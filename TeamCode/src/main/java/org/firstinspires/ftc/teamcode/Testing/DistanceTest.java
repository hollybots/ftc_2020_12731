package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpModesBase;

@Autonomous(name="Distance Test", group="none")
//@Disabled

public class DistanceTest extends AutonomousOpModesBase {

    @Override
    public void initAutonomous() {

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
        runtime.reset();
        waitForStart();

        /*********************************************
         * GAME IS ON !!
         * *******************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

//            for (p=0.2; p<0.5; p+=0.1) {
                for (int d=2; d<10; d+=2) {
                    moveForward(d, 0.5);
                    justWait(2000);
                }

            for (int d=2; d<10; d+=2) {
                moveBackward(d, 0.5);
                justWait(2000);
            }


//            }
        }
        stopMoving();
    }
}

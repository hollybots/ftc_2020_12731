package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutonomousOpModesBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * This was used in the past when we didn't have an Odometer.
 * We would chart the distance as a function of time.
 * Completely Obsolete
 */

@Autonomous(name="Speed Calibration", group="1")
@Disabled

public class SpeedTest extends AutonomousOpModesBase {

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

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Started!");
        telemetry.update();


        /*********************************************
         * GAME IS ON !!
         * *******************************************/


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double p;
            int t;
            for (p=0.2; p<0.5; p+=0.1) {
                for (t=1000; t<3000; t+=500) {
                    moveForwardByTime(t,  p);
                    gotoHeading(0);
                    stopMoving();
                    justWait(500);
                    moveBackwardByTime(t,  p);
                    justWait(4000);
                }
            }

            justWait(10000);

            for (p=0.5; p<0.8; p+=0.1) {
                for (t=300; t<1300; t+=300) {
                    moveForwardByTime(t,  p);
                    gotoHeading(0);
                    stopMoving();
                    justWait(500);
                    moveBackwardByTime(t,  p);
                    justWait(4000);
                }
            }

            justWait(10000);

            for (p=0.8; p<=1.0; p+=0.1) {
                for (t=200; t<900; t+=200) {
                    moveForwardByTime(t,  p);
                    gotoHeading(0);
                    stopMoving();
                    justWait(500);
                    moveBackwardByTime(t,  p);
                    justWait(4000);
                }
            }
        }
        stopMoving();
    }
}

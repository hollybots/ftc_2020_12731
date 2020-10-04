package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpModesBase;
import org.firstinspires.ftc.teamcode.Components.BotSounds;
import org.firstinspires.ftc.teamcode.Components.FieldPlacement;

@Autonomous(name="Speed Calibration", group="1")
//@Disabled

public class SpeedTest extends AutonomousOpModesBase {

    protected static final int STATE_idle                       = 0;

    protected static final int STATE_moveToStones               = 1;
    protected static final int STATE_scanForStone               = 2;
    protected static final int STATE_alignWithStone             = 3;
    protected static final int STATE_pickUpStone                = 4;
    protected static final int STATE_travelToBuildSite          = 5;
    protected static final int STATE_parkUnderBridge            = 6;

    protected static final int STATE_getCloseEnoughToPickup     = 7;
    protected static final int STATE_dropOffStone               = 8;
    protected static final int STATE_travelHome                 = 9;

    protected static final int STATE_moveToTray                 = 20;
    protected static final int STATE_clampTray                  = 21;
    protected static final int STATE_moveTrayBack               = 22;



    protected static final int STATE_done                       = 50;

    protected int currentState                                  = STATE_idle;

    protected FieldPlacement stoneRelativePlacement             = null;

    // Sounds
    protected BotSounds botSounds = null;

    // alignment
    protected static final double CAMERA_TO_CENTER               = 0.0;

    protected static final double BLING_MODE_CLAMP               = 0.6545;


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

//            for (p=0.2; p<0.5; p+=0.1) {
//                for (t=1000; t<3000; t+=500) {
//                    moveLeftByTime(t,  p);
//                    gotoHeading(0);
//                    stopMoving();
//                    justWait(500);
//                    moveRightByTime(t,  p);
//                    justWait(3000);
//                }
//            }


//
//            for (p=0.5; p<0.8; p+=0.1) {
//                for (t=300; t<1300; t+=300) {
//                    moveLeftByTime(t,  p);
//                    stopMoving();
//                    justWait(500);
//                    moveRightByTime(t,  p);
//                    gotoHeading(0);
//                    justWait(3000);
//                }
//            }

//
//            for (p=0.8; p<=1.0; p+=0.1) {
//                for (t=200; t<900; t+=200) {
//                    moveLeftByTime(t,  p);
//                    stopMoving();
//                    justWait(500);
//                    moveRightByTime(t,  p);
//                    gotoHeading(0);
//                    justWait(5000);
//                }
//            }

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

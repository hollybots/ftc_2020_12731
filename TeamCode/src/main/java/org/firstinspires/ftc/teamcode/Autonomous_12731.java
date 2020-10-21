package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Components.BotSounds;
import org.firstinspires.ftc.teamcode.Components.FieldPlacement;
import org.firstinspires.ftc.teamcode.Components.LedPatterns;

@Autonomous(name="Base Team 12731", group="none")
@Disabled

public class Autonomous_12731 extends AutonomousOpModesBase {

    protected static final int STATE_idle = 0;

    protected static final int STATE_1 = 1;
    protected static final int STATE_2 = 2;
    protected static final int STATE_3 = 4;


    protected static final int STATE_done = 50;

    protected int currentState = STATE_idle;

    protected FieldPlacement stoneRelativePlacement = null;

    // Sounds
    protected BotSounds botSounds = null;

    // alignment camera from center toward the the left is negative (with the selfie side forward)
    protected static final double CAMERA_TO_CENTER = -3;

    protected static final int MAX_CYCLES_FOR_FINDING_STONE = 3;

    protected static final double BLING_MODE_CLAMP = LedPatterns.LED_TEAM_COLORS4;
    protected static final double DISTANCE_TO_STONEWALL = 12.0;


    @Override
    public void initAutonomous() {

        DEBUG = true;
        super.initAutonomous();

        /* **********************************
           LIGHTS
        */
        botBase.setBling(0.7745);
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

            if (currentState == STATE_done) {
                break;
            }

            switch (currentState) {

                case STATE_idle:
                    stopMoving();
                    autonomousIdleTasks();
                    break;

                case STATE_1:
                    taskState_1();
                    break;

                case STATE_2:
                    taskState_2();
                    break;

                case STATE_3:
                    taskState_3();
                    break;

            }
            telemetry.update();
        }

        super.terminateAutonomous();

    }



    protected void taskState_1() {
        currentState = STATE_2;
        return;
    }

    protected void taskState_2() {
        currentState = STATE_3;
        return;
    }

    protected void taskState_3() {
        currentState = STATE_done;
        return;
    }

}
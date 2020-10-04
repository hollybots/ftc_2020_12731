package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Autonomous Sofia", group="none")
@Disabled

public class AutonomousOpMode_sofia extends AutonomousOpModesBase {

    /**
     * All possible states
     */
    protected static final int STATE_idle   = 0;
    protected static final int STATE_1      = 1;
    protected static final int STATE_2      = 2;
    protected static final int STATE_3      = 3;
    protected static final int STATE_done   = 50;


    /**
     * State your Opmode is currently in
     */
    protected int currentState = STATE_idle;

    @Override
    public void initAutonomous() {

        //

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
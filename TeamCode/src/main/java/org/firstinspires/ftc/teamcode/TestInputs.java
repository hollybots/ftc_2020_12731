package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.FieldPlacement;
import org.firstinspires.ftc.teamcode.Components.LedPatterns;

import java.util.List;
import org.firstinspires.ftc.teamcode.Components.RevInputs;

@Autonomous(name="Test input loops", group="none")
//@Disabled

public class TestInputs extends AutonomousOpModesBase {

    final int TEST_CYCLES                                               = 20;   // Number of control cycles to run to determine cycle times.

    protected static final double DRIVE_TRAIN_TRAVELING_POWER           = 0.7;
    protected static final double LAUNCH_POWER                          = 0.7;
    protected static final double LAUNCH_POWER_NONE                     = LAUNCH_POWER;
    protected static final double LAUNCH_POWER_SINGLE                   = LAUNCH_POWER;
    protected static final double LAUNCH_POWER_QUAD                     = LAUNCH_POWER;
    protected static final double LAUNCH_POWER_POWER_SHOT_FRONT         = 0.64;
    protected static final double INTAKE_MOTOR                          = 0.9;

    protected static final double WOBBLE_GOAL_DELIVERY_POWER            = 0.48; // lifting is negative, lowering is positive
    protected static final double TIME_TO_DELIVER                       = 2400;

    protected static final int TIME_TO_EXTEND                           = 300; //ms
    protected static final int TIME_TO_RETRACT                          = 350; //ms

    protected static final int NB_RINGS_POWER_SHOT                      = 1;

    protected static final double LAUNCHER_X                            = 24.0;  // position in inches from the robot
    protected static final double LAUNCHER_Y                            = 0.5;

    // Cycle Times
    double t1 = 0;
    double t2a = 0;
    double t2b = 0;
    double t2c = 0;
    double t2d = 0;
    double t3 = 0;
    double t4 = 0;

    // TSF
    private FieldPlacement ringPlacement        = null;
    protected String ringLabel                  = "None";

    @Override
    public void initAutonomous() {

        DEBUG = true;
        super.initAutonomous();
    }

    @Override
    public void runOpMode() {
        TFOD_MODEL_ASSET                            = "UltimateGoal.tflite";
        TFOD_MODEL_ASSETS_LABEL                     = new String[] {"Quad", "Single"};
        TFOD_TARGET_LABEL                           = "";
        IDENTIFICATION_SYSTEM                       = "TSF"; // can be VUFORIA, TSF, or NONE
        CAMERA_SYSTEM                               = "WEBCAM";  // can be PHONE or WEBCAM
        int cycles;

        initAutonomous();
        searchableTarget.stop();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*********************************************
         * WAIT FOR START
         * *******************************************/

        ElapsedTime timer = new ElapsedTime();
        telemetry.addData(">", "Press play to start tests");
        telemetry.addData(">", "Test results will update for each access method.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        /*****************************************/

        dbugThis("Test 1 is running");
        displayCycleTimes("Bulk Only");
        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            autonomousIdleTasks(RevInputs.BULK);
        }
        // calculate the average cycle time.
        t1 = timer.milliseconds() / cycles;
        dbugThis(String.format("t1 : %.1f", t1));

        /*****************************************/

        dbugThis("Test 2a is running");
        displayCycleTimes("Front Only");
        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            autonomousIdleTasks(RevInputs.RANGE_FRONT);
        }
        // calculate the average cycle time.
        t2a = timer.milliseconds() / cycles;
        dbugThis(String.format("t2a : %.1f", t2a));

        /*****************************************/

        dbugThis("Test 2b is running");
        displayCycleTimes("Back Only");
        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            autonomousIdleTasks(RevInputs.RANGE_REAR);
        }
        // calculate the average cycle time.
        t2b = timer.milliseconds() / cycles;
        dbugThis(String.format("t2b : %.1f", t2b));

        /*****************************************/


        dbugThis("Test 2c is running");
        displayCycleTimes("All Ranges");
        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            autonomousIdleTasks((short) (RevInputs.RANGE_LEFT | RevInputs.RANGE_RIGHT | RevInputs.RANGE_FRONT | RevInputs.RANGE_REAR));
        }
        // calculate the average cycle time.
        t2c = timer.milliseconds() / cycles;
        dbugThis(String.format("t2c : %.1f", t2c));

        /*****************************************/

        dbugThis("Test 2d is running");
        displayCycleTimes("All");
        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            autonomousIdleTasks(RevInputs.ALL);
        }
        // calculate the average cycle time.
        t2d = timer.milliseconds() / cycles;
        dbugThis(String.format("t2d : %.1f", t2d));

        /*****************************************/


        dbugThis("Test 3 is running");
        displayCycleTimes("Bulk Only, bulk disabled");
        List<LynxModule> allHubs = botBase.getAllHubs();
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            autonomousIdleTasks(RevInputs.BULK);
        }
        // calculate the average cycle time.
        t3 = timer.milliseconds() / cycles;
        dbugThis(String.format("t3: %.1f", t3));


        dbugThis("Test 4 is running");
        displayCycleTimes("All, bulk disabled");
        timer.reset();
        cycles = 0;
        while (opModeIsActive() && (cycles++ < TEST_CYCLES)) {
            autonomousIdleTasks(RevInputs.ALL);
        }
        // calculate the average cycle time.
        t4 = timer.milliseconds() / cycles;
        dbugThis(String.format("t4 : %.1f", t4));

        displayCycleTimes("Complete");

        // wait until op-mode is stopped by user, before clearing display.
        while (opModeIsActive()) ;
    }

    // Display three comparison times.
    void displayCycleTimes(String status) {
        telemetry.addData("Testing", status);
        telemetry.addData("Bulkable inputs",    "%5.1f mS/cycle", t1);
        telemetry.addData("1 sensor only",   "%5.1f mS/cycle", t2a);
        telemetry.addData("sensors back (2)",   "%5.1f mS/cycle", t2b);
        telemetry.addData("All range sensors (5)",   "%5.1f mS/cycle", t2c);
        telemetry.addData("All inputs",   "%5.1f mS/cycle", t2d);
        telemetry.addData("Bulk Disabled: Bulkable inputs", "%5.1f mS/cycle", t3);
        telemetry.addData("Bulk Disabled: All inputs", "%5.1f mS/cycle", t4);
        telemetry.update();
    }
}
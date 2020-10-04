/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Testing;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.BotBase;

import java.io.FileWriter;
import java.io.PrintWriter;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Battery Test", group="3")
//@Disabled
public class BatteryTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    protected BotBase botBase;

//    static final double     COUNTS_PER_MOTOR_REV    = 1440;
    static final double     COUNTS_PER_MOTOR_REV    = 383.6;
    PrintWriter log = null;


    @Override
    public void runOpMode() {

        botBase     = new BotBase(hardwareMap);
        try {
            log = new PrintWriter (new FileWriter("/sdcard/battery_test_matrix.txt", true));
            log.print("Time(ms) BatteryVoltage(V)  RPM\n");
            log.close();
        } catch (Exception e) {
            throw new RuntimeException("Cannot write to file", e);
        }

        // start front left propulsion
        botBase.getFrontLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botBase.getFrontLeftDrive().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        int samplingState = 1;
        double  startPosition = 0;
        double  deltaT = 0;
        double  startSampling = 0;

        botBase.getFrontLeftDrive().setPower(0.5);
        double startIntervalBetweenSamplings = runtime.milliseconds();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double now = runtime.milliseconds();

            switch (samplingState) {
                case 0:
                    if ((now - startIntervalBetweenSamplings) > 60000 ) {
                        samplingState = 1;
                        break;
                    }
                    break;

                case 1:
                    startPosition = botBase.getFrontLeftDrive().getCurrentPosition();
                    dbugThis("" + startPosition);
                    startSampling = now;
                    samplingState = 2;
                    break;

                case 2:
                    if ((now - startSampling) > 1000) {
                        double counts = Math.abs(botBase.getFrontLeftDrive().getCurrentPosition() - startPosition);
                        double battery = getBatteryVoltage();
                        double rpm = counts * 60000 / COUNTS_PER_MOTOR_REV  / (now - startSampling);
                        dbugThis(String.format("%.4f, %.4f, %.4f", now, battery, rpm));
                        startIntervalBetweenSamplings = now;
                        samplingState = 0;

                        try {
                            PrintWriter log = new PrintWriter(new FileWriter("/sdcard/battery_test.txt", true));
                            log.printf("%.4f, %.4f, %.4f\n", now, battery, rpm);
                            log.close();
                        } catch (Exception e) {
                            dbugThis(e.getMessage());
                        }

                    }
                    break;
            }
        }
        log.close();
    }


    void dbugThis(String s) {

        Log.d("BATTTEST: ", s);
    }



    // Computes the current battery voltage
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}

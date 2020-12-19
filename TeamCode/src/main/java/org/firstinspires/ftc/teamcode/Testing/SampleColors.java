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

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BotTop;
import org.firstinspires.ftc.teamcode.Components.BotBase;

import java.io.FileWriter;
import java.io.PrintWriter;

/**
 * This test is meant to record the speed output as a function of the battery voltage.
 * It records the points on a sdcard, that we use later to plot a graph
 */
@TeleOp(name="Color Sampler", group="3")
//@Disabled
public class SampleColors extends LinearOpMode {

    static final boolean DEBUG                  = true;
    private ElapsedTime runtime                 = null;
    PrintWriter log                             = null;

    static final int SAMPLING_RATE              = 200; // in ms

    /* Color sensors
     */
    protected ColorSensor bottomColor           = null;

    @Override
    public void runOpMode() {

        runtime = new ElapsedTime();
        try {
            bottomColor = hardwareMap.get(ColorSensor.class, "bottom_color");
        }
        catch (Exception e){
            bottomColor = null;
            dbugThis("Unable to initialize bottom_color");
        }

        try {
            log = new PrintWriter (new FileWriter("/sdcard/color_samples.csv", false));
            log.print("red,green,blue,hue,label\n");
            log.close();
        } catch (Exception e) {
            dbugThis("Cannot write to file");
            throw new RuntimeException("Cannot write to file", e);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double now = runtime.milliseconds();
        double startSampling = now;
        boolean done = false;
        String  label = "none";
        int nbSamples = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !done) {

            // Sampling Blue
            boolean isSamplingBlue              = gamepad1.x;  // a and b are already used by the Robot Controller to select the gamepad
            // Sampling Yellow
            boolean isSamplingYellow            = gamepad1.y;
            // Sampling White
            boolean isSamplingWhite             = gamepad1.a;
            // Sampling Red
            boolean isSamplingRed               = gamepad1.b;

            if (isSamplingBlue) {
                label = "blue";
            }
            else if (isSamplingYellow) {
                label = "yellow";
            }
            else if (isSamplingWhite) {
                label = "white";
            }
            else if (isSamplingRed) {
                label = "red";
            } else {
                label = "none";
            }

            now = runtime.milliseconds();
            // waiting for rmp calculations
            if ((now - startSampling) > SAMPLING_RATE && label != "none") {
                float hsvValues[] = {0F, 0F, 0F};
                if (bottomColor != null) {
                    // convert the RGB values to HSV values.
                    // multiply by the SCALE_FACTOR.
                    // then cast it back to int (SCALE_FACTOR is a double)
                    Color.RGBToHSV((int) (bottomColor.red() * 255),
                            (int) (bottomColor.green() * 255),
                            (int) (bottomColor.blue() * 255),
                            hsvValues);
                }
                dbugThis(String.format("%d,%d,%d,%.3f,%s", bottomColor.red(), bottomColor.blue(), bottomColor.green(), hsvValues[0], label));
                try {
                    PrintWriter log = new PrintWriter(new FileWriter("/sdcard/color_samples.csv", true));
                    log.printf(String.format("%d,%d,%d,%.3f,%s\n", bottomColor.red(), bottomColor.blue(), bottomColor.green(), hsvValues[0], label));
                    nbSamples++;
                    log.close();
                } catch (Exception e) {
                    dbugThis(e.getMessage());
                }
                startSampling = now;
            }
            telemetry.addData("Sampling Color", label);
            telemetry.addData("Number of samples", nbSamples);
            telemetry.update();

        }
        log.close();
    }

    void dbugThis(String s) {

        Log.d("SAMPLER: ", s);
    }
}

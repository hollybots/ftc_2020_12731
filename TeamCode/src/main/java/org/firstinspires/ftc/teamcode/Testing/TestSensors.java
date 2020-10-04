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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOpModesBase;

/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Test Sensors", group="3")
@Disabled
public class TestSensors extends TeleOpModesBase
{
    static final double     AUTONOMOUS_SPEED            = 0.6;
    static final double     COLLISION_DISTANCE          = 6.0;
    static final double     K                           = 0.2;
    private double          theta                       = 0;   // gyro angle.  For field centric autonomous mode we will use this to orient the robot


    // Range Sensors
    ModernRoboticsI2cRangeSensor distanceFront = null;
    ModernRoboticsI2cRangeSensor distanceBack = null;
    ModernRoboticsI2cRangeSensor distanceLeft = null;
    ModernRoboticsI2cRangeSensor distanceRight = null;

    DigitalChannel swingLimitUp = null;
    DigitalChannel swingLimitDown = null;
    DigitalChannel coilLimitUp = null;
    DigitalChannel coilLimitDown = null;
    DigitalChannel backCollision  = null;

    ColorSensor bottomColor = null;
    ColorSensor frontColor = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing the base...");
        super.init();
        telemetry.update();

        /* ***********************************
        RANGE SENSORS
        */
        try{
            distanceFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "front_range_1");
        } catch (Exception e) {
            distanceFront = null;
            dbugThis("Unable to map front_range_1");
        }
        try{
            distanceBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rear_range_1");
        } catch (Exception e) {
            distanceBack = null;
            dbugThis("Unable to map rear_range_1");
        }
        try{
            distanceLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "left_range_1");
        } catch (Exception e) {
            distanceLeft = null;
            dbugThis("Unable to map left_range_1");
        }
        try{
            distanceRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "right_range_1");
        } catch (Exception e) {
            distanceRight = null;
            dbugThis("Unable to map right_range_1");
        }


        /* ************************************
             LIMIT SWITCHES
        */


        /**
         * LIMIT Switches for safety
         */

        try{
            swingLimitUp                         = hardwareMap.get(DigitalChannel.class, "swing_limit_up");
        } catch (Exception e) {
            swingLimitUp = null;
            dbugThis("Unable to map swing_limit_up");
        }

        try{
            swingLimitDown                         = hardwareMap.get(DigitalChannel.class, "swing_limit_down");
        } catch (Exception e) {
            swingLimitDown = null;
            dbugThis("Unable to map swing_limit_down");
        }

        try{
            coilLimitUp                         = hardwareMap.get(DigitalChannel.class, "coil_limit_up");
        } catch (Exception e) {
            coilLimitUp = null;
            dbugThis("Unable to map coil_limit_up");
        }

        try{
            coilLimitDown                         = hardwareMap.get(DigitalChannel.class, "coil_limit_down");
        } catch (Exception e) {
            coilLimitDown = null;
            dbugThis("Unable to map coil_limit_down");
        }

        /**
         * To detect tray
         */
        try{
            backCollision                         = hardwareMap.get(DigitalChannel.class, "back_collision");
        } catch (Exception e) {
            backCollision = null;
            dbugThis("Unable to map back_collision");
        }


        /**
         * COLOR sensors
         */
        try{
            frontColor = hardwareMap.get(ColorSensor.class, "front_color");
        } catch (Exception e) {
            frontColor = null;
            dbugThis("Unable to map front_color");
        }
        try{
            bottomColor = hardwareMap.get(ColorSensor.class, "bottom_color");
        } catch (Exception e) {
            frontColor = null;
            dbugThis("Unable to map bottom_color");
        }

    }



    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("Status", "Make sure motors are free to move");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*
        Read gamepad value
         */
        // ... or for two 2-axis joysticks do this (Halo):
        // push joystick1 forward to go forward
        // push joystick1 to the right to strafe right
        // push joystick2 to the right to rotate clockwise
        double forward      = -gamepad1.right_stick_y;
        double right        = gamepad1.right_stick_x;
        double clockwise    = gamepad1.left_stick_x;


        boolean goingForward = forward > 0;
        boolean goingRight = right > 0;
        boolean goingLeft = right < 0;
        boolean goingBack = forward < 0;

        if ( goingForward ){
            dbugThis("Going forward");
        }
        if ( goingRight ){
            dbugThis("Going Right");
        }
        if ( goingLeft ){
            dbugThis("Going Left");
        }
        if ( goingBack ){
            dbugThis("Going Back");
        }

        if ( goingForward && tooClose(distanceFront) ) {

            dbugThis("Going forward");
            forward = 0;
            right = 0;
        }

        if ( goingRight && tooClose(distanceRight) ) {
            forward = 0;
            right = 0;
        }
        if ( goingLeft && tooClose(distanceLeft) ) {
            right = 0;
        }
        if ( goingBack && tooClose(distanceBack) ) {
            forward = 0;
            right = 0;
        }


        // Now add a tuning constant K for the rotateAutonomous_12731 axis sensitivity.
        // Start with K=0, and increase it very slowly (do not exceed K=1)
        // to find the right value after you’ve got fwd/rev and strafe working:
        clockwise = K*clockwise;

        // if "theta" is measured CLOCKWISE from the zero reference:
        double temp = forward* Math.cos(theta) + right* Math.sin(theta);
        right = -forward* Math.sin(theta) + right* Math.cos(theta);
        forward = temp;

// if "theta" is measured COUNTER-CLOCKWISE from the zero reference:
//        temp = forward*Math.cos(theta) - right*Math.sin(theta);
//        right = forward*Math.sin(theta) + right*Math.cos(theta);
//        forward = temp;


        // Now apply the inverse kinematic tranformation
        // to convert your vehicle motion command
        // to 4 wheel speed commands:
        double front_left = forward + clockwise + right;
        double front_right = forward - clockwise - right;
        double rear_left = forward + clockwise - right;
        double rear_right = forward - clockwise + right;

        // Finally, normalize the wheel speed commands
        // so that no wheel speed command exceeds magnitude of 1:
        double max = Math.abs(front_left);
        if (Math.abs(front_right)>max) {
            max = Math.abs(front_right);
        }
        if (Math.abs(rear_left)>max){
            max = Math.abs(rear_left);
        }
        if (Math.abs(rear_right)>max) {
            max = Math.abs(rear_right);
        }
        if ( max > 1.0 ) {
            front_left /= max;
            front_right /= max;
            rear_left /= max;
            rear_right /= max;
        }


        // Send calculated power to wheels
        botBase.getFrontLeftDrive().setPower(front_left);
        botBase.getFrontRightDrive().setPower(front_right);
        botBase.getRearLeftDrive().setPower(rear_left);
        botBase.getRearRightDrive().setPower(rear_right);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", String.format("Run Time: .2f", runtime.toString()));

        if (distanceFront != null) {
            telemetry.addData("Sensor Front :", String.format("%.2f" , getValidDistance(distanceFront)));
        }

        if (distanceRight != null) {
            telemetry.addData("Sensor Right :", String.format("%.2f" , getValidDistance(distanceRight)));
        }

        if (distanceLeft != null) {
            telemetry.addData("Sensor Left :", String.format("%.2f" , getValidDistance(distanceLeft)));
        }

        if (distanceBack != null) {
            telemetry.addData("Sensor Back :", String.format("%.2f" , getValidDistance(distanceBack)));
        }


        // Show limiy switches states
        if (swingLimitUp != null) {
            telemetry.addData("Arm up : ", "" + swingLimitUp.getState());
        }
        if (swingLimitDown != null) {
            telemetry.addData("Arm down : ", "" + swingLimitDown.getState());
        }
        if (coilLimitUp != null) {
            telemetry.addData("Coil up : ", "" + coilLimitUp.getState());
        }
        if (coilLimitDown != null) {
            telemetry.addData("Coil up : ", "" + coilLimitDown.getState());
        }


        /** Collision back
         *
         */
        if (backCollision != null) {
            telemetry.addData("Collision back : ", "" + backCollision.getState());
        }


        // Show color sensor info
        if (bottomColor != null) {
            float hsvValues[] = {0F, 0F, 0F};
            Color.RGBToHSV((int) (bottomColor.red() * 255),
                    (int) (bottomColor.green() * 255),
                    (int) (bottomColor.blue() * 255),
                    hsvValues);
            telemetry.addData("Alpha", bottomColor.alpha());
            telemetry.addData("Red  ", bottomColor.red());
            telemetry.addData("Green", bottomColor.green());
            telemetry.addData("Blue ", bottomColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            // R 168, 59 59
        }

        // Show color sensor info
        if (frontColor != null) {
            float hsvValues[] = {0F, 0F, 0F};
            Color.RGBToHSV((int) (frontColor.red() * 255),
                    (int) (frontColor.green() * 255),
                    (int) (frontColor.blue() * 255),
                    hsvValues);
            telemetry.addData("Alpha", frontColor.alpha());
            telemetry.addData("Red  ", frontColor.red());
            telemetry.addData("Green", frontColor.green());
            telemetry.addData("Blue ", frontColor.blue());
            telemetry.addData("Hue", hsvValues[0]);

        }

        telemetry.addData("Battery Voltage", String.format("%.2f", getBatteryVoltage()));


            telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }


    protected boolean tooClose(ModernRoboticsI2cRangeSensor sensor) {

        if ( sensor.getDistance(DistanceUnit.INCH) < COLLISION_DISTANCE ) {
            dbugThis(".and too close");
            return true;
        }
        return false;
    }


    /**
     * Checks the state of the limit switch for backCollision limit switch.
     * If there is no such limit switch, it returns false.
     *
     * @return
     */
    public boolean isCollidingBack() {

        if (backCollision == null) {
            return false;
        }

        return !(backCollision.getState() == true );
    }


    public double getValidDistance(ModernRoboticsI2cRangeSensor sensor) {

        if (sensor == null) {
            return 0;
        }

        double validDistance = sensor.getDistance(DistanceUnit.INCH);
        if ( validDistance  == DistanceSensor.distanceOutOfRange )  {
            return 0;
        }

        return validDistance;
    }
}

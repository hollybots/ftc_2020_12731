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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY"S PATENT RIGHTS ARE GRANTED BY THIS
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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;


/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="IO Tests", group="1")
//@Disabled
public class IoTests extends OpMode
{

    protected static final int state_IDLE                   = 0;
    protected static final int state_SELECT_DEVICE          = 1;
    protected static final int state_SERVO_SELECTION        = 2;
    protected static final int state_I2C_SELECTION          = 3;
    protected static final int state_SERVO_CONTROL          = 4;
    protected static final int state_DINPUT_SELECTION       = 5;


    /****
     * DEVICES
     */
    String[] devices = {"NONE", "DCMOTOR", "SERVO", "I2C", "DIGITAL INPUT"};

    /***
     * OUTPUTS (will be connected to the gamepad)
     */

    // DC Motors
    List<DcMotor> motors;

    // Servo
    List<Servo> servos;

    /**
     * INPUTS
     */

    // Range Sensors
    List<ModernRoboticsI2cRangeSensor> i2cs;

    // Digital Inputs
    List<DigitalChannel> dinputs;


    /**
     * State variables
     */

    private int current_state = state_IDLE;
    // General selection states
    private Boolean left_bumper_was_down = false;
    private Boolean right_bumper_was_down = false;

    private Boolean previous_gamepad_x = false;
    private Boolean previous_gamepad_y = false;

    // Devices selection
    private int selecting_device_mode = 0;
    private int current_device_selection = 0;

    // Servo selection
    private double servo_current_value = 0.0;
    private int current_servo_selection = 0;

    // I2c Selection
    private int current_i2c_selection = 0;
    private double current_distance = -1.0;

    // Digital input selection
    private int current_dinput_selection = 0;
    private Boolean current_input_state = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {


        /*********************************************
         * Harware map
         */
        servos       = new ArrayList();
        for (int i=0; i<10; i++) {
            int hub = (i / 5) + 1;
            try {
                servos.add(hardwareMap.get(Servo.class, "servo_" + hub + "_"  + i));
            }
            catch (Exception e) {
                Log.d("TEST_IO: ", "Cannot intialize servo_" + hub + "_"  + i);
            }
        }

        motors       = new ArrayList();
        for (int i=0; i<6; i++) {
            int hub = (i / 3) + 1;
            try {
                motors.add(hardwareMap.get(DcMotor.class, "motor_" + hub + "_" + i));
                motors.get(i).setDirection(DcMotor.Direction.FORWARD);
                motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            catch (Exception e) {
                Log.d("TEST_IO: ", "Cannot intialize motor_" + hub + "_" + i);
            }
        }

        i2cs       = new ArrayList();
        for (int i=0; i<6; i++) {
            int hub = (i / 3) + 1;
            try {
                i2cs.add(hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "i2c_" + hub + "_" + i));
            } catch (Exception e) {
                Log.d("TEST_IO: ", "Cannot intialize i2c_" + hub + "_" + i);
            }
        }

        dinputs       = new ArrayList();
        for (int i=0; i<6; i++) {
            int hub = (i / 3) + 1;
            try {
                dinputs.add(hardwareMap.get(DigitalChannel.class, "dinput_" + hub + "_" + i));
                dinputs.get(i).setMode(DigitalChannel.Mode.INPUT);
            } catch (Exception e) {
                Log.d("TEST_IO: ", "Cannot intialize dinput_" + hub + "_" + i);
            }
        }
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        switch (current_state) {

            case state_IDLE:
                i2cDisplay();
                dInputDisplay();
                outputPropulsion();

                if (gamepad1.x) {
                    previous_gamepad_x = true;
                }
                else if (!gamepad1.x && previous_gamepad_x) {
                    Log.d("TEST_IO: ", "================= Entering DEVICE SELECTION mode. Use right and left bumper to select device ============");
                    current_state = state_SELECT_DEVICE;
                    previous_gamepad_x = false;
                }
                return;

            case state_SELECT_DEVICE:
                deviceSelection();
                if (gamepad1.y) {
                    previous_gamepad_y = true;
                }
                else if (!gamepad1.y && previous_gamepad_y) {

                    if (devices[current_device_selection] == "SERVO") {
                        Log.d("TEST_IO: ", "================= Entering SELECTING servo mode ================ ");
                        Log.d("TEST_IO: ", "Current Servo: " + current_servo_selection);
                        current_state = state_SERVO_SELECTION;
                    }
                    else if (devices[current_device_selection] == "I2C") {
                        Log.d("TEST_IO: ", "================= Entering SELECTING i2c mode ============");
                        Log.d("TEST_IO: ", "Current I2C: " + current_i2c_selection);
                        current_state = state_I2C_SELECTION;
                    }
                    else if (devices[current_device_selection] ==  "DIGITAL INPUT") {
                        Log.d("TEST_IO: ", "================= Entering SELECTING Digital Input mode ============");
                        Log.d("TEST_IO: ", "Current Digital Input: " + current_dinput_selection);
                        current_state = state_DINPUT_SELECTION;
                    }
                    else {
                        current_state = state_IDLE;
                    }
                    previous_gamepad_y = false;
                }
                return;

            case state_SERVO_SELECTION:
                servoSelection();
                if (gamepad1.y) {
                    previous_gamepad_y = true;
                }
                else if (!gamepad1.y && previous_gamepad_y) {

                    if (devices[current_device_selection] == "SERVO") {
                        Log.d("TEST_IO: ", "================= MOVING servo " + current_servo_selection + " ============");
                        current_state = state_SERVO_CONTROL;
                    }
                    else {
                        current_state = state_IDLE;
                    }
                    previous_gamepad_y = false;
                }
                return;

            case state_SERVO_CONTROL:
                servoControl();
                if (gamepad1.y) {
                    previous_gamepad_y = true;
                }
                else if (!gamepad1.y && previous_gamepad_y) {
                    current_state = state_IDLE;
                    previous_gamepad_y = false;
                }
                return;

            case state_I2C_SELECTION:
                i2cSelection();
                if (gamepad1.y) {
                    previous_gamepad_y = true;
                }
                else if (!gamepad1.y && previous_gamepad_y) {
                    current_state = state_IDLE;
                    previous_gamepad_y = false;
                }
                return;

            case state_DINPUT_SELECTION:
                dInputSelection();
                if (gamepad1.y) {
                    previous_gamepad_y = true;
                }
                else if (!gamepad1.y && previous_gamepad_y) {
                    current_state = state_IDLE;
                    previous_gamepad_y = false;
                }
                return;


        }

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }

    /**
     * Handles the selection of a device using left & right bumpers
     * returns: If the program is in device section mode
     * @return
     */
    private void deviceSelection()
    {
        /**
         * In this mode we are using the bumpers to select a device
         */
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            left_bumper_was_down = true;
            return;
        }
        if (!gamepad1.left_bumper && left_bumper_was_down) {
            if (current_device_selection < 1) {
                current_device_selection = devices.length-1;
            } else {
                current_device_selection--;
            }
            Log.d("TEST_IO: ", "Current Device: " + devices[current_device_selection]);
            left_bumper_was_down = false;
            return;
        }
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            right_bumper_was_down = true;
            return;
        }
        if (right_bumper_was_down && !gamepad1.right_bumper) {
            if (current_device_selection > devices.length-2) {
                current_device_selection = 0;
            } else {
                current_device_selection++;
            }
            Log.d("TEST_IO: ", "Current Device: " + devices[current_device_selection]);
            right_bumper_was_down = false;
            return;
        }
    }


    /***
     * Process of selecting and controlling a servo
     */
    private void servoSelection()
    {

        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            left_bumper_was_down = true;
            return;
        }
        if (!gamepad1.left_bumper && left_bumper_was_down) {
            if (current_servo_selection < 1) {
                current_servo_selection = servos.size()-1;
            } else {
                current_servo_selection--;
            }
            Log.d("TEST_IO: ", "Current Servo: " + current_servo_selection);
            left_bumper_was_down = false;
            return;
        }
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            right_bumper_was_down = true;
            return;
        }
        if (right_bumper_was_down && !gamepad1.right_bumper) {
            if (current_servo_selection > servos.size()-2) {
                current_servo_selection = 0;
            } else {
                current_servo_selection++;
            }
            Log.d("TEST_IO: ", "Current Servo: " + current_servo_selection);
            right_bumper_was_down = false;
            return;
        }
        return;
    }


    /***
     * Process of controlling a servo
     */
    private void servoControl()
    {
        // servo are incremented by 0.1 each time the bumper button is pressed right -> + left -> -
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            left_bumper_was_down = true;
            return;
        }

        if (left_bumper_was_down && !gamepad1.left_bumper) {
            servo_current_value = Math.max(0.0, servo_current_value - 0.1);
            servos.get(current_servo_selection).setPosition(servo_current_value);
            Log.d("TEST_IO: ", String.format("Setting Servo %d to %.2f", current_servo_selection, servo_current_value ));
            left_bumper_was_down = false;
            return;
        }

        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            right_bumper_was_down = true;
            return;
        }

        if (right_bumper_was_down && !gamepad1.right_bumper) {
            servo_current_value = Math.min(1.0, servo_current_value + 0.1);
            servos.get(current_servo_selection).setPosition(servo_current_value);
            Log.d("TEST_IO: ", String.format("Setting Servo %d to %.2f", current_servo_selection, servo_current_value ));
            right_bumper_was_down = false;
            return;
        }
        return;
    }


    /**"
     * Process of selecting a I2c device for monitoring
     */
    private void i2cSelection()
    {

        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            left_bumper_was_down = true;
            return;
        }
        if (!gamepad1.left_bumper && left_bumper_was_down) {
            if (current_i2c_selection < 1) {
                current_i2c_selection = i2cs.size()-1;
            } else {
                current_i2c_selection--;
            }
            Log.d("TEST_IO: ", "Current I2C: " + current_i2c_selection);
            left_bumper_was_down = false;
            return;
        }
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            right_bumper_was_down = true;
            return;
        }
        if (right_bumper_was_down && !gamepad1.right_bumper) {
            if (current_i2c_selection > i2cs.size()-2) {
                current_i2c_selection = 0;
            } else {
                current_i2c_selection++;
            }
            Log.d("TEST_IO: ", "Current I2C: " + current_i2c_selection);
            right_bumper_was_down = false;
            return;
        }

        return;
    }


    /**"
     * Process of selecting a I2c device for monitoring
     */
    private void dInputSelection()
    {
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            left_bumper_was_down = true;
            return;
        }
        if (!gamepad1.left_bumper && left_bumper_was_down) {
            if (current_dinput_selection < 1) {
                current_dinput_selection = dinputs.size()-1;
            } else {
                current_dinput_selection--;
            }
            Log.d("TEST_IO: ", "Current Digital Input: " + current_dinput_selection);
            left_bumper_was_down = false;
            return;
        }
        if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            right_bumper_was_down = true;
            return;
        }
        if (right_bumper_was_down && !gamepad1.right_bumper) {
            if (current_dinput_selection > dinputs.size()-2) {
                current_dinput_selection = 0;
            } else {
                current_dinput_selection++;
            }
            Log.d("TEST_IO: ", "Current Digital Input: " + current_dinput_selection);
            right_bumper_was_down = false;
            return;
        }

        return;
    }


    private void i2cDisplay() {
        double distance = i2cs.get(current_i2c_selection).getDistance(DistanceUnit.INCH);
        if (distance != current_distance) {
            Log.d("TEST_IO: ", String.format("Distance sensor %d: %.4f", current_i2c_selection, distance));
            current_distance = distance;
        }
    }

    private void dInputDisplay() {
        Boolean state = dinputs.get(current_dinput_selection).getState();
        if (state != current_input_state) {
            Log.d("TEST_IO: ", String.format("Digital Input #%d state: %b", current_dinput_selection, state));
            current_input_state = state;
        }
    }


    private void outputPropulsion() {
        if (selecting_device_mode == 0) {

            double motor0_command                  = -gamepad1.right_stick_y;
            double motor1_command                   = gamepad1.right_stick_x;
            double motor2_command                  = -gamepad1.left_stick_y;
            double motor3_command                   = gamepad1.left_stick_x;

            // Send calculated power to wheels
            motors.get(0).setPower(motor0_command);
            motors.get(1).setPower(motor1_command);
            motors.get(2).setPower(motor2_command);
            motors.get(3).setPower(motor3_command);
        }
    }

}

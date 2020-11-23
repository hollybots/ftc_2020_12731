package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.BotBase;
import org.firstinspires.ftc.teamcode.Components.TravelDirection;
import org.firstinspires.ftc.teamcode.Components.WheelPower;

/**
 * Teleop mode.
 */
@TeleOp(name="TeleOp Base class", group="none")
@Disabled
public class TeleOpModesBase extends OpMode {

    private static final boolean DEBUG      = true;
    private static final double K           = 1.0;
    private static final double theta       = 0;   // gyro angle.  For field centric autonomous mode we will use this to orient the robot

    protected BotBase botBase = null;
    protected BotTop botTop = null;

    // State variable
    int currentRampNumberForward = 20;  // index of 0 in the ramp
    int currentRampNumberRight = 20;  // index of 0 in the ramp
    double previousPowerForward = 0;
    double previousPowerRight = 0;


    // Timekeeper OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();


    /**
     * Initializes the robot.
     * Called once before the match when the "Init" button is pressed.
     */
    @Override
    public void init() {
        botBase = new BotBase(hardwareMap);
        botTop = new BotTop(hardwareMap);
    }

    /**
     * Main loop function.
     * Called repeatedly during the match after the "Play" button is pressed.
     */
    @Override
    public void loop() {
    }


    /**
     * Stops the robot.
     * Called once at the end of the match when time runs out.
     */
    @Override
    public void stop() {
        botBase.stop();
    }

    protected void dbugThis(String s) {
        if (DEBUG == true) {
            Log.d("TELEOP: ", s);
        }
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

    /**
     * Takes a rotation, from the rotation joystick, a forward and right value from the translation joystick,
     * will calculate the power on each mechanum wheels
     *
     * @param clockwise
     * @param forward
     * @param right
     * @return
     */
    protected WheelPower calcWheelPower(double clockwise, double forward, double right) {

        WheelPower wheels = new WheelPower();

        // Now add a tuning constant K for the rotate axis sensitivity.
        // Start with K=0, and increase it very slowly (do not exceed K=1)
        // to find the right value after you’ve got fwd/rev and strafe working:
        clockwise = K * clockwise;

        // if "theta" is measured CLOCKWISE from the zero reference:
        double temp = forward * Math.cos(theta) + right * Math.sin(theta);
        right = -forward * Math.sin(theta) + right * Math.cos(theta);
        forward = temp;

        // Now apply the inverse kinematic tranformation
        // to convert your vehicle motion command
        // to 4 wheel speed command:
        wheels.front_left = forward + clockwise + right;
        wheels.front_right = forward - clockwise - right;
        wheels.rear_left = forward + clockwise - right;
        wheels.rear_right = forward - clockwise + right;

        wheels.normalize();
        return wheels;
    }

    /**
     * @param which
     * @param setPoint
     * @return
     * @todo: Using the IMU asinput, limit the acceleration so the robot doesn't skid
     */
    private double rampUp(TravelDirection which, double setPoint) {
        if (which == TravelDirection.FORWARD) {
        }
        return 0;
    }
}


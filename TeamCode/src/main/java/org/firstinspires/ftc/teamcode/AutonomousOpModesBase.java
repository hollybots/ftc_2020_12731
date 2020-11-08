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

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Components.BotBase;
import org.firstinspires.ftc.teamcode.Components.FieldPlacement;
import org.firstinspires.ftc.teamcode.Components.ObjectIdentificationInterface;
import org.firstinspires.ftc.teamcode.Components.TensorFlowObjectIdentification;
import org.firstinspires.ftc.teamcode.Components.TravelDirection;
import org.firstinspires.ftc.teamcode.Components.VuMarkIdentification;


/**
 * This file contains the base class definition for all Autonomous OpModes.
 */

@Autonomous(name="Autonomous Base Class", group="none")
@Disabled
public class AutonomousOpModesBase extends LinearOpMode {

    protected static final double CLOSE_ENOUGH_X                = 1.0;
    protected static final double CLOSE_ENOUGH_Y                = 1.0;


    // the  can always be overriden in the extended class

    protected String IDENTIFICATION_SYSTEM         = "VUFORIA"; // can be VUFORIA, TSF, or NONE
    protected String CAMERA_SYSTEM                 = "PHONE";  // can be PHONE or WEBCAM

    // Will dump debug information in the LogCat if true
    protected boolean DEBUG                                     = false;

    // Needed for VUFORIA Vumark Identification
    protected String TRACKABLE_ASSET_NAME                       = "Skystone";
    protected String TRACKABLE_NAME                             = "Sky Stone";  // Can be anyting
    protected int TRACKABLE_INDEX                               = 0;


    // Needed for TENSORFLOW object Identification
    protected String TFOD_MODEL_ASSET                           = "Skystone.tflite";
    protected String [] TFOD_MODEL_ASSETS_LABEL                 = {"Stone", "Skystone"};
    protected String TFOD_TARGET_LABEL                          = "Skystone";

    /**
     * FIELD CONSTANT
     */
    static final double FIELD_WIDTH                     = 48.0;
    static final double FIELD_LENGTH                    = 96.0;

    /**
     * PROPULSION CONSTANTS
     */
    static final double DRIVE_SPEED                     = 0.8;
    static final double TURNING_SPEED                   = 0.5;


    static final double     HEADING_THRESHOLD       = 1.0 ;      // As tight as we can make it with an integer gyro
    /***  IMPORTANT NOTE IF YOU DONT WANT TO GET STUCK in an infinite loop while turning:
     P_TURN_COEFF * TURNING_SPEED must be > 0.1
     ************************************************************************* */
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    static final double K                               = 1.17396293; // constant that maps change in voltage to change in RPM

    /* Propulsion and basic hardware
     */
    public BotBase botBase                                      = null;
    public BotTop botTop                                        = null;


    /* Object detection sub system
     */
    protected ObjectIdentificationInterface searchableTarget      = null;


    /* IMU
     */
    protected BNO055IMU gyro = null;

    /* Color sensors
    */
    protected ColorSensor bottomColor                                     = null;

    /**
     * For state management
     */
    // stall probability (0-5 5:stalled)
    int stallProbability                        = 0;

    // Robot placement at all time in Field Coordinates [x:{-F/2, F/2} y: {-F/2,F/2}]
    FieldPlacement botCurrentPlacement          = null;

    // Previous Robot placement in Field Coordinates [x:{-F/2, F/2} y: {-F/2,F/2}]
    FieldPlacement botPreviousPlacement         = null;

    // current direction of displacement
    TravelDirection propulsionDirection         = TravelDirection.IDLE;

    // Timekeeper OpMode members.
    protected ElapsedTime runtime = new ElapsedTime();

    /***
     * Needs to be overriden/implemented in the derived class
     */
//    @Override
    public void runOpMode() {}


    /**
     * Inits the the automomous mode
     * 1) imports the hardware
     * 2) Initialize the hardware
     */
    protected void initAutonomous() {

        botBase     = new BotBase(hardwareMap);
        botTop      = new BotTop(hardwareMap);

        /*********************       GYRO        *********************** */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn"t actually
        // provide positional information.
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gyroParameters.loggingEnabled      = true;
        gyroParameters.loggingTag          = "IMU";
        gyroParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(gyroParameters);


        /* ************************************
            OBJECT IDENTIFICATION SYSTEM
         */

        if (IDENTIFICATION_SYSTEM == "VUFORIA") {
            searchableTarget  = new VuMarkIdentification(
                hardwareMap,
                telemetry,
                TRACKABLE_ASSET_NAME,
                TRACKABLE_NAME,
                TRACKABLE_INDEX,
                CAMERA_SYSTEM,
                this.DEBUG
            );
        }

        else if (IDENTIFICATION_SYSTEM == "TSF") {
            searchableTarget  = new TensorFlowObjectIdentification(
                hardwareMap,
                telemetry,
                TFOD_MODEL_ASSET,
                TFOD_MODEL_ASSETS_LABEL,
                TFOD_TARGET_LABEL,
                CAMERA_SYSTEM,
                this.DEBUG
            );
        }


        /* ************************************
            LINE DETECTION @todo move to its own component
         */
        try {
            bottomColor = hardwareMap.get(ColorSensor.class, "bottom_color");
        }
        catch (Exception e){
            bottomColor = null;
            dbugThis("Unable to initialize bottom_color");
        }

        telemetry.addData("Status", "Robot Initialized");
        telemetry.update();
    }


    /**
     * This free the resources and objects created for this opmode.  Must be called when exiting the main loop
     */
    protected void terminateAutonomous()
    {
        if (searchableTarget != null) {
            searchableTarget.stop();
        }
        stopMoving();
    }


    /**
     * this function rotates the robot by adding the angle passed in parameter to the current heading.
     *
     * @param angle:    degrees to add to the current heading
     */
    protected void turn(double angle)
    {

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        double finalTheta = actualAngle + angle;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !_onHeading(TURNING_SPEED, finalTheta, P_TURN_COEFF)) {

            autonomousIdleTasks();
        }

        justWait(0.5);
        stopMoving();
    }


    /**
     * This function stops all drive train motors simultaneously
     */
    protected void stopMoving() {
        botBase.stop();
        propulsionDirection = TravelDirection.IDLE;
        return;
    }


    /**
     * Use the Gyro to turn (rotate) the robot until heading is equal to the angle passed in parameter.
     * When the games starts, the heading is reset to 0.  This means that all headings are relative to
     * the robot's initial heading.
     * This is why it is a good practice to align the robot with the Y axis of the field when the game start.
     *
     * @param angle         : Final heading in degrees
     */
    protected void gotoHeading(double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !_onHeading(TURNING_SPEED, angle, P_TURN_COEFF)) {
            autonomousIdleTasks();
        }
        stopMoving();
        return;
    }


    /**
     * This method uses to Odometer to move the robot to the given targetX and targetY parameters using power.
     *
     * TargetX and targetY are absolute values given in inches in a robot centric coordinates system:
     * The Y axis is always pointing in the same direction as the robot's heading.
     *
     * Because targetX and targetY are absolute values, the caller must be aware of the current position of the robot.
     *
     * See moveDiagonally() in order to move the robot at a certain angle.
     * See moveToRelative() in order to move the robot to a specific robot centric coordinate.
     *
     * @param targetX in inches
     * @param targetY in inches
     * @param power in fraction
     */
    private void _moveTo(double targetX, double targetY, double power) {
        // cannot use this function without Odometry
        if (!botBase.hasOdometry()) {
            return;
        }

        while (opModeIsActive()) {

            autonomousIdleTasks();
            double x = botBase.odometer.getCurrentXPos();
            double y = botBase.odometer.getCurrentYPos();
            double deltaX = targetX - x;
            double deltaY = targetY - y;

            //recalculate the angle
            double theta = Math.atan2(deltaX, deltaY);
            powerPropulsionAtAngle(theta, power * 1.5);

            if (Math.abs(targetX - x) < CLOSE_ENOUGH_X && Math.abs(targetY - y) < CLOSE_ENOUGH_Y) {
                break;
            }
        }
        stopMoving();
        return;
    }


    /**
     * This method uses to Odometer to move the robot to the given targetX and targetY parameters using power.
     *
     * TargetX and targetY are given in inches in a robot centric coordinates system.
     * The Y axis is always pointing in the same direction as the robot's heading.  The origin of this coordinate system is
     * always the current position of the robot.
     *
     * @param targetX in inches
     * @param targetY in inches
     * @param power in fraction
     */
    public void moveToRelative(double targetX, double targetY, double power) {
        // cannot use this function without Odometry
        if (!botBase.hasOdometry()) {
            return;
        }

        botBase.odometer.resetToZero();
        _moveTo(targetX, targetY, power);
    }


    /**
     * This function moves the robot at the given angle until it reaches the final destination.
     * PLEASE NOTE that the heading of the robot does not change.  The robot will be straffing.
     *
     * angleInDegrees is ALWAYS the angle in degrees between the desired direction of the movement and the robot's heading.
     *
     * @param angleInDegrees
     */
    protected void moveDiagonally(double angleInDegrees, double distance, double power)
    {
        // cannot use this function without Odometry
        if (!botBase.hasOdometry()) {
            return;
        }
        angleInDegrees = angleInDegrees % 360;
        if (angleInDegrees < 0) {
            angleInDegrees = 360 + angleInDegrees;
        }

        if (angleInDegrees == 0.0) {
            move(TravelDirection.FORWARD, distance, power, false);
            return;
        }
        if (angleInDegrees == 90.0) {
            move(TravelDirection.RIGHT, distance, power, false);
            return;
        }
        if (angleInDegrees == 180.0) {
            move(TravelDirection.BACKWARD, distance, power, false);
            return;
        }
        if (angleInDegrees == 270.0) {
            move(TravelDirection.LEFT, distance, power, false);
            return;
        }

        double angleInRadians = angleInDegrees * Math.PI / 180.0;
        double limitX = botBase.odometer.getCurrentXPos() + distance * Math.sin(angleInRadians);
        double limitY = botBase.odometer.getCurrentYPos() + distance * Math.cos(angleInRadians);

        _moveTo(limitX, limitY, power);
        return;
    }


    /**
     * Robot centric. Moves Right for the given distance, and using the given power
     * @param distance: in inches
     * @param power: motor power
     */
    protected void moveRight(double distance, double power)
    {
        if (botBase.hasOdometry()) {
            move(TravelDirection.RIGHT, distance, power, false);
        } else {
            moveByTime(TravelDirection.RIGHT, timeToMoveInMs(TravelDirection.RIGHT, power, distance), power, false);
        }
    }

    /**
     * Robot centric. Moves Left for the given distance, and using the given power
     * @param distance: in inches
     * @param power: motor power
     */
    protected void moveLeft(double distance, double power)
    {
        if (botBase.hasOdometry()) {
            move(TravelDirection.LEFT, distance, power, false);
        } else {
            moveByTime(TravelDirection.LEFT, timeToMoveInMs(TravelDirection.LEFT, power, distance), power, false);
        }
    }

    /**
     * Robot centric. Moves Backward for the given distance, and using the given power
     * @param distance: in inches
     * @param power: motor power
     */
    protected void moveBackward(double distance, double power)
    {
        if (botBase.hasOdometry()) {
            move(TravelDirection.BACKWARD, distance, power, false);
        } else {
            moveByTime(TravelDirection.BACKWARD, timeToMoveInMs(TravelDirection.BACKWARD, power, distance), power, false);
        }
    }

    /**
     * Robot centric. Moves Forward for the given distance, and using the given power
     * @param distance: in inches
     * @param power: motor power
     *
     * @param distance: in inches
     */
    protected void moveForward(double distance, double power)
    {
        if (botBase.hasOdometry()) {
            move(TravelDirection.FORWARD, distance, power, false);
        } else {
            moveByTime(TravelDirection.FORWARD, timeToMoveInMs(TravelDirection.FORWARD, power, distance), power, false);
        }
    }


    /**
     * Robot centric. This function moves the robot at the given angle for a certain amount of time.
     * PLEASE NOTE that the heading of the robot does not change.  The robot will be straffing and moving at the
     * same time.
     *
     * angleInDegrees is ALWAYS the angle between the desired direction and the direction the robot is facing.
     *
     * @param angleInDegrees
     */
    private void moveDiagonallyByTime(double angleInDegrees, double ms, double power)
    {
        double angleInRadians = angleInDegrees * Math.PI / 180.0;
        double limit = runtime.milliseconds() + ms;
        double now;

        powerPropulsionAtAngle(angleInRadians, power);

        while (opModeIsActive() &&
            (now = runtime.milliseconds()) < limit) {
            autonomousIdleTasks();
        }
        stopMoving();
        return;
    }


    /**
     * Robot centric. Powers up the propulsion to move Right for a period of time, and at a given power
     * @param ms: in milliseconds
     * @param power: motor power
     *
     */
    protected void moveRightByTime(int ms, double power)
    {
        moveByTime(TravelDirection.RIGHT, ms, power, false);
    }

    /**
     * Robot centric. Powers up the propulsion to move Left for a period of time, and at a given power
     * @param ms: in milliseconds
     * @param power: motor power
     *
     */
    protected void moveLeftByTime(int ms, double power)
    {
        moveByTime(TravelDirection.LEFT, ms, power, false);
    }

    /**
     * Robot centric. Powers up the propulsion to move Back for a period of time, and at a given power
     *
     * @param ms: in milliseconds
     * @param power: motor power
     *
     */
    protected void moveBackwardByTime(int ms, double power)
    {
        moveByTime(TravelDirection.BACKWARD, ms, power,false);
    }

    /**
     * Robot centric. Powers up the propulsion to move Forward for a period of time, and at a given power
     *
     * @param ms: in milliseconds
     * @param power: motor power
     *
     */
    protected void moveForwardByTime(int ms, double power)
    {
        moveByTime(TravelDirection.FORWARD, ms, power,false);
    }



    /**
     * Robot centric. Powers up the propulsion to move Right until it senses a color from a sensor
     * @param color
     * @param power
     */
    protected void moveRightToColor(int color, double power) {

        if ( getValidColor(bottomColor) == color ) {
            return;
        }

        powerPropulsion(TravelDirection.RIGHT, power);

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.RIGHT) &&
            getValidColor(bottomColor) != color
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }



    /**
     * Robot centric. Powers up the propulsion to move Left until it senses a color from a sensor
     * @param color
     * @param power
     */
    protected void moveLeftToColor(int color, double power) {

        if ( getValidColor(bottomColor) == color ) {
            return;
        }

        powerPropulsion(TravelDirection.LEFT, power);

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.LEFT) &&
            getValidColor(bottomColor) != color
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }



    /**
     * Robot centric. Powers up the propulsion to move Forward until it senses a color from a sensor
     * @param color
     * @param power
     */
    protected void moveForwardToColor(int color, double power) {

        if ( getValidColor(bottomColor) == color ) {
            return;
        }

        powerPropulsion(TravelDirection.FORWARD, power);

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.FORWARD) &&
            getValidColor(bottomColor) != color
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }



    /**
     * Robot centric. Powers up the propulsion to move Backward until it senses a color from a sensor
     * @param color
     * @param power
     */
    protected void moveBackwardToColor(int color, double power) {

        if (getValidColor(bottomColor) == color ) {
            return;
        }

        powerPropulsion(TravelDirection.BACKWARD, power);

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.BACKWARD) &&
            getValidColor(bottomColor) != color
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     * Robot centric. Powers up the propulsion to move Left until it is x inches from an object
     * Please NOTE that The Range Sensor combines ultrasonic and optical measuring elements to obtain a reading between
     * 1cm and 255cm.
     * ANYTHING outside of that rance cannot be evaluated propaerly with this sensor
     * @param x
     * @param ms
     * @param power
     */
    public void moveXInchesFromLeftObject(double x, double ms, double power) {

        if (botBase.hasSensorPositioningLeft() && getDistance(TravelDirection.LEFT) < x ) {
            return;
        }

        powerPropulsion(TravelDirection.LEFT, power);
        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.LEFT) &&
            runtime.milliseconds() < limit &&
            (!isValidDistance(TravelDirection.LEFT) || isValidDistance(TravelDirection.LEFT) && getDistance(TravelDirection.LEFT) > x)
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     * Robot centric. Powers up the propulsion to move Right until it is x inches from an object
     * Please NOTE that The Range Sensor combines ultrasonic and optical measuring elements to obtain a reading between
     * 1cm and 255cm.
     * ANYTHING outside of that rance cannot be evaluated propaerly with this sensor
     * @param x
     * @param ms
     * @param power
     */
    public void moveXInchesFromRightObject(double x, double ms, double power) {

        if (botBase.hasSensorPositioningRight() && getDistance(TravelDirection.RIGHT) < x ) {
            return;
        }

        powerPropulsion(TravelDirection.RIGHT, power);
        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.RIGHT) &&
            runtime.milliseconds() < limit &&
            (!isValidDistance(TravelDirection.RIGHT) || isValidDistance(TravelDirection.RIGHT) && getDistance(TravelDirection.RIGHT) > x)
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     * Robot centric. Powers up the propulsion to move Forward until it is x inches from an object
     * Please NOTE that The Range Sensor combines ultrasonic and optical measuring elements to obtain a reading between
     * 1cm and 255cm.
     * ANYTHING outside of that rance cannot be evaluated propaerly with this sensor
     *
     * @param x
     * @param ms
     * @param power
     */
    public void moveXInchesFromFrontObject(double x, double ms, double power) {

        if (botBase.hasSensorPositioningFront() && getDistance(TravelDirection.FORWARD) < x ) {
            return;
        }

        powerPropulsion(TravelDirection.FORWARD, power);
        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.FORWARD) &&
            runtime.milliseconds() < limit &&
            (!isValidDistance(TravelDirection.FORWARD) || isValidDistance(TravelDirection.FORWARD) && getDistance(TravelDirection.FORWARD) > x)
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }


    /**
     * Powers up the propulsion to move Back until it is x inches from an object
     * Please NOTE that The Range Sensor combines ultrasonic and optical measuring elements to obtain a reading between
     * 1cm and 255cm.
     * ANYTHING outside of that rance cannot be evaluated propaerly with this sensor
     *
     * @param x
     * @param ms
     * @param power
     */
    public void moveXInchesFromBackObject(double x, double ms, double power) {

        if (botBase.hasSensorPositioningBack() && getDistance(TravelDirection.BACKWARD) < x ) {
            return;
        }

        powerPropulsion(TravelDirection.BACKWARD, power);
        double limit = runtime.milliseconds() + ms;

        while (
            opModeIsActive() &&
            !isHittingSomething(TravelDirection.BACKWARD) &&
            runtime.milliseconds() < limit &&
            (!isValidDistance(TravelDirection.BACKWARD) || isValidDistance(TravelDirection.BACKWARD) && getDistance(TravelDirection.BACKWARD) > x)
        ) {
            autonomousIdleTasks();
        }

        stopMoving();
        return;
    }



    /**
     *  Robot centric. Moves the robot in either 4 direction for ms amount of time OR until it hits an object whichever comes first
     *  if the useCollisionAlerts flag is set to true.  If useCollisionAlerts is set to false, motor will move according
     *  to time limit only.
     *
     *  If the robot has the Odometry module, please use move() instead
     *
     *
     * @param direction                         : FORWARD,BACKWARD,LEFT,RIGHT
     * @param ms                                : Limit of time the motos will be in motion
     * @param power                             : Motor power
     * @param useCollisionAlerts                : If true, he robot will move until it activates the limit switch for more than 1 second.
     *                                          NOTE:  don't use this unless you have a functional limit switch
     */
    private void moveByTime(TravelDirection direction, double ms, double power, boolean useCollisionAlerts) {

        if (power == 0) {
            power = DRIVE_SPEED;
        }
        boolean hasCollidedWithBack     = false;
        boolean hasCollidedWithFront    = false;
        powerPropulsion(direction, power);

        double limit = runtime.milliseconds() + ms;
        double limitToSlowDown = 0;
        double now;

        while (
            opModeIsActive() &&
            (now = runtime.milliseconds()) < limit
        ) {
            if (useCollisionAlerts) {
                /**
                 * If the robot base is equipped with collision switches, this stop looping if colliding
                 */
                if (botBase.collisionFront.isColliding() && direction == TravelDirection.FORWARD && !hasCollidedWithFront) {
                    // this logic was put it so when we collide with an object, we keep going for about 1 second at very low speed
                    // just in case we bumped into it and pushed it away
                    hasCollidedWithFront = true;
                    powerPropulsion(direction, 0.1);
                    limit = now + 1000;
                }
                if (botBase.collisionBack.isColliding() && direction == TravelDirection.BACKWARD && !hasCollidedWithBack) {
                    // this logic was put it so when we collide with an object, we keep going for about 1 second at very low speed
                    // just in case we bumped into it and pushed it away
                    hasCollidedWithBack = true;
                    powerPropulsion(direction, 0.1);
                    limit = now + 1000;
                }
            }
            // @todo needs to be tested
//            if ( now > limitToSlowDown && power > 0.4 ) {
//                powerPropulsion(direction, power / 2.0);
//            }
//            else {
//                powerPropulsion(direction, power);
//            }

            autonomousIdleTasks();
        }
        stopMoving();
        return;
    }


    /**
     *  Robot centric. Moves the robot in either 4 direction until it reaches the given distance OR until it hits an object whichever comes first
     *  if the useCollisionAlerts flag is set to true.  If useCollisionAlerts is set to false, it will ignore the limit switch
     *
     * @param direction                         : FORWARD,BACKWARD,LEFT,RIGHT
     * @param distance                          : Distance to move
     * @param power                             : Motor power
     * @param useCollisionAlerts                : If true, he robot will move until it activates the limit switch for more than 1 second.
     *                                          NOTE:  don't use this unless you have a functional limit switch
     */
    private void move(TravelDirection direction, double distance, double power, boolean useCollisionAlerts) {

        // cannot use this function without Odometry.  see moveByTime instead
        if (!botBase.hasOdometry()) {
            return;
        }

        if (power == 0) {
            power = DRIVE_SPEED;
        }
        boolean hasCollidedWithBack     = false;
        boolean hasCollidedWithFront    = false;
        double limitTime = 0;
        double limit = 0;
        double limitToSlowDown = 0;
        double now;

        powerPropulsion(direction, power);

        switch (direction) {
            case FORWARD:
                limit = botBase.odometer.getCurrentYPos() + distance;
                limitToSlowDown = botBase.odometer.getCurrentYPos() + 0.85 * distance;
                break;
            case BACKWARD:
                limit = botBase.odometer.getCurrentYPos() + distance;
                limitToSlowDown = botBase.odometer.getCurrentYPos() - 0.85 * distance;
                break;
            case RIGHT:
                limit = botBase.odometer.getCurrentXPos() + distance;
                limitToSlowDown = botBase.odometer.getCurrentXPos() + 0.85 * distance;
                break;
            case LEFT:
                limit = botBase.odometer.getCurrentXPos() + distance;
                limitToSlowDown = botBase.odometer.getCurrentXPos() - 0.85 * distance;
                break;
        }

        while (
            opModeIsActive() &&
            (
                (direction == TravelDirection.FORWARD && Math.abs(limit - botBase.odometer.getCurrentYPos()) > 1.0) ||
                (direction == TravelDirection.BACKWARD && Math.abs(limit - botBase.odometer.getCurrentYPos()) > 1.0) ||
                (direction == TravelDirection.RIGHT && Math.abs(limit - botBase.odometer.getCurrentXPos()) > 1.0)  ||
                (direction == TravelDirection.LEFT && Math.abs(limit - botBase.odometer.getCurrentXPos()) > 1.0)
            )
        ) {
            autonomousIdleTasks();

            dbugThis("Limit: " + limit);
            dbugThis("Current: " + botBase.odometer.getCurrentYPos());

            now = runtime.milliseconds();
            if (useCollisionAlerts) {
                /**
                 * If the robot base is equipped with collision switches, this stop looping if colliding
                 */
                if (botBase.collisionFront.isColliding() && direction == TravelDirection.FORWARD && !hasCollidedWithFront) {
                    // this logic was put it so when we collide with an object, we keep going for about 1 second at very low speed
                    // just in case we bumped into it and pushed it away
                    hasCollidedWithFront = true;
                    powerPropulsion(direction, 0.1);
                    limitTime = now + 1000;
                }

                if (botBase.collisionBack.isColliding() && direction == TravelDirection.BACKWARD && !hasCollidedWithBack) {
                    // this logic was put it so when we collide with an object, we keep going for about 1 second at very low speed
                    // just in case we bumped into it and pushed it away
                    hasCollidedWithBack = true;
                    powerPropulsion(direction, 0.1);
                    limitTime = now + 1000;
                }

                if (now > limitTime) {
                    break;
                }
            }

            // @todo needs to be tested
//            if ( now > limitToSlowDown && power > 0.5 ) {
//                powerPropulsion(direction, power / 2.0);
//            }
//            else {
//                powerPropulsion(direction, power);
//            }


        }
        stopMoving();
        return;
    }


    /***
     *
     * Powers the drive train to so the robot moves at an angle
     * angleInRadians is ALWAYS the angle expressed in radians between the desired direction of the movement and the heading of the robot.
     *
     * @param angleInRadians
     * @param power
     *
     * WARNING ***************  This method MUST be called INSIDE a control loop. (see moveDiagonally)  ********************
     */
    protected void powerPropulsionAtAngle(double angleInRadians, double power) {
        if (power == 0) {
            power = DRIVE_SPEED;
        }

        double temp = -angleInRadians + Math.PI/4.0;

        double front_left = power * Math.cos(temp);
        double front_right = power * Math.sin(temp);
        double rear_left = power * Math.sin(temp);
        double rear_right = power * Math.cos(temp);

        // normalize the wheel speed so we don"t exceed 1
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

        botBase.getFrontLeftDrive().setPower(front_left);
        botBase.getFrontRightDrive().setPower(front_right);
        botBase.getRearLeftDrive().setPower(rear_left);
        botBase.getRearRightDrive().setPower(rear_right);

    }


    /**
     * Powers the propulsion to go in either 4 directions
     *
     * @param direction
     * @param power
     *
     * WARNING ***************  This method MUST be called INSIDE a control loop. ********************
     *
     */
    protected void powerPropulsion(TravelDirection direction, double power) {

        if (power == 0) {
            power = DRIVE_SPEED;
        }

        double multiplierFL = 0;
        double multiplierFR = 0;
        double multiplierRL = 0;
        double multiplierRR = 0;

        switch (direction) {
            case FORWARD:
                multiplierFL = 1;
                multiplierFR = 1;
                multiplierRL = 0.97;
                multiplierRR = 0.97;
                propulsionDirection = TravelDirection.FORWARD;
                break;
            case BACKWARD:
                multiplierFL = -1;
                multiplierFR = -1;
                multiplierRL = -0.97;
                multiplierRR = -0.97;
                propulsionDirection = TravelDirection.BACKWARD;
                break;
            case LEFT:
                multiplierFL = -1;
                multiplierFR = 1;
                multiplierRL = 0.97;
                multiplierRR = -0.97;
                propulsionDirection = TravelDirection.LEFT;
                break;
            case RIGHT:
                multiplierFL = 1;
                multiplierFR = -1;
                multiplierRL = -0.97;
                multiplierRR = 0.97;
                propulsionDirection = TravelDirection.RIGHT;
                break;
            default:
                return;

        }

        botBase.getFrontRightDrive().setPower(power * multiplierFR);
        botBase.getRearRightDrive().setPower(power * multiplierRR);
        botBase.getFrontLeftDrive().setPower(power * multiplierFL);
        botBase.getRearLeftDrive().setPower(power * multiplierRL);
    }


    /**
     * @TODO: implement this
     * Determines if the robot is stalled by checking every 2 seconds if is has moved significantly.  It uses
     * class properties to keep track of current and previous positions, and it accrues the stallProbability each time it failed
     * to detect movement.  stallProbability is reset whenever movement is detected.
     *
     * NOTE:  This method is used as a condition to break a movement loop.
     *
     * @return true|false
     */
    private boolean isStalled()
    {
        return false;
    }


    /**
     *
     * @param currentPlacement
     * @param finalDestination
     * @return              : true if position is within target range according to the rule CLOSE_ENOUGH_X and CLOSE_ENOUGH_Y
     *
     */
    public boolean isPositionWithinAcceptableTargetRange(FieldPlacement currentPlacement, FieldPlacement finalDestination)
    {
        return ( (Math.abs(currentPlacement.x - finalDestination.x) <= CLOSE_ENOUGH_X) && (Math.abs(currentPlacement.y - finalDestination.y) <= CLOSE_ENOUGH_Y));
    }



    /**
     * Performs one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    private Boolean _onHeading(double speed, double angle, double PCoeff) {

        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = _getHeadingError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = _getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        botBase.getFrontLeftDrive().setPower(leftSpeed);
        botBase.getRearLeftDrive().setPower(leftSpeed);
        botBase.getFrontRightDrive().setPower(rightSpeed);
        botBase.getRearRightDrive().setPower(rightSpeed);

        return onTarget;
    }


    /**
     * Determines the error between the target angle and the robot"s current heading
     *
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot"s frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     *
     */
    private double _getHeadingError(double targetAngle) {

        double robotError;
        double actualAngle;

        Orientation angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        actualAngle = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - actualAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;

        return robotError;
    }


    /**
     * Calculates the proportional term of the PID
     *
     * NOTE:
     * the output of a PID controller is composed of three distinct components.
     * 1) the proportional term
     * 2) the Integral term
     * 3) the Derivative term
     *
     * The proportional term u = PCoeff * e, where u is the control input, PCoeff is the proportional gain, and e is
     * the error, produces a control input in direct proportion to the error signal.
     * The Proportional Gain Coefficient is a tunable parameter that determines the aggressiveness of the control action.
     *
     * Returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return steer
     *
     */
    private double _getSteer(double error, double PCoeff) {

        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     *
     * This function does nothing but wait while allowing to other processes to run
     *
     * @param ms       : any number of milliseconds.  Can be less than 1
     *
     */
    protected void justWait(double ms) {

        ms = Math.abs(ms);
        double limit = runtime.milliseconds() + ms;

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && runtime.milliseconds() < limit  ) {

            autonomousIdleTasks();
        }
    }


    /**
     * Logs a string to the LogCat Window.  Don't enable DEBUGGING during a match.
     * It will slow down your opmode
     * @param s
     */
    void dbugThis(String s) {

        if ( DEBUG == true ) {
            Log.d("AUTONOMOUS: ", s);
        }
    }

    /**
     *
     * Given a direction, returns true if the robot is too close
     *
     * @param direction
     * @return bool
     *
     */
    public boolean isHittingSomething(TravelDirection direction) {
//
//
//        switch (direction) {
//
//            case FORWARD:
//                if ( getDistance(distanceFront) < 2.0 ) {
//                    return true;
//                }
//                return false;
//
//            case BACKWARD:
//                if ( getDistance(distanceBack) < 2.0 ) {
//                    return true;
//                }
//                return false;
//
//            case LEFT:
//                if ( getDistance(distanceLeft) < 2.0 ) {
//                    return true;
//                }
//                return false;
//
//            case RIGHT:
//                if ( getDistance(distanceRight) < 2.0 ) {
//                    return true;
//                }
//                return false;
//        }

        return false;
    }


    /**
     * Always insert this function inside a control loop as it checks the emergency situations related to limit switches and collisions and acts
     * consequently.
     */
    protected void autonomousIdleTasks() {
        idle();
        botTop.checkAllLimitSwitches();
        botBase.globalCoordinatePositionUpdate();
        botBase.updateLimitSwitchesState();
        botBase.updateSensorPositioningDistances();
        if (searchableTarget != null) {
            searchableTarget.find();
        }
    }


    /**
     * Given a direction, and the power level of the motor, this function will return the amount
     * of time required to power the propulsion to travel the required displacement in inches
     *
     * @param direction
     * @param power
     * @param displacementInInches
     * @return amount of milliseconds in inces
     *
     */
    private int timeToMoveInMs(TravelDirection direction, double power, double displacementInInches) {

        if (direction == TravelDirection.LEFT || direction == TravelDirection.RIGHT ) {
            if (power <= 0.2) {
                return (int) adjustForVoltageDrop(displacementInInches * 166.67);
            }
            if (power <= 0.3) {
                return (int) adjustForVoltageDrop(displacementInInches * 93.80);
            }
            if (power <= 0.4) {
                return (int) adjustForVoltageDrop(displacementInInches * 71.42);
            }
            if (power <= 0.5) {
                return (int) adjustForVoltageDrop(displacementInInches * 60.00);
            }
            if (power <= 0.6) {
                return (int) adjustForVoltageDrop(displacementInInches * 45.45);
            }
            if (power <= 0.7) {
                return (int) adjustForVoltageDrop(displacementInInches * 37.5);
            }

            return (int) adjustForVoltageDrop(displacementInInches * 20.00);
        }
        if (direction == TravelDirection.FORWARD || direction == TravelDirection.BACKWARD ) {
            if (power <= 0.2) {
                return (int) adjustForVoltageDrop(displacementInInches * 125.0);
            }
            if (power <= 0.3) {
                return (int) adjustForVoltageDrop(displacementInInches * 89.29);
            }
            if (power <= 0.4) {
                return (int) adjustForVoltageDrop(displacementInInches * 58.82);
            }
            if (power <= 0.5) {
                return (int) adjustForVoltageDrop(displacementInInches * 53.57);
            }
            if (power <= 0.6) {
                return (int) adjustForVoltageDrop(displacementInInches * 40.92);
            }
            if (power <= 0.7) {
                return (int)adjustForVoltageDrop(displacementInInches * 33.33);
            }
            return (int) adjustForVoltageDrop(displacementInInches * 28.57);
        }

        return 0;
    }

    /**
     * This function applies a transformation to a time value to compensate for the
     * battery voltage drop
     * NOTE: This function becomes totally obsolete when the robot is equipped with the odometry module.
     *
     * @param ms
     * @return
     */
    private double adjustForVoltageDrop(double ms) {
        return ms / (1 - K + (K*_getBatteryVoltage()/13.0));
    }


    /**
     * Returns the distance in inches to the next object in the direction given by direction
     *
     * @param direction
     * @return
     *
     */
    public double getDistance(TravelDirection direction) {

        switch (direction) {
            case FORWARD:
                if (botBase.hasSensorPositioningFront()) {
                    return botBase.distanceFront.getDistance();
                }
                break;
            case BACKWARD:
                if (botBase.hasSensorPositioningBack()) {
                    return botBase.distanceBack.getDistance();
                }
                break;
            case LEFT:
                if (botBase.hasSensorPositioningLeft()) {
                    return botBase.distanceLeft.getDistance();
                }
                break;
            case RIGHT:
                if (botBase.hasSensorPositioningRight()) {
                    return botBase.distanceRight.getDistance();
                }
                break;
        }
        return 0.0;
    }


    /**
     * Returns true if the distance returned by getDistance() can be trusted
     *
     * @param direction
     * @return if the distance is valid and can be trusted
     *
     */
    protected Boolean isValidDistance(TravelDirection direction) {

        switch (direction) {
            case FORWARD:
                if (botBase.hasSensorPositioningFront()) {
                    return !botBase.distanceFront.isOutOfRange();
                }
                break;
            case BACKWARD:
                if (botBase.hasSensorPositioningBack()) {
                    return !botBase.distanceBack.isOutOfRange();
                }
                break;
            case LEFT:
                if (botBase.hasSensorPositioningLeft()) {
                    return !botBase.distanceLeft.isOutOfRange();
                }
                break;
            case RIGHT:
                if (botBase.hasSensorPositioningRight()) {
                    return !botBase.distanceRight.isOutOfRange();
                }
                break;
        }
        return false;
    }


    /***
     * @todo: Make this a component
     *
     * This function returns either RED, BLUE, or BLACK as a default.
     * We use it to position the robot over a line (see the functions move(left|Right|ForwardBackward)ToColor())
     * @param sensor
     * @return
     */
    public int getValidColor(ColorSensor sensor) {
        /**
         * PLEASE NOTE THAT these values are purely experimental and were obtained using a v3 REV Robotics color sensor
         * at a distance no more than 1in. from the target.
         * If you use a v1 or v2, or if the sensor is further away from the target, you will have to recalibrate this
         * function.
         * @todo:  Use machine learning to predict the color based on RGVH values and distance
         */
        if (sensor == null ) {
            dbugThis("Valid color is null");
            return Color.BLACK;
        }

        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        float hsvValues[] = {0F, 0F, 0F};
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (sensor.red() * 255),
                (int) (sensor.green() * 255),
                (int) (sensor.blue() * 255),
                hsvValues);

        if ( red > 2000 && green > 1000 && blue < green &&  hsvValues[0] < 80 ) {
            dbugThis("Valid color is yellow");
            return Color.YELLOW;
        }

        if ( red > 80 && red > green &&  red > blue  ) {
            dbugThis("Valid color is red");
            return Color.RED;
        }

        if ( blue > 80 && blue > green &&  blue > red ) {
            dbugThis("Valid color is blue");
            return Color.BLUE;
        }

        double prb = _percentChange(red,green);
        double ppg = _percentChange(green,blue);
        double prg = _percentChange(red,blue);

        if ( prb > 70  && prb < 80 && ppg > 6 && ppg < 11 && prg > 53 && prg < 62 && hsvValues[0] > 160 && hsvValues[0] < 180) {
            dbugThis("Valid color is white");
            return Color.WHITE;
        }

        if ( red > 700  && red < 800 && green > 1200 && green < 1400 && blue > 1000 && blue < 1200) {
            dbugThis("Valid color is white");
            return Color.WHITE;
        }

        dbugThis("Valid color is default");
        dbugThis("red: " + red + " green: " + green + " blue: " + blue);
        return Color.BLACK;
    }

    /***
     * Calculates the percentage of change from color1 to color 2
     * @param color1
     * @param color2
     * @return
     */
    private int _percentChange(int color1, int color2) {
        return Math.abs(color2 - color1) * 100 / color1;
    }


    /**
     * Computes the current battery voltage.  This function is used in
     * the calculation of the time required to power the drivetrain in
     * order to travel a certain distance.
     * If you have an Odometer, this function becomes completely obsolete
     * @return
     */
    private double _getBatteryVoltage() {
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
package org.firstinspires.ftc.teamcode.Components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This class abstracts the Robot propulsion sytem.
 * It can be used with any types of wheel or motors.
 */

public class BotBase {

    static final boolean DEBUG = false;

    // Timekeeper OpMode members.
    ElapsedTime runtime = new ElapsedTime();

    // Drive Train
    private DcMotor frontLeftDrive      = null;
    private DcMotor frontRightDrive     = null;
    private DcMotor rearLeftDrive       = null;
    private DcMotor rearRightDrive      = null;

    // Decorative LEDs
    private BlinkinBling bling          = null;
    private Boolean hasBling            = null;

    // Odometer
    public Odometer odometer            = null;
    private Boolean hasOdometry         = false;

    // Collision alerts
    public CollisionAlert collisionFront      = null;
    private Boolean hasCollisionFront          = false;
    public CollisionAlert collisionBack       = null;
    private Boolean hasCollisionBack           = false;

    // Sensor Positioning
    public SensorPositioning distanceBack       = null;
    private Boolean hasSensorPositioningBack    = false;
    public SensorPositioning distanceFront      = null;
    private Boolean hasSensorPositioningFront   = false;
    public SensorPositioning distanceLeft       = null;
    private Boolean hasSensorPositioningLeft    = false;
    public SensorPositioning distanceRight      = null;
    private Boolean hasSensorPositioningRight   = false;



    /**
     * initRobot()
     * <p>
     * Configure the Hardware according to the Team Hardware Spreadsheet.
     */
    public BotBase(HardwareMap hardwareMap) {


        /* ***********************************
            ODOMETRY COMPONENT
        */

        hasOdometry = ((odometer = Odometer.init(hardwareMap, "front_left_drive", "front_right_drive", "rear_right_drive")) != null);
        if (!hasOdometry) {
            dbugThis("Odometer not working");
        }

        /* ***********************************
            COLLISION COMPONENTS
        */
        hasCollisionFront = ((collisionFront = CollisionAlert.init(hardwareMap, "collision_front")) != null);
        hasCollisionBack = ((collisionBack = CollisionAlert.init(hardwareMap, "collision_back")) != null);

        /* ***********************************
            SENSOR POSITIONING COMPONENTS
        */
        hasSensorPositioningFront = ((distanceFront = SensorPositioning.init(hardwareMap, 1, "distance_front")) != null);
        hasSensorPositioningBack = ((distanceBack = SensorPositioning.init(hardwareMap, 1, "distance_back")) != null);
        hasSensorPositioningLeft = ((distanceLeft = SensorPositioning.init(hardwareMap, 1, "distance_left")) != null);
        hasSensorPositioningRight = ((distanceRight = SensorPositioning.init(hardwareMap, 1, "distance_right")) != null);


        /* ***********************************
            LED LIGHTS COMPONENTS
         */
        hasBling = ((bling = BlinkinBling.init(hardwareMap, "bling")) != null);


        /* ************************************
            DRIVE TRAIN
        */

        /**
         * NOTE
         * ====
         * Most robots need the motor on one side to be reversed to drive forward
         * Reverse the motor that runs backwards when connected directly to the battery
         */
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearLeftDrive = hardwareMap.get(DcMotor.class, "rear_left_drive");
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearRightDrive = hardwareMap.get(DcMotor.class, "rear_right_drive");
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void stop() {
        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        rearLeftDrive.setPower(0.0);
        rearRightDrive.setPower(0.0);
    }

    public DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    public DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    public DcMotor getRearRightDrive() {
        return rearRightDrive;
    }

    public DcMotor getRearLeftDrive() {
        return rearLeftDrive;
    }

    protected BlinkinBling getBling() {
        return bling;
    }

    public void setBling(double mode) {
        if (hasBling()) {
            bling.setBlinkinPattern(mode);
        }
    }

    public Boolean hasOdometry() { return hasOdometry; }
    public Boolean hasCollisionFront() { return hasCollisionFront; }
    public Boolean hasCollisionBack() { return hasCollisionBack; }
    public Boolean hasSensorPositioningBack() { return hasSensorPositioningBack; }
    public Boolean hasSensorPositioningFront() { return hasSensorPositioningFront; }
    public Boolean hasSensorPositioningLeft() { return hasSensorPositioningLeft; }
    public Boolean hasSensorPositioningRight() { return hasSensorPositioningRight; }
    public Boolean hasBling() { return hasBling; }


    public void globalCoordinatePositionUpdate() {
        if (hasOdometry()) {
            odometer.globalCoordinatePositionUpdate();
        }
    }

    public void updateLimitSwitchesState() {
        if (hasCollisionFront()) {
            collisionFront.updateLimitSwitchState();
        }
        if (hasCollisionBack()) {
            collisionBack.updateLimitSwitchState();
        }
    }

    public void updateSensorPositioningDistances() {
        if ( hasSensorPositioningFront() ) {
            distanceFront.updateSensorPositioningDistance();
        }
        if ( hasSensorPositioningBack() ) {
            distanceBack.updateSensorPositioningDistance();
        }
        if ( hasSensorPositioningLeft() ) {
            distanceLeft.updateSensorPositioningDistance();
        }
        if ( hasSensorPositioningRight() ) {
            distanceRight.updateSensorPositioningDistance();
        }
    }

    private static void dbugThis(String s) {
        if (DEBUG == true) {
            Log.d("BOTBASE: ", s);
        }
    }
}

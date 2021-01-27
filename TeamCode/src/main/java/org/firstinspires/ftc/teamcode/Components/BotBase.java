package org.firstinspires.ftc.teamcode.Components;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Components.RevInputs;

import java.util.List;


/**
 * This class abstracts the Robot propulsion sytem.
 * It can be used with any types of wheel or motors.
 */

public class BotBase {

    static private boolean  ODOMETRY_REVERSE_RIGHT_ENCODER           = true;
    static private boolean  ODOMETRY_REVERSE_LEFT_ENCODER            = true;
    static private boolean  ODOMETRY_REVERSE_HORIZONTAL_ENCODER      = false;

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

    // hubs
    List<LynxModule> allHubs                    = null;



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
        } else {
            if (ODOMETRY_REVERSE_RIGHT_ENCODER) {
                odometer.reverseRightEncoder();
            }
            if (ODOMETRY_REVERSE_LEFT_ENCODER) {
                odometer.reverseLeftEncoder();
            }
            if (ODOMETRY_REVERSE_HORIZONTAL_ENCODER) {
                odometer.reverseHorizontalEncoder();
            }
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


        /* ************************************
            Get access to the Expansion/Control Hub Modules to enable changing caching methods.
            This will allow faster hardware refresh
         */
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

    }

    public  List<LynxModule> getAllHubs() {
        return allHubs;
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

    /**
     * Just a quicker way to cjeck if sensor position is present
     * @param robotSide
     * @return
     */
    public boolean hasSensorPositioning(TravelDirection robotSide) {
        switch (robotSide) {
            case FORWARD:
                return hasSensorPositioningFront;
            case BACKWARD:
                return hasSensorPositioningBack;
            case RIGHT:
                return hasSensorPositioningRight;
            case LEFT:
                return hasSensorPositioningLeft;
        }
        return false;
    }


    private boolean checkThisInput(short thisInput, short amongThisInputs) {
        return ((thisInput & amongThisInputs) != 0);
    }

    public void updateComponents(short whichInputs) {

        if (checkThisInput(RevInputs.BULK, whichInputs)) {
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            if (hasCollisionFront()) {
                collisionFront.updateLimitSwitchState();
            }
            if (hasCollisionBack()) {
                collisionBack.updateLimitSwitchState();
            }
            if (hasOdometry()) {
                odometer.globalCoordinatePositionUpdate();
            }
        }

        // Ic2 doesn't get refreshed with bulk read, so we limit them to what we need
        if (checkThisInput(RevInputs.RANGE_FRONT, whichInputs)) {
            if (hasSensorPositioningFront()) {
                distanceFront.updateSensorPositioningDistance();
            }
        }

        if (checkThisInput(RevInputs.RANGE_REAR, whichInputs)) {
            if (hasSensorPositioningBack()) {
                dbugThis("Updating distance back");
                distanceBack.updateSensorPositioningDistance();
            }
        }

        if (checkThisInput(RevInputs.RANGE_LEFT, whichInputs)) {
            if (hasSensorPositioningLeft()) {
                dbugThis("Updating distance left");
                distanceLeft.updateSensorPositioningDistance();
            }
        }

        if (checkThisInput(RevInputs.RANGE_RIGHT, whichInputs)) {
            if (hasSensorPositioningRight()) {
                dbugThis("Updating distance right");
                distanceRight.updateSensorPositioningDistance();
            }
        }
    }

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
            Log.d("__BOTBASE: ", s);
        }
    }
}

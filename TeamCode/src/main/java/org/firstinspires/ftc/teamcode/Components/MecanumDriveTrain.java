package org.firstinspires.ftc.teamcode.Components;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public class MecanumDriveTrain {

    static final boolean DEBUG = false;

    // Drive Train
    private DcMotor frontLeftDrive      = null;
    private DcMotor frontRightDrive     = null;
    private DcMotor rearLeftDrive       = null;
    private DcMotor rearRightDrive      = null;

    /**
     * STATE VARIABLES
     */

    // Position variables used for storage and calculations


    /**
     * Public static constructor.
     * @param hardwareMap
     * @param hardwareNames
     * @return intance of DriveTrain
     */
    public static MecanumDriveTrain init(HardwareMap hardwareMap, String [] hardwareNames) {
        DcMotor frontLeftDrive      = null;
        DcMotor frontRightDrive     = null;
        DcMotor rearLeftDrive       = null;
        DcMotor rearRightDrive      = null;
        try {
            /**
             * NOTE
             * ====
             * Most robots need the motor on one side to be reversed to drive forward
             * Reverse the motor that runs backwards when connected directly to the battery
             */
            frontLeftDrive = hardwareMap.get(DcMotor.class, hardwareNames[0]);
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontRightDrive = hardwareMap.get(DcMotor.class, hardwareNames[1]);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rearLeftDrive = hardwareMap.get(DcMotor.class, hardwareNames[2]);
            rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rearRightDrive = hardwareMap.get(DcMotor.class, hardwareNames[3]);
            rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
            rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            dbugThis("Cannot intialize drive train");
            return null;
        }
        return new MecanumDriveTrain(new DcMotor[]{
            frontLeftDrive,
            frontRightDrive,
            rearLeftDrive,
            rearRightDrive
        });
    }

    /****
     * Private constructor
     * @param wheels
     */
    private MecanumDriveTrain(DcMotor [] wheels)
    {
        frontLeftDrive      = wheels[0];
        frontRightDrive     = wheels[1];
        rearLeftDrive       = wheels[2];
        rearRightDrive      = wheels[3];
    }

    /**
     * Power the drive train
     * @param wheelPower
     */
    public void setPower(WheelPower wheelPower) {
        frontLeftDrive.setPower(wheelPower.front_left);
        frontRightDrive.setPower(wheelPower.front_right);
        rearLeftDrive.setPower(wheelPower.rear_left);
        rearRightDrive.setPower(wheelPower.rear_right);
    }

    /**
     * Power all wheels with equal power
     * @param power
     */
    public void setPower(double power) {
        Range.clip(power, -1, 1);
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        rearLeftDrive.setPower(power);
        rearRightDrive.setPower(power);
    }

    /**
     * Stops the robot
     */
    public void stop() {
        setPower(0.0);
    }


    public DcMotor getFrontLeftDrive() {
        return frontLeftDrive;
    }

    public DcMotor getFrontRightDrive() {
        return frontRightDrive;
    }

    public DcMotor getRearRightDrive() {
        return rearRightDrive;
    }

    public DcMotor getRearLeftDrive() {
        return rearLeftDrive;
    }


    static private void dbugThis(String s)
    {
        if (DEBUG == true) {
            Log.d("DRIVE TRAIN: ", s);
        }
    }
}

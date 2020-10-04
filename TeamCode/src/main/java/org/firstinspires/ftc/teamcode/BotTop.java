package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This class abstracts the Robot mission related hardware
 */

public class BotTop {

    static final boolean DEBUG                                  = true;


    static final double TORQUENADO_COUNTS_PER_MOTOR_REV        = 1440;                 // eg: REV Motor Encoder
    static final double NEVEREST40_COUNTS_PER_MOTOR_REV        = 1120;                 // eg: REV Motor Encoder

    static final double PROPULSION_DRIVE_GEAR_REDUCTION     = 1.3;                  // This is < 1.0 if geared UP > 1 we are gering down (the small drives the big)
    static final double WHEEL_CIRCUMFERENCE                 = 4.0 * 3.14159;        // For figuring circumference
    static final double PROPULSION_ENCODER_COUNTS_PER_INCH  = (TORQUENADO_COUNTS_PER_MOTOR_REV * PROPULSION_DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;


    /* ************************************
        DC Motors
    */
    private DcMotor dcMotor                     = null;
    private DcMotor intakeMotor                 = null;


    /* ************************************
        CR SErvos
    */
    private CRServo crServo                     = null;


    /* ************************************
        SERVOS
    */
    private Servo servo                             = null;


    /* ************************************
        LIMIT SWITCHES
    */
    DigitalChannel limitDirection1                  = null;
    DigitalChannel limitDirection2                  = null;


    /**
     * STATE VARIABLES
     */
    private int  dcMotorDirection           = 1;

    // Timekeeper
    ElapsedTime runtime = new ElapsedTime();


    /**
     *    Instantiate and configure the Hardware according to the Team Hardware Spreadsheet.
     */
    public BotTop(HardwareMap hardwareMap)
    {
        /* ************************************
            DC MOTORS
        */
        try {
            intakeMotor  = hardwareMap.get(DcMotor.class, "intake_motor");
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize dcMotor");
            intakeMotor = null;
        }


        /**
         * CR SERVOS
         */
        try {
            crServo = hardwareMap.get(CRServo.class, "crservo_motor");
        }
        catch (Exception e) {
            dbugThis("Cannot initialize crServo");
            crServo = null;
        }


        /**
         * SERVO MECHANISMS
         */
        try {
            servo = hardwareMap.get(Servo.class, "servo");
        }
        catch (Exception e) {
            dbugThis("Cannot initialize servo");
            servo = null;
        }

        /**
         * LIMIT Switches for safety
         */
        try {
            limitDirection1 = hardwareMap.get(DigitalChannel.class, "limit_1");
            limitDirection1.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize limitDirection1");
            limitDirection1 = null;
        }
        try {
            limitDirection2 = hardwareMap.get(DigitalChannel.class, "limit_2");
            limitDirection2.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception e) {
            dbugThis("Cannot initialize limitDirection2");
            limitDirection2 = null;
        }
    }

    public void stopAll()
    {
    }

    /**
     * This method should be completed and called during the idle part of autonomous mode
     * @return
     */
    public Boolean checkAllLimitSwitches()
    {
        return true;
    }


    /**
     * SERVO MOVEMENT COMBINATIONS
     */
    public void combo1() {
        if (servo != null) {
            servo.setPosition(0.5);
        }
    }

    void dbugThis(String s) {

        if ( DEBUG == true ) {
            Log.d("BOTTOP: ", s);
        }
    }

    public DcMotor getIntakeMotor() {
        return intakeMotor;
    }
}

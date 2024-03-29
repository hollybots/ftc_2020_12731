package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This class abstracts the Robot mission related hardware
 */

public class BotTop {

    static final double STACKING        = 0.25;
    static final double LAUNCHING       = 0.6;

    static final double RETRACTED       = 1.0;
    static final double EXTENDED        = 0.8;

    static final double CLOSED          = 0.0;
    static final double OPEN            = 1.0;

    static final boolean DEBUG          = false;


    /* ****************************************
        LAUNCHER Volocity PID values
     */
    static final double KP_LAUNCHER     = 60;
    static final double KI_LAUNCHER     = 0;
    static final double KD_LAUNCHER     = 50;
    static final double KF_LAUNCHER     = 14;
    public PIDFCoefficients launcherVelocityPID = null;

    /* ************************************
        DC Motors
    */
    private DcMotor intakeMotor = null;
    private DcMotorEx launchMotor = null;
    private DcMotor clawMotor = null;


    /* ************************************
        CR SErvos
    */
    private CRServo hookCrServo = null;


    /* ************************************
        SERVOS
    */
    private Servo armServo = null;
    private Servo magazineServo = null;
    private Servo collectServo = null;


    /* ************************************
        LIMIT SWITCHES
    */
    DigitalChannel limitDirection1 = null;
    DigitalChannel limitDirection2 = null;

    /* ************************************
        BATTERY VOLTAGE
    */
    private VoltageSensor batteryVoltageSensor;


    /**
     * STATE VARIABLES
     */
    private int dcMotorDirection = 1;

    // Timekeeper
    ElapsedTime runtime = new ElapsedTime();


    /**
     * Instantiate and configure the Hardware according to the Team Hardware Spreadsheet.
     */
    public BotTop(HardwareMap hardwareMap) {
        /* ************************************
            DC MOTORS
        */
        try {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            dbugThis("Cannot initialize intakeMotor");
            intakeMotor = null;
        }

        try {
            clawMotor = hardwareMap.get(DcMotor.class, "claw_motor");
            clawMotor.setDirection(DcMotor.Direction.FORWARD);
            clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            dbugThis("Cannot initialize clawMotor");
            clawMotor = null;
        }

        try {
            launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
            launchMotor.setDirection(DcMotorEx.Direction.REVERSE);

            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            MotorConfigurationType motorConfigurationType = launchMotor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            launchMotor.setMotorType(motorConfigurationType);

            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            dbugThis("Cannot initialize launchMotor");
            launchMotor = null;
        }


        launcherVelocityPID = new PIDFCoefficients(KP_LAUNCHER, KI_LAUNCHER, KD_LAUNCHER, KF_LAUNCHER);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(launchMotor, launcherVelocityPID, batteryVoltageSensor);

        /**
         * CR SERVOS
         */

        try {
            hookCrServo = hardwareMap.get(CRServo.class, "hook_crservo");
        } catch (Exception e) {
            dbugThis("Cannot initialize hook_crservo");
            hookCrServo = null;
        }


        /**
         * SERVO MECHANISMS
         */

        try {
            collectServo = hardwareMap.get(Servo.class, "collect_servo");
        } catch (Exception e) {
            dbugThis("Cannot initialize collectServo");
            collectServo = null;
        }

        try {
            armServo = hardwareMap.get(Servo.class, "arm_servo");
        } catch (Exception e) {
            dbugThis("Cannot initialize armServo");
            armServo = null;
        }

        try {
            magazineServo = hardwareMap.get(Servo.class, "magazine_servo");
        } catch (Exception e) {
            dbugThis("Cannot initialize magazineServo");
            magazineServo = null;
        }

        /**
         * LIMIT Switches for safety
         */
        try {
            limitDirection1 = hardwareMap.get(DigitalChannel.class, "limit_1");
            limitDirection1.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            dbugThis("Cannot initialize limitDirection1");
            limitDirection1 = null;
        }
        try {
            limitDirection2 = hardwareMap.get(DigitalChannel.class, "limit_2");
            limitDirection2.setMode(DigitalChannel.Mode.INPUT);
        } catch (Exception e) {
            dbugThis("Cannot initialize limitDirection2");
            limitDirection2 = null;
        }
    }

    /**
     * Turn the intake motor on using the power in param
     *
     * @param power
     */
    public void intakeMotorOn(double power) {
        intakeMotor.setPower(power);
    }

    /**
     *
     */
    public void intakeMotorOff() {
        intakeMotor.setPower(0.0);
    }

    /**
     * @param power
     */
    public void clawMotorOn(double power) {
        clawMotor.setPower(power);
    }

    public void clawMotorOff() {
        clawMotor.setPower(0);
    }

    public void wobbleGoalHookMotorOn(double power) {
        hookCrServo.setPower(power);
    }

    public void wobbleGoalHookMotorOff() {
        hookCrServo.setPower(0.0);
    }

    public void launchMotorOn(double velocity) {
        launchMotor.setVelocity(velocity);
    }

    public void launchMotorOff() {
        launchMotor.setVelocity(0);
    }

    public DcMotorEx getLaunchMotor() {
        return launchMotor;
    }

    /**
     * Set magazine position so it can stack rings
     */
    public void lowerMagazine() {
        magazineServo.setPosition(STACKING);
    }

    /**
     * Ramp up magazine position so it can launch rings
     */
    public void liftMagazine() {
        magazineServo.setPosition(LAUNCHING);
    }

    /**
     * Ramp up magazine position so it can launch rings
     */
    public boolean rampUpMagazine() {
        double currentPosition = magazineServo.getPosition();
        // it is possible that the servo is in an unknown position, in which case is going to return NaN.
        // If this is the case, we are going to set its position back to stacking
        if (Double.isNaN(currentPosition)) {
            currentPosition = STACKING;
            magazineServo.setPosition(currentPosition);
        }
        currentPosition += 0.1;
        magazineServo.setPosition(Math.min(currentPosition, LAUNCHING));
        if (currentPosition >= LAUNCHING) {
            return true;
        }
        return false;
    }

    /**
     * Set collector position so it closes in on the wobble goal
     */
    public void collectIn() {
        collectServo.setPosition(CLOSED);
    }

    /**
     * Set collector position so it opens up
     */
    public void collectOut() {
        collectServo.setPosition(OPEN);
    }

    /**
     * Set actuator arm position so it is retracted
     */
    public void retractArm() {
        armServo.setPosition(RETRACTED);
    }

    /**
     * Set actuator arm position so it is extended where it can launch a ring to the launcher
     */
    public void extendArm() {
        armServo.setPosition(EXTENDED);
    }

    /***
     * Stop all motor at once
     */
    public void stopAll() {
        launchMotorOff();
        intakeMotorOff();
        clawMotorOff();
    }


    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients, VoltageSensor batteryVoltageSensor) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }


    /**
     * This method should be completed and called during the idle part of autonomous mode
     *
     * @return
     */
    public Boolean checkAllLimitSwitches() {
        return true;
    }

    void dbugThis(String s) {

        if (DEBUG == true) {
            Log.d("BOTTOP: ", s);
        }
    }

}

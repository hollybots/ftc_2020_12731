package org.firstinspires.ftc.teamcode.Components;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;


public class SensorPositioning {

    private static boolean DEBUG                       = false;
    private static int INVALID_DISTANCE                = -1;

    private double currentDistance              = INVALID_DISTANCE;
    private double previousDistance             = INVALID_DISTANCE;
    private String prefix                       = "";

    private List<ModernRoboticsI2cRangeSensor> distanceSensors = null;

    public static SensorPositioning init(HardwareMap hardwareMap, int nbSensors, String sensorPrefix) {

        List<ModernRoboticsI2cRangeSensor> distanceSensors        = new ArrayList();
        ModernRoboticsI2cRangeSensor sensor;
        for (int i=1; i<=nbSensors; i++) {
            try {
                sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, sensorPrefix + "_" + i);
            } catch (Exception e) {
                dbugThis("Cannot initialize sensor " + sensorPrefix + " " + i);
                continue;
            }
            distanceSensors.add(sensor);
        }
        if (distanceSensors.size() <= 0) {
            return null;
        }
        return new SensorPositioning(distanceSensors, sensorPrefix);
    }


    /**
     * This method initializes this class with the parameters needed to orient our robot using the Distance sensors on the robot
     *
     * @param distanceSensors
     */
    private SensorPositioning(List<ModernRoboticsI2cRangeSensor> distanceSensors, String prefix)
    {
        this.distanceSensors         = distanceSensors;
        this.prefix                  = prefix;

    }


    public double getDistance()
    {
        return currentDistance;
    }

    public Boolean isValidDistance()
    {
        return (currentDistance != INVALID_DISTANCE);
    }

    public void updateSensorPositioningDistance() {

        double average = 0.0;
        double distanceRead = -1;
        Boolean distanceIsValid = false;

        if (distanceSensors.size() <= 0) {
            return;
        }
        int n = 0;
        for (int i=0; i < distanceSensors.size(); i++) {
            distanceRead = distanceSensors.get(i).getDistance(DistanceUnit.INCH);
            dbugThis(String.format("Distance for index %d: %.3f", i, distanceRead));
            if (distanceRead != DistanceSensor.distanceOutOfRange && distanceRead > 0 && !Double.isNaN(distanceRead) && distanceRead < 144.0) {
                dbugThis(String.format("Distance for %s_%d: %.3f", prefix, i, distanceRead));
                distanceIsValid = true;
                average += distanceRead;
                n++;
            } else {
                dbugThis(String.format("Distance for %s_%d is invalid", prefix, i));
            }
        }

        if (distanceIsValid) {
            average = average / n;
            previousDistance = currentDistance;
            currentDistance = average;
            dbugThis(String.format("Average distance for %s: %.3f", prefix, average));
            return;
        }
        currentDistance = INVALID_DISTANCE;
    }

    private static void dbugThis(String s) {
        if (DEBUG == true) {
            Log.d("SENSOR_POS: ", s);
        }
    }
}
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

    private static boolean DEBUG                       = true;
    private static int INVALID_DISTANCE                = -1;
    private static int TIME_SERIES_SIZE                = 5;

    private double currentDistance              = INVALID_DISTANCE;
    private double previousDistance             = INVALID_DISTANCE;
    private String prefix                       = "";
    private ElapsedTime sampleTimer             = new ElapsedTime();
    private double timestamp                    = 0;

    private class DataPoint {
        public double timestamp;
        public double value;
        public DataPoint(double timestamp, double value) {
            this.timestamp = timestamp;
            this.value = value;
        }
    }
    DataPoint[] timeSeries = new DataPoint[TIME_SERIES_SIZE];
    int nbSamples = 0;

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
        sampleTimer.reset();
        timestamp = 0;
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
//            dbugThis(String.format("Distance for index %d: %.3f", i, distanceRead));
            if (!isOutlier(sampleTimer.milliseconds(), distanceRead)) {
                dbugThis(String.format("Distance for %s_%d: %.3f", prefix, i, distanceRead));
                distanceIsValid = true;
                average += distanceRead;
                n++;
            } else {
                dbugThis(String.format("Distance for %s_%d is invalid", prefix, i));
            }
        }

        if (distanceIsValid) {
            // check if change makes sense
            average = average / n;
            previousDistance = currentDistance;
            currentDistance = average;
            dbugThis(String.format("Average distance for %s: %.3f", prefix, average));
            return;
        }
        currentDistance = INVALID_DISTANCE;
    }


    private boolean isOutlier(double timestamp, double value) {

        if (value == DistanceSensor.distanceOutOfRange || value < 0 || Double.isNaN(value) || value > 144.0) {
            return true;
        }

        if (nbSamples < 5) {
            insertInTimeSeries(timestamp, value);
            nbSamples++;
            return false;
        }

        // get the previous value and calculate the slope for n-1,n n-2,n-1 n-3, n-2
        DataPoint v_t_minus_1 = new DataPoint(timeSeries[timeSeries.length-1].timestamp, slope(timeSeries[timeSeries.length-1], new DataPoint(timestamp, value)));
        DataPoint v_t_minus_2 = new DataPoint(timeSeries[timeSeries.length-2].timestamp, slope(timeSeries[timeSeries.length-2], timeSeries[timeSeries.length-1]));
        DataPoint v_t_minus_3 = new DataPoint(timeSeries[timeSeries.length-2].timestamp, slope(timeSeries[timeSeries.length-3], timeSeries[timeSeries.length-2]));

        if (Math.abs(v_t_minus_2.value - v_t_minus_1.value) > 3.0) {  //in inch per millisecond
            dbugThis("It's an outlier because of spped");
            return true;
        }

//        double a_t_minus_2 = slope(v_t_minus_2, v_t_minus_1);
//        double a_t_minus_3 = slope(v_t_minus_3, v_t_minus_2);
//
//        if (Math.abs(v_t_minus_2.value - v_t_minus_1.value) > 3.0) {  //in inch per millisecond
//            return true;
//        }

        insertInTimeSeries(timestamp, value);
        return false;
    }

    private double slope(DataPoint p1, DataPoint p2) {
        if (p2.timestamp != p1.timestamp) {
            return (p2.value - p1.value) / (p2.timestamp - p1.timestamp);
        }
        return 0;
    }

    private void insertInTimeSeries(double timestamp, double value) {
        for (int i=0; i <= timeSeries.length-2; i++) {
            timeSeries[i] = timeSeries[i+1];
        }
        timeSeries[timeSeries.length-1] = new DataPoint(timestamp, value);
    }

    private static void dbugThis(String s) {
        if (DEBUG == true) {
            Log.d("__SENSOR_POS: ", s);
        }
    }
}
package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;


public class SensorPositioning {

    private boolean DEBUG                       = false;

    private double currentDistance              = DistanceSensor.distanceOutOfRange;
    private double previousDistance             = DistanceSensor.distanceOutOfRange;

    private List<ModernRoboticsI2cRangeSensor> distanceSensors = null;

    public static SensorPositioning init(HardwareMap hardwareMap, int nbSensors, String sensorPrefix) {

        List<ModernRoboticsI2cRangeSensor> distanceSensors        = new ArrayList();
        ModernRoboticsI2cRangeSensor sensor;
        for (int i=1; i<=nbSensors; i++) {
            try {
                sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, sensorPrefix + "_" + i);
            } catch (Exception e) {
                continue;
            }
            distanceSensors.add(sensor);
        }
        if (distanceSensors.size() <= 0) {
            return null;
        }
        return new SensorPositioning(distanceSensors);
    }


    /**
     * This method initializes this class with the parameters needed to orient our robot using the Distance sensors on the robot
     *
     * @param distanceSensors
     */
    private SensorPositioning(List<ModernRoboticsI2cRangeSensor> distanceSensors)
    {
        this.distanceSensors         = distanceSensors;
    }


    public double getDistance()
    {
        return currentDistance;
    }

    public Boolean isOutOfRange()
    {
        return (DistanceSensor.distanceOutOfRange == currentDistance);
    }

    public void updateSensorPositioningDistance() {

        double average = 0.0;
        double distanceRead = DistanceSensor.distanceOutOfRange;
        Boolean distanceIsValid = false;

        if (distanceSensors.size() <= 0) {
            return;
        }

        for (int i=0; i < distanceSensors.size(); i++) {
            distanceRead = distanceSensors.get(i).getDistance(DistanceUnit.INCH);
            if (distanceRead != DistanceSensor.distanceOutOfRange) {
                distanceIsValid = true;
                average += distanceRead;
            }
        }
        if (distanceIsValid) {
            average = average / distanceSensors.size();
            previousDistance = currentDistance;
            currentDistance = average;
            return;
        }

//        if (sensor == null) {
//            return 255;
//        }

//        double limit = runtime.milliseconds() + 1000;
//        double validDistance = sensor.getDistance(DistanceUnit.INCH);
//        while ( opModeIsActive() &&
//                runtime.milliseconds() < limit &&
//                ((validDistance = sensor.getDistance(DistanceUnit.INCH))) == DistanceSensor.distanceOutOfRange )  {
//            autonomousIdleTasks();
//        }
//
//        dbugThis("From get Valid distance" + validDistance);
//
//        return validDistance == DistanceSensor.distanceOutOfRange ? 255 : validDistance;
    }
}
package org.firstinspires.ftc.teamcode.Components;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LineDetector {

    static final boolean DEBUG = false;

    /* Color sensors
     */
    protected ColorSensor bottomColorSensor             = null;

    /**
     * STATE VARIABLES
     */

    // Position variables used for storage and calculations
    private int bottomColorValue                    = -1;
    private int bottomColorPreviousValue            = -1;
    private int red;
    private int green;
    private int blue;


    /**
     * Public static constructor.
     * @param hardwareMap
     * @param hardwareName
     * @return
     */
    public static LineDetector init(HardwareMap hardwareMap, String hardwareName) {
        ColorSensor bottomColorSensor;
        try {
            bottomColorSensor = hardwareMap.get(ColorSensor.class, hardwareName);
        } catch (Exception e) {
            dbugThis("Cannot intialize " + hardwareName);
            return null;
        }
        return new LineDetector(bottomColorSensor);
    }

    /****
     * Private constructor
     * @param bottomColor
     */
    private LineDetector(ColorSensor bottomColor)
    {
        bottomColorSensor = bottomColor;
    }

    public void updateLineDetectorValue()
    {
        bottomColorPreviousValue = bottomColorValue;
        bottomColorValue = getValidColor();
    }

    public int getLineDetectorValue() {
        return bottomColorValue;
    }

    /***
     * This function returns either RED, WHITE, YELLOW, or BLUE, or BLACK as a default.
     * We use it to position the robot over a line (see the functions move(left|Right|ForwardBackward)ToColor())
     * @return
     */
    private int getValidColor() {
        /**
         * PLEASE NOTE THAT these values are purely experimental and were obtained using a v3 REV Robotics color sensor
         * at a distance no more than 1in. from the target.
         * If you use a v1 or v2, or if the sensor is further away from the target, you will have to recalibrate this
         * function.
         * @todo:  Use machine learning to predict the color based on RGVH values and distance
         */
        if (bottomColorSensor == null ) {
            dbugThis("Valid color is null");
            return Color.BLACK;
        }

        red = bottomColorSensor.red();
        green = bottomColorSensor.green();
        blue = bottomColorSensor.blue();

        float hsvValues[] = {0F, 0F, 0F};
        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        Color.RGBToHSV((int) (bottomColorSensor.red() * 255),
                (int) (bottomColorSensor.green() * 255),
                (int) (bottomColorSensor.blue() * 255),
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

        if ( red > 600  && red < 800 && green > 1000 && green < 1400 && blue > 900 && blue < 1200) {
            dbugThis("Valid color is white");
            return Color.WHITE;
        }

        dbugThis("Valid color is default");
        dbugThis("red: " + red + " green: " + green + " blue: " + blue);
        return Color.BLACK;
    }

    /**
     * Returns the red value for this sensor
     * @return
     */
    public int getRed() {
        return red;
    }

    /**
     * Returns the green value for this sensor
     * @return
     */
    public int getGreen() {
        return red;
    }

    /**
     * Returns the blue value for this sensor
     * @return
     */
    public int getBlue() {
        return red;
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

    static private void dbugThis(String s)
    {
        if (DEBUG == true) {
            Log.d("LINE DETECTOR: ", s);
        }
    }
}

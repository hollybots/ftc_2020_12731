package org.firstinspires.ftc.teamcode.Components;

public class FieldPlacement {

    /*
        Position along the X axis running from left to right, when facing the Blue alliance.
        Having 0 as the field center
     */
    public double x;     //inches

    /*
        Position along the Y axis running from Red alliance to the Blue alliances
        Having 0 as the field center
    */
    public double y;     //inches

    public double theta;
    public double r;

    /*
        Heading of the robot.  0 is the heading when the robot is parallel to the Y axis
     */
    public double orientation;   // degrees


    public FieldPlacement(FieldPlacement another) {
        this.x              = another.x;
        this.y              = another.y;
        this.theta          = another.theta;
        this.r              = another.r;
        this.orientation    = another.orientation;
    }

    public FieldPlacement(double x, double y) {

        this.x = x;
        this.y = y;
        r = Math.sqrt(x*x + y*y);
        theta = Math.acos(x/r);

        this.orientation = 0.0;
    }


    public FieldPlacement(double x, double y, double orientation) {

        this.x = x;
        this.y = y;
        r = Math.sqrt(x*x + y*y);
        theta = Math.acos(x/r);

        this.orientation = orientation;

    }

    public FieldPlacement(double r, double theta, boolean vectorial) {

        this.theta = theta;
        this.r = r;
        x = r * Math.cos(theta);
        y = r * Math.sin(theta);

        this.orientation = 0.0;

    }

    public FieldPlacement(double r, double theta, boolean vectorial, double orientation) {

        this.theta = theta;
        this.r = r;
        x = r * Math.cos(theta);
        y = r * Math.sin(theta);

        this.orientation = orientation;

    }

    public String toString() {
        return String.format("dx: %.3f, dy: %.3f, <%.3f", x, y, theta);
    }

}

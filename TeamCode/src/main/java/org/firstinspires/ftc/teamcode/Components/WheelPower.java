package org.firstinspires.ftc.teamcode.Components;

public class WheelPower {
    public double front_left;
    public double front_right;
    public double rear_left;
    public double rear_right;


    public void normalize() {
        // normalize the wheel speed so we don"t exceed 1
        double max = Math.abs(front_left);
        if (Math.abs(front_right)>max) {
            max = Math.abs(front_right);
        }
        if (Math.abs(rear_left)>max){
            max = Math.abs(rear_left);
        }
        if (Math.abs(rear_right)>max) {
            max = Math.abs(rear_right);
        }
        if ( max > 1.0 ) {
            front_left /= max;
            front_right /= max;
            rear_left /= max;
            rear_right /= max;
        }
    }
};
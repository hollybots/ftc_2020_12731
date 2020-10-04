package org.firstinspires.ftc.teamcode.Components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BlinkinBling  {

    private Servo blinkinModule           = null;

    public static BlinkinBling init(HardwareMap hardwareMap, String hardwareName) {
        Servo blinkinModule;
        try {
            blinkinModule = hardwareMap.get(Servo.class, hardwareName);
        } catch (Exception e) {
            Log.d("BOTBASE: ", "Cannot intialize Bling");
            return null;
        }
        return new BlinkinBling(blinkinModule);
    }

    public BlinkinBling(Servo blinkinModule )
    {
        this.blinkinModule = blinkinModule;
    }

    public void  setBlinkinPattern(double patternNo)
    {
        blinkinModule.setPosition(patternNo);
    }
}
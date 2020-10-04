package org.firstinspires.ftc.teamcode.Components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class CollisionAlert {

    static final boolean DEBUG = true;


    /**
     * HARDWARE
     */
    // Collision limit switches
    DigitalChannel limitSwitch                      = null;

    /**
     * STATE VARIABLES
     */

    // Position variables used for storage and calculations
    private Boolean limitSwichtState                    = false;
    private Boolean limitSwichtPreviousState            = false;

    public static CollisionAlert init(HardwareMap hardwareMap, String limitSwitchHardwareName) {
        DigitalChannel limitSwitch;
        try {
            limitSwitch = hardwareMap.get(DigitalChannel.class, limitSwitchHardwareName);
        } catch (Exception e) {
            dbugThis("Cannot intialize " + limitSwitchHardwareName);
            return null;
        }
        return new CollisionAlert(limitSwitch);
    }


    /****
     *
     * @param limitSwitch
     */
    private CollisionAlert(DigitalChannel limitSwitch)
    {
        this.limitSwitch = limitSwitch;
    }

    public void updateLimitSwitchState()
    {
        limitSwichtPreviousState = limitSwichtState;
        limitSwichtState = limitSwitch.getState();
    }

    public Boolean isColliding()
    {
        return (limitSwichtState != true);
    }


    static void dbugThis(String s)
    {
        if (DEBUG == true) {
            Log.d("COLLISION: ", s);
        }
    }
}

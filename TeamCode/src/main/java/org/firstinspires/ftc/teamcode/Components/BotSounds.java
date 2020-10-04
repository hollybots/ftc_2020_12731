
package org.firstinspires.ftc.teamcode.Components;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class plays a redefined song indluded in the SkyStone resources.
 * Construct the class using sounds = new BotSounds(Hardwaremap);
 * then play a song
 * sounds.play(<name>);
 *
 */

public class BotSounds
{

//    // List of available sound resources
//    String  sounds[] =  {"ss_alarm", "ss_bb8_down", "ss_bb8_up", "ss_darth_vader", "ss_fly_by",
//            "ss_mf_fail", "ss_laser", "ss_laser_burst", "ss_light_saber", "ss_light_saber_long", "ss_light_saber_short",
//            "ss_light_speed", "ss_mine", "ss_power_up", "ss_r2d2_up", "ss_roger_roger", "ss_siren", "ss_wookie" };


    boolean soundPlaying                = false;
    Context myApp                       = null;
    SoundPlayer.PlaySoundParams params  = null;


    public BotSounds(HardwareMap hardwareMap) {

        // Variables for choosing from the available sounds
        myApp = hardwareMap.appContext;

        // create a sound parameter that holds the desired player parameters.
        params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = true;
    }



    public void play(String soundName)
    {
        if (soundPlaying) {
            return;
        }

        int     soundID         = -1;

        // Determine Resource IDs for the sounds you want to play, and make sure it's valid.
        if ((soundID = myApp.getResources().getIdentifier(soundName, "raw", myApp.getPackageName())) == 0) {
            return;
        }

        // Signal that the sound is now playing.
        soundPlaying = true;

        // Start playing, and also Create a callback that will clear the playing flag when the sound is complete.
        SoundPlayer.getInstance().startPlaying(myApp, soundID, params, null,
            new Runnable() {
                public void run() {
                    soundPlaying = false;
                }} );
    }
}

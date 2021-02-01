package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import android.app.Activity;
import android.widget.ImageView;
import android.widget.LinearLayout;

public class AdWidget {

    public static void displayLogo(HardwareMap hm) {

        final AppUtil appUtil   = AppUtil.getInstance();
        final Activity activity = appUtil.getActivity();
        final HardwareMap hardwareMap = hm;
        appUtil.runOnUiThread(new Runnable()
        {
            @Override public void run()
            {
                LinearLayout linearLayout = (LinearLayout) activity.findViewById(com.qualcomm.ftcrobotcontroller.R.id.monitorContainer);
                ImageView imageview = (ImageView) ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.logoId);
                imageview.setImageResource(org.firstinspires.ftc.teamcode.R.drawable.b3_logo);
            }
        });
    }
}

package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import android.app.Activity;
import android.content.res.Configuration;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.LinearLayout;

public class Advertise {

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
//    protected void setMonitorViewParent(@IdRes int resourceId)
//    {
//        View view = this.activity.findViewById(resourceId); // may return null if resourceId is, e.g., zero
//        setMonitorViewParent((ViewGroup)view);
//    }
//
//    protected void setMonitorViewParent(@Nullable ViewGroup viewParent)
//    {
//        this.glSurfaceParent = viewParent;
//        if (glSurfaceParent != null)
//        {
//            glSurfaceParentPreviousVisibility = glSurfaceParent.getVisibility();
//        }
//    }


}

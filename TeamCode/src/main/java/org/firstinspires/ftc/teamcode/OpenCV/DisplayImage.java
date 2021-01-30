package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import android.content.res.Configuration;
import android.view.View;
import android.widget.ImageView;
import android.widget.LinearLayout;

public class DisplayImage {

    private int monitorViewId = 0;
    HardwareMap hardwareMap;
    DisplayImage(HardwareMap hardwareMap) {
        monitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.hardwareMap = hardwareMap;


    }

    public void loadFromResource(String name) {
//
//        ImageView image = (ImageView) findViewById(monitorViewId);
//        Bitmap bMap = BitmapFactory.decodeFile("/sdcard/test2.png");
//        image.setImageBitmap(bMap);
//        Mat img = Utils.loadResource(hardwareMap.appContext.getPackageName(), refrenceimgID, Highgui.CV_LOAD_IMAGE_COLOR);

//        LinearLayout monitorContainer = (LinearLayout) findViewById(com.qualcomm.ftcrobotcontroller.R.id.monitorContainer);
//        if (configuration.orientation == Configuration.ORIENTATION_LANDSCAPE) {
//            // When the phone is landscape, lay out the monitor views horizontally.
//            monitorContainer.setOrientation(LinearLayout.HORIZONTAL);
//            for (int i = 0; i < monitorContainer.getChildCount(); i++) {
//                View view = monitorContainer.getChildAt(i);
//                view.setLayoutParams(new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1 /* weight */));
//            }
//        } else {
//            // When the phone is portrait, lay out the monitor views vertically.
//            monitorContainer.setOrientation(LinearLayout.VERTICAL);
//            for (int i = 0; i < monitorContainer.getChildCount(); i++) {
//                View view = monitorContainer.getChildAt(i);
//                view.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1 /* weight */));
//            }
//        }
    }

}

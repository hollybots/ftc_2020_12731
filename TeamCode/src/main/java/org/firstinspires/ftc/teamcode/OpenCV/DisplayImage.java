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

public class DisplayImage {

    private int monitorViewId = 0;
    HardwareMap hardwareMap;
    DisplayImage(HardwareMap hardwareMap) {
        monitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.hardwareMap = hardwareMap;


    }

    public void loadFromResource(String name) {
        Mat img = Utils.loadResource(hardwareMap.appContext.getPackageName(), refrenceimgID, Highgui.CV_LOAD_IMAGE_COLOR);
    }

}

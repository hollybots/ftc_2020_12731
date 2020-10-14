package org.firstinspires.ftc.teamcode.Components;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public class TensorFlowObjectIdentification implements ObjectIdentificationInterface
{
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    private HardwareMap hardwareMap                     = null;
    private Telemetry telemetry                         = null;
    private WebcamName webcam                           = null;

    // If we want to display DEBUG info in Logcat
    private boolean DEBUG
            = true;
    // VuForia Key, register online
    protected static final String VUFORIA_KEY = "AQSpIWb/////AAABmarkN2D6nEtEtiFZY75jnL8H12+mIcvqBOz+WbP/qxrgyY4JZJzBTN7NjGGX8q2gu0M06DZpIqUq8R0c06ZoXTOT/77I4+Hp8s4hwMr5jcZvqzq9TGgg/83hGs48KIRNW0kRiWWJulpUG8v+c+jNBW3csk/Un2yofaPK61SkPAQkLaddWk7j4zMZfk1lOaRv4H11MTX3g12DB2eVjRcv8jGC3Wt0T5q+zll0iLjqpegF2FrL2dxDyEb7dyfQJ+5OlnqotKESWhCMDEBTJZKgopJTA7Rnf5uUdLVzRGV2S0VoJFwG/eYc+SaP6jx1dTt3SqTEzs3GbY9u4HQms03n7WavQtc20U4SKU9eyXvhS6NX";


    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private String targetName = null;

    private FieldPlacement oldPlacement            = null;
    private FieldPlacement currentPlacement        = null;


    protected  List<Recognition> lastUpdatedRecognitions = null;

    public TensorFlowObjectIdentification(
        HardwareMap hardwareMap,
        Telemetry telemetry,
        String modelAssetName,      // identifiable models file ressource
        String [] assetNames,       // Name of the identifiable asset within the file, important
        String targetName,
        String typeOfCamera,        // PHONE or WEBCAM
        boolean debug)
    {
        this.DEBUG          = debug;
        this.hardwareMap    = hardwareMap;
        this.telemetry      = telemetry;
        this.targetName     = targetName;

        initVuforia(typeOfCamera);
        initTfod(modelAssetName, assetNames);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
    }

    public FieldPlacement getTargetRelativePosition() {
        return currentPlacement;
    }


    public void find()
    {
        FieldPlacement placement = null;

        if (tfod == null) {
            return;
        }

        List<Recognition> recognitions = tfod.getRecognitions();
        if (recognitions == null) {
            return;
        }

        telemetry.addData("# Object Detected", recognitions.size());
        // step through the list of recognitions and display boundary info.
        int i = 0;

        // ther emigh be multpi objects to recognize here.  We are only interestd in one
        for (Recognition recognition : recognitions) {

            if (recognition.getLabel() != targetName) {
                continue;
            }

            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());

            oldPlacement = currentPlacement;
            currentPlacement = new FieldPlacement(recognition.getRight(), recognition.getBottom());
        }
    }

    public void stop()
    {
        if (tfod != null) {
            tfod.shutdown();
        }
    }


    /**
     * Initialize the Vuforia localization engine.
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    private void initVuforia(String typeOfCamera) {

        VuforiaLocalizer.Parameters parameters = null;

        if (typeOfCamera == "WEBCAM") {
            try {
                webcam = hardwareMap.get(WebcamName.class, "webcam");
            } catch (Exception e) {
                webcam = null;
                dbugThis("Cannot intialize webcam");
            }
            parameters = new VuforiaLocalizer.Parameters();
            parameters.cameraName = webcam;

        } else {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.cameraDirection = FRONT;
        }

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private TFObjectDetector initTfod(String modelAssetName, String [] assetsLabel) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId",
            "id",
            hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(modelAssetName, assetsLabel);
        return tfod;
    }


    private void dbugThis(String s) {

        if ( this.DEBUG == true ) {
            Log.d("TFOD:", s);
        }
    }
}
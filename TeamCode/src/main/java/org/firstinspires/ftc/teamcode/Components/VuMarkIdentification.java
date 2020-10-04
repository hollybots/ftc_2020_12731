/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.Components;


import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This class abstracts the Vuforia engine.
 * It is used to identity a Vuforia VuMark encountered on field.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */
public class VuMarkIdentification implements ObjectIdentificationInterface
{
    // Webcam
    private WebcamName webcam           = null;

    // VuForia Key, register online
    protected static final String VUFORIA_KEY = "AQSpIWb/////AAABmarkN2D6nEtEtiFZY75jnL8H12+mIcvqBOz+WbP/qxrgyY4JZJzBTN7NjGGX8q2gu0M06DZpIqUq8R0c06ZoXTOT/77I4+Hp8s4hwMr5jcZvqzq9TGgg/83hGs48KIRNW0kRiWWJulpUG8v+c+jNBW3csk/Un2yofaPK61SkPAQkLaddWk7j4zMZfk1lOaRv4H11MTX3g12DB2eVjRcv8jGC3Wt0T5q+zll0iLjqpegF2FrL2dxDyEb7dyfQJ+5OlnqotKESWhCMDEBTJZKgopJTA7Rnf5uUdLVzRGV2S0VoJFwG/eYc+SaP6jx1dTt3SqTEzs3GbY9u4HQms03n7WavQtc20U4SKU9eyXvhS6NX";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;

    private OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    private Telemetry telemetry;
    private boolean DEBUG                          = false;

    // active state
    private boolean active                         = false;

    VuforiaTrackable mainTarget                    = null;  //
    VuforiaTrackables targetTrackables             = null;  // List of all the trackable in

    private FieldPlacement oldPlacement            = null;
    private FieldPlacement currentPlacement        = null;


    public VuMarkIdentification(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            String trackableAssetName,  // Trcakable file
            String mainAssetName,       // Name of the trackable asset (for debug info. Can be anyting)
            int targetIndex,            // index of the trackable in the trackable asset file
            String typeOfCamera,        // PHONE or WEBCAM
            boolean debug)
    {

        this.DEBUG = debug;
        this.telemetry = telemetry;

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
            /*
             * If using the phone, we can configure Vuforia by passing the handle to a camera preview resource (on the RC phone);
             */
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.cameraDirection = FRONT;
        }
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * Instantiate the Vuforia engine
         */
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetTrackables = vuforia.loadTrackablesFromAsset(trackableAssetName);
        mainTarget = targetTrackables.get(targetIndex);
        mainTarget.setName(mainAssetName);

        targetTrackables.activate();
    }

    public void find() {

        FieldPlacement placement = null;

        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)mainTarget.getListener()).getFtcCameraFromTarget();
        telemetry.addData("Pose", format(pose));
        dbugThis("Pose : " + format(pose));

        /* We further illustrate how to decompose the pose into useful rotational and
         * translational components */
        if (pose != null) {
            VectorF translation     = pose.getTranslation();
            Orientation rot         = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            double tX = translation.get(0) / mmPerInch;
            double tY = translation.get(1) / mmPerInch;
            oldPlacement = currentPlacement;
            currentPlacement = new FieldPlacement(tX, tY);
        }
        return;
    }


    public void stop() {
        targetTrackables.deactivate();
    }


    private String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public FieldPlacement getTargetRelativePosition() {
        return currentPlacement;
    }


    private void dbugThis(String s) {

        if ( this.DEBUG == true ) {
            Log.d("VUMARK:", s);
        }
    }
}

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

package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.FieldPlacement;
import org.firstinspires.ftc.teamcode.Components.ObjectIdentificationInterface;
import org.firstinspires.ftc.teamcode.Components.TensorFlowObjectIdentification;
import org.firstinspires.ftc.teamcode.Components.VuMarkIdentification;
import org.firstinspires.ftc.teamcode.Components.WheelPower;
import org.firstinspires.ftc.teamcode.TeleOpModesBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Test Object Detection", group="3")
@Disabled
public class TestObjectDetection extends TeleOpModesBase
{

    protected static final String CAMERA_SYSTEM                 = "PHONE";
    //    protected static final String CAMERA_SYSTEM                 = "WEBCAM";

    protected static final String IDENTIFICATION_SYSTEM         = "VUFORIA";
//    protected static final String IDENTIFICATION_SYSTEM         = "TSF";

    // Will dump debug information in the LogCat if true
    protected boolean DEBUG                               = false;

    // Needed for VUFORIA Vumark Identification
    protected String TRACKABLE_ASSET_NAME                 = "Skystone";
    protected String TRACKABLE_NAME                       = "Sky Stone";  // Can be anyting
    protected int TRACKABLE_INDEX                         = 0;


    // Needed for TENSORFLOW object Identification
    private static final String TFOD_MODEL_ASSET            = "Skystone.tflite";
    private static final String [] TFOD_MODEL_ASSETS_LABEL  = {"Stone", "Skystone"};
    private static final String TFOD_TARGET_LABEL           = "Skystone";

    // alignment camera from center
    protected static final double CAMERA_TO_CENTER               = -1.8;

    /* Object detection sub system
     */
    protected ObjectIdentificationInterface searchableTarget      = null;
    protected FieldPlacement objectRelativePlacement           = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing the base...");
        super.init();
        telemetry.update();

        /* ************************************
            OBJECT DETECTION
        */
        /* ************************************
            OBJECT IDENTIFICATION
         */

        if (IDENTIFICATION_SYSTEM == "VUFORIA") {
            searchableTarget  = new VuMarkIdentification(
                    hardwareMap,
                    telemetry,
                    TRACKABLE_ASSET_NAME,
                    TRACKABLE_NAME,
                    TRACKABLE_INDEX,
                    CAMERA_SYSTEM,
                    this.DEBUG
            );
        }
        else if (IDENTIFICATION_SYSTEM == "TSF") {
            searchableTarget  = new TensorFlowObjectIdentification(
            hardwareMap,
            telemetry,
            TFOD_MODEL_ASSET,      // identifiable models file ressource
            TFOD_MODEL_ASSETS_LABEL,       // Name of the identifiable asset within the file, important
            TFOD_TARGET_LABEL,
            CAMERA_SYSTEM,
            this.DEBUG);
        }
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("Status", "Make sure motors are free to move");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double forward      = -gamepad1.right_stick_y;
        double right        = gamepad1.right_stick_x;
        double clockwise    = gamepad1.left_stick_x;
        WheelPower wheels   = calcWheelPower(clockwise, forward, right);

        boolean goingForward = forward > 0;
        boolean goingRight = right > 0;
        boolean goingLeft = right < 0;
        boolean goingBack = forward < 0;

        if ( goingForward ){
            dbugThis("Going forward");
        }
        if ( goingRight ){
            dbugThis("Going Right");
        }
        if ( goingLeft ){
            dbugThis("Going Left");
        }
        if ( goingBack ){
            dbugThis("Going Back");
        }

        /**
         * OUTPUT PROPULSION
         */
        // Send calculated power to wheels
        botBase.getFrontLeftDrive().setPower(wheels.front_left);
        botBase.getFrontRightDrive().setPower(wheels.front_right);
        botBase.getRearLeftDrive().setPower(wheels.rear_left);
        botBase.getRearRightDrive().setPower(wheels.rear_right);

        dbugThis(String.format("%.02f,%.02f,%.02f,%.02f", wheels.front_left,wheels.front_right,wheels.rear_left,wheels.rear_right));

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", String.format("Run Time: .2f", runtime.toString()));
        telemetry.addData("Battery Voltage", String.format("%.2f", getBatteryVoltage()));


        // This is the distance from the camera to the actual part of the robot that must align with the center of the Vumark
        double offset = CAMERA_TO_CENTER;
        searchableTarget.find();
        objectRelativePlacement = searchableTarget.getTargetRelativePosition();

        if (objectRelativePlacement != null) {
            double delta = objectRelativePlacement.y - CAMERA_TO_CENTER;
            double absDelta = Math.abs(delta);
            telemetry.addData("delta", String.format("%.2f", delta));
        }

        else {
            telemetry.addData("delta", "Traget not found");
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        super.stop();
    }

}

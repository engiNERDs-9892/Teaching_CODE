package org.firstinspires.ftc.teamcode.AutonomusActions_ExampleCode.Camera_Piplines_And_Use.Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Camera Code Example",group="used")
@Disabled
public class Camrea_Code_Example extends LinearOpMode {
    OpenCvWebcam webcam;
    Example_Pipline.examplePipline pipeline;
    Example_Pipline.examplePipline.Detection_Positions snapshotAnalysis = Example_Pipline.examplePipline.Detection_Positions.RIGHT; // default

    @Override
    public void runOpMode() {

        // HardwareMap Section (Used to talk to the driver hub for the configuration)


        // Camera

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new Example_Pipline.examplePipline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // This is in what viewing window the camera is seeing through and it doesn't matter
                // what orientation it is | UPRIGHT, SIDEWAYS_LEFT, SIDEWAYS_RIGHT, etc.

                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();


        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        telemetry.addData("Status", "\uD83C\uDD97");
        telemetry.update();
        waitForStart();


        // Based on the Example_Pipline Calculations

        switch (snapshotAnalysis) {
            case LEFT: // Level 3
            {

                // IF THE CASE IS LEFT, PERFORM ACTIONS BELOW

                break;

            }


            case RIGHT: // Level 1
            {

                // IF THE CASE IS RIGHT, PERFORM ACTIONS BELOW


                break;
            }

            case CENTER: // Level 2
            {

                // IF THE CASE IS CENTER, PERFORM ACTIONS BELOW

                break;
            }


        }
    }
}
package org.firstinspires.ftc.teamcode.DriverActions_ExampleCode.Advanced.Roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Tuning_Variables_ROADRUNNER.Useful.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tuning_Variables_ROADRUNNER.Useless.PoseStorage;

@TeleOp(group = "advanced")
@Disabled
public class RRMovement_For_Motors_RC_EXAMPLE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for Teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            // This controls the power of your motors for the wheels
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * .7,
                            -gamepad1.left_stick_x * .7,
                            -gamepad1.right_stick_x * .7
                    )
            );

            // Update everything. Odometry. Etc. in order to use RR to drive
            drive.update();

            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.Autos.Parking;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Tuning_Variables.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", preselectTeleOp = "EngiNERDs_Control_RC_V2")
//@Disabled
public class SHORT_PARKING extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence tajpark = drive.trajectorySequenceBuilder(new Pose2d())

                .back(40)
                .build();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(tajpark);

        sleep(20000);
    }
}

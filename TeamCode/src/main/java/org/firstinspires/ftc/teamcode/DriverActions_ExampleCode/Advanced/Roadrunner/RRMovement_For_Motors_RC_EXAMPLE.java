package org.firstinspires.ftc.teamcode.DriverActions_ExampleCode.Advanced.Roadrunner;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Tuning_Variables_ROADRUNNER.Useful.SampleMecanumDrive;

@TeleOp(group = "drive")
//@Disabled
public class RRMovement_For_Motors_RC_EXAMPLE extends LinearOpMode {


    @Override

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();


        while (opModeIsActive()) {


            ////////////////////////////////////////////////////
            // Movement Code ///////////////////////////////////
            ////////////////////////////////////////////////////


            if (gamepad1.right_trigger != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y,
                                gamepad1.left_stick_x,
                                gamepad1.right_stick_x
                        )
                );
            }


            if (gamepad1.left_trigger != 0) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y * .2,
                                gamepad1.left_stick_x * .2,
                                gamepad1.right_stick_x * .2
                        )
                );
            } else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y * .7,
                                gamepad1.left_stick_x * .7,
                                gamepad1.right_stick_x * .7
                        )
                );
            }
        }

    }
}


package org.firstinspires.ftc.teamcode.DriveCode.Current_Code;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.DegreeTorque;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.ShootPlane;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorINTAKE;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorRiseyRise;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables;
import org.firstinspires.ftc.teamcode.Tuning_Variables.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
//@Disabled
public class EngiNERDs_Control_FC_V2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // A way to store values that the gamepad enters
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // A way to store values that gamepad enters
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        new EngiNERDs_Variables(hardwareMap);



        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Stored values of the gamepad inputs
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Stored values of the gamepad inputs
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double RaiseandLower = -gamepad2.left_stick_y;


            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();




            ///////////////////////////////////////////////////////
            // Movement Code  /////////////////////////////////////
            ///////////////////////////////////////////////////////



            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately

            if (gamepad1.right_trigger != 0){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                );
            }

            if (gamepad1.left_trigger != 0){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX() * .3,
                                input.getY() * .3,
                                -gamepad1.right_stick_x *.3
                        )
                );
            }

            else {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX() * .6,
                                input.getY() * .6,
                                -gamepad1.right_stick_x * .6
                        )
                );
            }

            // Update everything. Odometry. Etc.
            drive.update();






            ////////////////////////////////////////////////////////////////////////
            // Arm Playing Mechanism ///////////////////////////////////////////////                                             /
            ////////////////////////////////////////////////////////////////////////


            // Linear Slide Code (Up / Down is in the else statement)
            if (RaiseandLower == 0) {
                motorRiseyRise.setPower(0);
                motorLiftyLift.setPower(0);
            }

            else {
                // move slide up for RaiseandLower < 0, move slide down on RaiseandLower > 0
                motorRiseyRise.setPower(RaiseandLower * 1);
                motorLiftyLift.setPower(RaiseandLower * 1);
            }




            // Wrist Joint Servos
            if (Math.abs(gamepad2.right_stick_y) >= 0.5) {
                WristServoL.setPosition((WristServoL.getPosition() + 0.0005 * Math.signum(-gamepad2.right_stick_y)));
                WristServoR.setPosition((WristServoR.getPosition() + 0.0005 * Math.signum(-gamepad2.right_stick_y)));
            }



            // Airplane Launch Servo
            if (gamepad1.right_bumper) {
                AirplaneLaunchServo.setPosition(ShootPlane * DegreeTorque);
            }



            // Intake Motor
            if (gamepad2.right_trigger != 0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.FORWARD);
                motorINTAKE.setPower(.65);
            }
            else if (gamepad2.left_trigger !=0) {
                motorINTAKE.setDirection(DcMotorSimple.Direction.REVERSE);
                motorINTAKE.setPower(.65);
            }
            else {
                motorINTAKE.setPower(0);
            }















            // Telemetry
            telemetry.addData("LEFT LS POS", motorLiftyLift.getCurrentPosition());
            telemetry.addData("RIGHT LS POS", motorRiseyRise.getCurrentPosition());
            telemetry.addData("WRIST SERVO R POS", WristServoR.getPosition());
            telemetry.addData("WRIST SERVO L POS", WristServoR.getPosition());
            telemetry.addData("LEFT ARM POS", FlooppyFloop.getPosition());
            telemetry.addData("RIGHT ARM POS", FlippyFlip.getPosition());
            telemetry.update();
        }
    }
}


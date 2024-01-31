package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneLaunchServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.AirplaneMountServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeAirplane;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeArm;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeWrist;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.GearServo;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.InitArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.InitArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Open;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.OpenAuto;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.RightClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack1ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack1ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack2ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack2ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack3ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack3ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack4ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack4ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotateFlip;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Stack5ArmRotateFloop;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateGround;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateInit;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.WristRotateStack;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorRiseyRise;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Examples.RedPipline;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.opmode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(group = "advanced", preselectTeleOp = "RESET_HARDWARE")
public class SERVOTEST extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        GearServo = hardwareMap.servo.get("GearServo");

        FlooppyFloop.setDirection(Servo.Direction.FORWARD);


        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)


        waitForStart();
        while (opModeIsActive()) {
            FlooppyFloop.setPosition(Stack1ArmRotateFloop * DegreeArm); // Rotates at an angle forwards
            FlippyFlip.setPosition(Stack1ArmRotateFlip * DegreeArm); // rotates at an angle forwards
            GearServo.setPosition(WristRotateStack * DegreeWrist); // Rotates into the air
        }
    }

    }

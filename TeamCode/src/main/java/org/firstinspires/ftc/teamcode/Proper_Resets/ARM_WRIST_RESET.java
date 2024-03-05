package org.firstinspires.ftc.teamcode.Proper_Resets;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.Degree5Turn;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlippyFlip;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.FlooppyFloop;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoL;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.WristServoR;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "advanced")
public class ARM_WRIST_RESET extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        FlippyFlip = hardwareMap.servo.get("FlippyFlip");
        FlooppyFloop = hardwareMap.servo.get("FlooppyFloop");
        WristServoL = hardwareMap.servo.get("WristServoL");
        WristServoR = hardwareMap.servo.get("WristServoR");

        FlippyFlip.setDirection(Servo.Direction.REVERSE);
        WristServoL.setDirection(Servo.Direction.REVERSE);
        WristServoR.setDirection(Servo.Direction.REVERSE);

        // this initializes the camera (Not going into it tooo much but it initalizes the camera + hw map, and the pipline as well)

        FlippyFlip.setPosition(init * Degree5Turn);
        FlooppyFloop.setPosition(init * Degree5Turn);
        WristServoR.setPosition(init * Degree5Turn);
        WristServoL.setPosition(init * Degree5Turn);
        waitForStart();

        while (opModeIsActive()) {
        }
    }

    }

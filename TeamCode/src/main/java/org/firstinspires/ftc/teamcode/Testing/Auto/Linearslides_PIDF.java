package org.firstinspires.ftc.teamcode.Testing.Auto;

import static org.firstinspires.ftc.teamcode.drive.Variables.Autonomous_Variables.motorRiseyRise;
import static org.firstinspires.ftc.teamcode.drive.Variables.Autonomous_Variables.motorLiftyLift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
//@Disabled
@Autonomous(group = "drive")
public class Linearslides_PIDF extends LinearOpMode {


    private PIDController controller;
    // P = How quickly the thing moves, D = Dampener of how much it slows down at the end (Only 2 you should adjust)
    public static double p = 0, i = 0, d = 0;

    // Feedforward Component of the linear slides
    public static double f = 0;

    public static int target1 = 700;
    public static int target2 = 500;

    public final double ticks_in_degrees = 537.6 / 180;
    @Override
    public void runOpMode() {

        controller = new PIDController(p, i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motorLiftyLift = hardwareMap.get(DcMotor.class,"motorLiftyLift");
        motorRiseyRise = hardwareMap.get(DcMotor.class,"motorRiseyRise");


        waitForStart();


        controller.setPID(p,i,d);
        int LinearSlide_Pos1 = motorRiseyRise.getCurrentPosition();
        int LinearSlide_Pos2 = motorLiftyLift.getCurrentPosition();

        double pid = controller.calculate(LinearSlide_Pos1, target1);
        double ff = Math.cos(Math.toRadians(target1 / ticks_in_degrees)) * f;

        double power = pid + ff;

        motorRiseyRise.setPower(power);
        motorLiftyLift.setPower(power);

        telemetry.addData("RiseyRise Pos", LinearSlide_Pos1);
        telemetry.addData("LiftyLift Pos", LinearSlide_Pos2);
        telemetry.addData("Target Pos", target1);
        telemetry.update();


    }
    }

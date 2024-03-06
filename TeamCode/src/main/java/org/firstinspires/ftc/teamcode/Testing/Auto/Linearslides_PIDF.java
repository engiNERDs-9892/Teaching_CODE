package org.firstinspires.ftc.teamcode.Testing.Auto;

import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorRiseyRise;
import static org.firstinspires.ftc.teamcode.Tuning_Variables.EngiNERDs_Variables.motorLiftyLift;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Disabled
@Autonomous(group = "drive")
public class Linearslides_PIDF extends OpMode {

    private PIDController controller;

    // Variables for the LS motors
    public static double P = 0.021, I = 0, D = 0.0004;

    // Feedforward Component of the linear slides
    public static double f = 0;

    public static int target = 0;

    public final double ticks_in_degrees = 751.8  / 180;

        @Override
        public void init() {
            controller = new PIDController(P, I,D);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            motorLiftyLift = hardwareMap.get(DcMotor.class,"motorLiftyLift");
            motorRiseyRise = hardwareMap.get(DcMotor.class,"motorRiseyRise");

            motorLiftyLift.setDirection(DcMotorSimple.Direction.REVERSE);

            target = 0;
}

        @Override
        public void loop() {

            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift

            controller.setPID(P, I,D);
            int LinearSlide_Pos1 = motorRiseyRise.getCurrentPosition();
            int LinearSlide_Pos2 = motorLiftyLift.getCurrentPosition();

            double pid = controller.calculate(LinearSlide_Pos1,target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double power = pid + ff;

            motorRiseyRise.setPower(power);
            motorLiftyLift.setPower(power);

            telemetry.addData("Risey Rise Pos", LinearSlide_Pos1);
            telemetry.addData("LiftyLift Pos", LinearSlide_Pos2);
            telemetry.addData("Target Pos", target);
            telemetry.update();}

    }


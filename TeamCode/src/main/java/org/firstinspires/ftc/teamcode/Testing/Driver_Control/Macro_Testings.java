package org.firstinspires.ftc.teamcode.Testing.Driver_Control;


import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.Close;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.DegreeClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.LeftClaw;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorLiftyLift;
import static org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables.motorRiseyRise;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Variables.EngiNERDs_Variables;

@TeleOp(name="FSM Example")
public class Macro_Testings extends LinearOpMode {
    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.LIFT_START;

    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode

    // the dump servo
    public Servo liftDump;
    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    // the amount of time the dump servo takes to activate in seconds
    final double DUMP_TIME = 4;

    final int LIFT_LOW = 0; // the low encoder position for the lift


    @Override
    public void runOpMode() {
        new EngiNERDs_Variables(hardwareMap);

        liftTimer.reset();


        motorLiftyLift.setTargetPosition(LIFT_LOW);
        motorLiftyLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRiseyRise.setTargetPosition(LIFT_LOW);
        motorRiseyRise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    waitForStart();

    while (opModeIsActive()){

        switch (liftState) {
            case LIFT_START:
                // Waiting for some input
                if (gamepad1.square) {
                    // x is pressed, start extending
                    LeftClaw.setPosition(0.15);
                    liftTimer.reset();
                    liftState = LiftState.LIFT_DUMP;
                }
                break;
            case LIFT_DUMP:
                if (liftTimer.seconds() >= DUMP_TIME) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    LeftClaw.setPosition(Close * DegreeClaw );
                    liftState = LiftState.LIFT_START;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.LIFT_START;
        }


        if (gamepad1.y && liftState != LiftState.LIFT_START) {
            liftState = LiftState.LIFT_START;
        }
    telemetry.addData("LeftCLaw POS", LeftClaw.getPosition());
        telemetry.update();
    }
    }
}
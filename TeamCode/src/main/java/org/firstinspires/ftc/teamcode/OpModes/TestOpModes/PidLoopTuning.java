package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.teamcode.Modules.PidController;

@TeleOp(name="PID tuning", group="Linear OpMode")
public class PidLoopTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        PidController PID = new PidController(hardwareMap, 200f, true);

        float prop = 0.0f;
        PID.setProportional(prop);

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                prop += 0.1f;
                PID.setProportional(prop);
            }
            if (gamepad1.bWasPressed()) {
                prop -= 0.1f;
                PID.setProportional(prop);
            }
            PID.Loop();
            PID.getTelemetry().addData("Kp", prop);
            PID.updateTelemetry();
        }
    }
}

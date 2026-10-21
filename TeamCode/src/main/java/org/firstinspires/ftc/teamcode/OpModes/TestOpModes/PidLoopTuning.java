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

        // actual ultimate gain is 3.0 -> 3.1
        float ultimate_gain = 0f;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                ultimate_gain += 0.05f;
            }
            if (gamepad1.bWasPressed()) {
                ultimate_gain -= 0.05f;
            }
            PID.setPidForTuning(ultimate_gain, 0, 0);

            PID.Loop();
            PID.getTelemetry().addData("Kp", ultimate_gain);
            PID.getTelemetry().addData("bounds_bottom", 0);
            PID.getTelemetry().addData("bounds_top", 312);
            PID.updateTelemetry();
        }
    }
}

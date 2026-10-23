package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.PidController;

@TeleOp(name="RPM accuracy determiner", group="Linear OpMode")
public class RpmAccuracyChecker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PidController pidController;

        waitForStart();

        pidController = new PidController(hardwareMap, 200f, true);

        while (opModeIsActive()) {
            pidController.Loop();
        }
    }
}

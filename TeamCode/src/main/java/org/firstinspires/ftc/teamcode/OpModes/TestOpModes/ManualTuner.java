package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PidController;

@TeleOp(name="Manual Tuner", group="Linear OpMode")
public class ManualTuner extends LinearOpMode {

    public enum targets {
        Lowest(0),
        KindaLow(100),
        KindaHigh(200),
        Highest(300);
        float RPM;

        targets(float RPM) {
            this.RPM = RPM;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        PidController PID = new PidController(hardwareMap, 200f, true);
        Telemetry telemetry1 = FtcDashboard.getInstance().getTelemetry();
        float Proportional = 1;
        float Integral = 0;
        float Derivative = 0;

        while (opModeIsActive()) {
            int i = 0;
            i++;

            Proportional += (0.005f * gamepad1.left_stick_y);
            Integral += (0.005f * gamepad1.right_stick_y);
            Derivative += (0.005f * (gamepad1.left_trigger - gamepad1.right_trigger));
            PID.setPidForTuning(Proportional,Integral,Derivative);
            telemetry1.addData("P", PID.Kp);
            telemetry1.addData("I", PID.Ki);
            telemetry1.addData("D", PID.Kd);
            PID.Loop();
        }
    }
}

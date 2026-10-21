package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Time scale determiner", group="Linear OpMode")
public class TimeScaleDetector extends LinearOpMode {
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        elapsedTime = new ElapsedTime(0);
        elapsedTime.reset();

        while (opModeIsActive()) {
            FtcDashboard.getInstance().getTelemetry().addData("bottom", 0);
            FtcDashboard.getInstance().getTelemetry().addData("top", 312);
            FtcDashboard.getInstance().getTelemetry().addData("time", elapsedTime.milliseconds() / 10);
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}

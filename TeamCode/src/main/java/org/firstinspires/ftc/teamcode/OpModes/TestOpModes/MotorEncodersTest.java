package org.firstinspires.ftc.teamcode.OpModes.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Motor Encoder Test", group="Linear OpMode")
public class MotorEncodersTest extends LinearOpMode {

    DcMotor motor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class, "PidMotor");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(1);

//            telemetry.addData("Encoder rots", motor.getCurrentPosition() / 537.7);
//            telemetry.update();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            dashboardTelemetry.addData("EncoderCount", motor.getCurrentPosition() / 537.7);
            dashboardTelemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID.PidController;

public class PidSingleMotor extends PidController {

    // Motor //
    private final DcMotor Motor;
    private final float encodersPerRevolution = 537.7f;
    private final float maxRPM = 312;

    // process value calculations //
    private float motorPreviousEncoderCount = 0;

    public PidSingleMotor(HardwareMap HwMap, Float Target, int sampleSpeed) {

        // called to appease PidController constructor
        super(Target, sampleSpeed);

        Motor = HwMap.get(DcMotor.class, "PidMotor");
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    protected void customLoopControls() {
        Motor.setPower(Motor.getPower() + remap(0, maxRPM, 0, 1, getCalculatedUt()));

        getDashboardTelemetry().addData("bounds_top", maxRPM);
        getDashboardTelemetry().addData("Motor Power (u(t))", Motor.getPower());
    }

    @Override
    protected float CalculateProcessVariable(float deltaTime) {

        // calculate the process variable
        float processVariable =  ((Motor.getCurrentPosition() - motorPreviousEncoderCount) / encodersPerRevolution);
        processVariable *=  (60000 / deltaTime);

        // set previous values to be read next [function call] -> (iteration)
        motorPreviousEncoderCount = Motor.getCurrentPosition();
        return processVariable;
    }
}

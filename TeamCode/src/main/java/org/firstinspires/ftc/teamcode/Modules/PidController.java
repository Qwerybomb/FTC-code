package org.firstinspires.ftc.teamcode.Modules;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PidController {

    // Pre-Calculated Gains //
    public Gains gains = new Gains(0,0,0);

    // Timings (all in ms) //
    private float activeDeltaTime = 0;
    private float previousTimePoint = 0;
    private float sampleTimingPrevious = 0;
    private float sampleTime = 50;
    private ElapsedTime time;

    // Motor //
    private final DcMotor Motor;
    private final float encodersPerRevolution = 537.7f;
    private final float maxRPM = 312;

    // integration //
    private float integral = 0;

    // Error visualization helper //
    Telemetry telemetry;
    boolean printTelemetry;

    // Error & Error calculatory values //
    public float Target;
    private float currentError;
    private float previousError;

    // process value calculations //
    private float motorPreviousEncoderCount = 0;

    /**
     * Assign initial values for all robot or externally dependent objects
     * @param HwMap Hardware map of the robot, from which to retrieve the motor
     * @param Target Desired Initial Target value
     * @param printTelemetry boolean to define whether or not telemetry should be printed
     */
    public PidController(HardwareMap HwMap, Float Target, boolean printTelemetry) {
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
        this.printTelemetry = printTelemetry;

        updateTarget(Target);

        Motor = HwMap.get(DcMotor.class, "PidMotor");
        Motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // precautions
        time = new ElapsedTime(0);
        time.reset();
    }

    /**
     * Primary control loop for the PID mechanism.
     * The telemetry for the dashboard is updated here. for custom telemetry additions,
     * please add them before calling Loop();
     */
    public void Loop() {

        if (sampleTimingPrevious + sampleTime > time.milliseconds()) {
            return;
        }
        sampleTimingPrevious = (float) time.milliseconds();

        // calculate the active delta time.
        activeDeltaTime = (float) (time.milliseconds() - previousTimePoint);
        previousTimePoint = (float) time.milliseconds();

        // set the error last iteration to the error this iteration
        previousError = currentError;

        // set the current error to the target - PV
        currentError = Target - CalculateProcessVariable(activeDeltaTime);

        float calculated_power = CalculateProportional() + CalculateDerivative() + CalculateIntegral();
        Motor.setPower(Motor.getPower() + remap(0, maxRPM, 0, 1, calculated_power));

        // prints out telemetry. bottom target and top define the ranges
        if (printTelemetry) {
            telemetry.addData("bounds_bottom", 0);
            telemetry.addData("target", Target);
            telemetry.addData("bounds_top", maxRPM);
            telemetry.addData("currentError", currentError);
            telemetry.addData("Process Variable (RPM)", Target - currentError);
            telemetry.addData("Motor Power (u(t))", Motor.getPower());
            telemetry.addData("current delta time", activeDeltaTime);
            telemetry.update();
        }
    }

    /**
     * function to calculate the P in PID
     * @return the P in PID
     */
    private float CalculateProportional() {
        return gains.Kp * currentError;
    }

    /**
     * function to calculate the I in PID. Reminder -> All time in Ms
     * @return the I in PID
     */
    private float CalculateIntegral() {
        integral = gains.Ki * ((integral + currentError) * activeDeltaTime);
        return integral;
    }

    /**
     * function to calculate the D in PID
     * @return the D in PID
     */
    private float CalculateDerivative() {
        return gains.Kd * (currentError - previousError) / activeDeltaTime;
    }

    /**
     * Calculate the Process Variable (RPM) using the previously known encoder count and the amount of encoder counts per revolution, as well at the known time-per-iteration (could be improved with added deltaTime).
     * @param deltaTime the time the previous iteration took to process in milliseconds.
     * @return Process Variable
     */
    private float CalculateProcessVariable(float deltaTime) {

        // calculate the process variable
        float processVariable =  ((Motor.getCurrentPosition() - motorPreviousEncoderCount) / encodersPerRevolution);
        processVariable *=  (60000 / deltaTime);

        // set previous values to be read next [function call] -> (iteration)
        motorPreviousEncoderCount = Motor.getCurrentPosition();
        return processVariable;
    }

    /**
     * remaps values from one range A to range B. example, a value of 500 in a 0-1000 scale can be remapped to 0.5 on a 0-1 scale.
     * @param low1 range A lower expected
     * @param high1 range A higher expected
     * @param low2 range B lower expected
     * @param high2 range B higher expected
     * @param value value to remap.
     * @return remapped value
     */
    private float remap(float low1, float high1, float low2, float high2, float value) {
        return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
    }

    /**
     * Change the current target into a new Target.
     * @param Target The new Target
     */
    public void updateTarget(Float Target) {
        this.Target = Target;
    }
}

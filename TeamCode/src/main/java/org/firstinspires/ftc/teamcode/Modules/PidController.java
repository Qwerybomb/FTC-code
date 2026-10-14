package org.firstinspires.ftc.teamcode.Modules;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PidController {

    // Pre-Calculated Gains //
    private final float Kp = 0;
    private final float Ki = 0;
    private final float Kd = 0;

    // Timings (all in ms) //
    private final float Dt = 100;

    // Motor //
    private final DcMotor Motor;
    private final float encodersPerRevolution = 537.7f;

    // integration //
    private float integral = 0;

    // Error visualization helper //
    Telemetry telemetry;
    boolean printTelemetry;

    // Error & Error calculatory values //
    private float Target;
    private float currentError;
    private float previousError;

    // process value calculations //
    private float motorPreviousEncoderCount = 0;

    /**
     * Assign initial values for all robot or externally dependent objects
     * @param HwMap Hardware map of the robot, from which to retrieve the motor
     * @param telemetry Telemetry for the Driver Station
     * @param Target Desired Initial Target value
     */
    public PidController(HardwareMap HwMap, Telemetry telemetry, Float Target, boolean printTelemetry) {
        this.telemetry = telemetry;
        this.printTelemetry = printTelemetry;

        UpdateTarget(Target);
        Motor = HwMap.get(DcMotor.class, "PidMotor");
    }

    /**
     * Primary control loop for the PID mechanism.
     */
    public void PidLoop() {
        previousError = currentError;
        currentError = Target - CalculateProcessVariable();
        Motor.setPower(CalculateProportional() + CalculateDerivative() + CalculateIntegral());

        if (printTelemetry) {
            telemetry.addData("currentError", currentError);
            telemetry.addData("Process Variable", Target - currentError);
            telemetry.addData("Motor Power (u(t))", Motor.getPower());
            telemetry.update();
        }

        try {
            Thread.sleep((long) Dt);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * function to calculate the P in PID
     * @return the P in PID
     */
    private float CalculateProportional() {
        return Kp * currentError;
    }

    /**
     * function to calculate the I in PID. Reminder -> All time in Ms
     * @return the I in PID
     */
    private float CalculateIntegral() {
        integral = Ki * ((integral + currentError) * Dt);
        return integral;
    }

    /**
     * function to calculate the D in PID
     * @return the D in PID
     */
    private float CalculateDerivative() {
        return Kd * (currentError - previousError) / Dt;
    }

    /**
     * Calculate the Process Variable (RPM) using the previously known encoder count and the amount of encoder counts per revolution, as well at the known time-per-iteration (could be improved with added deltaTime).
     * @return Process Variable
     */
    private float CalculateProcessVariable() {

        // calculate the process variable
        float processVariable =  ((Motor.getCurrentPosition() - motorPreviousEncoderCount) / encodersPerRevolution);
        processVariable *=  (60000 / Dt);

        // set previous values to be read next [function call] -> (iteration)
        motorPreviousEncoderCount = Motor.getCurrentPosition();
        return processVariable;
    }

    /**
     * Change the current target into a new Target.
     * @param Target The new Target
     */
    public void UpdateTarget(Float Target) {
        this.Target = Target;
    }
}

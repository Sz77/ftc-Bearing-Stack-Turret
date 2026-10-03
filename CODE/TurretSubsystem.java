package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * FTC Bearing Stack Turret Subsystem
 *
 * --- WIRING GUIDE (CONTROL HUB / EXPANSION HUB) ---
 * Servos:
 * - Port 0: "servoA1" (Continuous Rotation Servo on 25T gear)
 * - Port 1: "servoA2" (Continuous Rotation Servo on 25T gear)
 *
 * Analog Inputs (Axon Mini Absolute Encoders output 3.3V max):
 * - Port 0: "encoderA1" (Attached to servoA1's 25T gear)
 * - Port 1: "encoderA2" (Attached to servoA2's 25T gear)
 * - Port 2: "encoderB"  (Attached to the 20T gear)
 *
 * --- HOW TO ZERO THE ENCODERS ---
 * 1. Deploy the code with offsets set to 0.0.
 * 2. Physically rotate the turret by hand so it is facing exactly straight forward (0 degrees).
 * 3. Look at the Driver Station Telemetry to find "Raw A1", "Raw A2", and "Raw B".
 * 4. Copy those exact numbers into the OFFSET constants below.
 * 5. Re-deploy the code. The turret will now read exactly 0 at that position.
 */
public class TurretSubsystem {

    // --- HARDWARE CONFIG NAMES ---
    public static final String NAME_SERVO_A1 = "servoA1";
    public static final String NAME_SERVO_A2 = "servoA2";
    public static final String NAME_ENCODER_A1 = "encoderA1";
    public static final String NAME_ENCODER_A2 = "encoderA2";
    public static final String NAME_ENCODER_B = "encoderB";

    // --- ENCODER CONSTANTS ---
    public static final double ENCODER_MAX_VOLTAGE = 3.3;
    public static final double MAX_DEGREES = 360.0;

    // TODO: Enter your raw zero values here after mechanical alignment
    public static final double OFFSET_A1 = 0.0;
    public static final double OFFSET_A2 = 0.0;
    public static final double OFFSET_B  = 0.0;

    // --- TURRET LIMIT CONSTANTS ---
    public static final double MIN_TURRET_ANGLE_DEG = -180.0;
    public static final double MAX_TURRET_ANGLE_DEG = 180.0;
    public static final double HARD_STOP_BUFFER_DEG = 179.0; // Stop driving if we cross this to protect wires

    // --- MOTOR POWER CONSTANTS ---
    public static final double MAX_SERVO_POWER = 1.0;
    public static final double MIN_SERVO_POWER = -1.0;
    public static final double STOP_POWER = 0.0;

    // --- PID CONSTANTS ---
    // Tune these values to get smooth, accurate movement without oscillation.
    public static final double kP = 0.015;
    public static final double kI = 0.000;
    public static final double kD = 0.0001;

    // Limit for the integral sum to prevent integral windup
    public static final double MAX_INTEGRAL_SUM = 20.0;
    // Only accumulate Integral when within this many degrees of the target
    public static final double INTEGRAL_ACTIVE_ZONE_DEG = 15.0;

    // Hardware variables
    private CRServo servoA1, servoA2;
    private AnalogInput encoderA1, encoderA2, encoderB;

    // PID variables
    private ElapsedTime pidTimer;
    private double lastError = 0.0;
    private double integralSum = 0.0;

    public TurretSubsystem(HardwareMap hwMap) {
        servoA1 = hwMap.get(CRServo.class, NAME_SERVO_A1);
        servoA2 = hwMap.get(CRServo.class, NAME_SERVO_A2);

        // Uncomment and change direction if servos are physically mirrored
        // servoA2.setDirection(CRServo.Direction.REVERSE);

        encoderA1 = hwMap.get(AnalogInput.class, NAME_ENCODER_A1);
        encoderA2 = hwMap.get(AnalogInput.class, NAME_ENCODER_A2);
        encoderB = hwMap.get(AnalogInput.class, NAME_ENCODER_B);

        pidTimer = new ElapsedTime();
    }

    /**
     * Converts raw analog voltage (0-3.3v) into normalized degrees (0-360) factoring in the offset.
     */
    private double getNormalizedAngle(AnalogInput encoder, double offset) {
        double rawVolt = encoder.getVoltage();
        double rawDeg = (rawVolt / ENCODER_MAX_VOLTAGE) * MAX_DEGREES;
        double adjustedDeg = (rawDeg - offset) % MAX_DEGREES;

        if (adjustedDeg < 0) adjustedDeg += MAX_DEGREES;
        return adjustedDeg;
    }

    /**
     * Uses circular math to average the two 25T encoders accurately.
     */
    private double getAverageAngleA() {
        double a1 = Math.toRadians(getNormalizedAngle(encoderA1, OFFSET_A1));
        double a2 = Math.toRadians(getNormalizedAngle(encoderA2, OFFSET_A2));

        double x = Math.cos(a1) + Math.cos(a2);
        double y = Math.sin(a1) + Math.sin(a2);

        double avgDeg = Math.toDegrees(Math.atan2(y, x));

        if (avgDeg < 0) avgDeg += MAX_DEGREES;
        return avgDeg;
    }

    /**
     * Calculates the absolute turret position using the 5:1 and 4:1 gear difference.
     * Returns a value between -180 and 180.
     */
    public double getTurretAngle() {
        double angleB = getNormalizedAngle(encoderB, OFFSET_B);
        double avgAngleA = getAverageAngleA();

        // 5x gear angle - 4x gear angle = 1x turret angle
        double turretAngle = (angleB - avgAngleA) % MAX_DEGREES;

        if (turretAngle < 0) turretAngle += MAX_DEGREES;

        // Shift domain to [-180, 180) to respect the physical limits
        if (turretAngle > MAX_TURRET_ANGLE_DEG) {
            turretAngle -= MAX_DEGREES;
        }

        return turretAngle;
    }

    /**
     * Drives the turret to a target angle using a PID controller.
     * Takes the "long way" if switching between positive and negative extremes
     * to prevent wire-tangling across the 180/-180 boundary.
     */
    public void setTargetAngle(double targetAngle) {
        // Enforce maximum boundaries on the target
        targetAngle = Math.max(MIN_TURRET_ANGLE_DEG, Math.min(MAX_TURRET_ANGLE_DEG, targetAngle));

        double currentAngle = getTurretAngle();

        // --- LONG WAY PATHING ---
        // By NOT using an angleWrap() function here, an error from 170 to -170
        // evaluates as -340 degrees (rather than +20 degrees). This forces the
        // turret to travel back through 0, naturally taking the "long way" and
        // avoiding crossing the hard stop.
        double error = targetAngle - currentAngle;

        // PID time delta calculation
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Calculate Integral only if we are close to the target.
        // Since taking the "long way" generates massive errors (up to 360),
        // accumulating the I-term during long travel causes dangerous integral windup.
        if (Math.abs(error) < INTEGRAL_ACTIVE_ZONE_DEG) {
            integralSum += (error * dt);
            // Anti-windup clamping
            integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(MAX_INTEGRAL_SUM, integralSum));
        } else {
            integralSum = 0; // Reset I-term while doing long-distance travel
        }

        double derivative = 0;
        if (dt > 0) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        // Calculate base PID power
        double power = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Clip power to valid servo range [-1.0, 1.0]
        power = Math.max(MIN_SERVO_POWER, Math.min(MAX_SERVO_POWER, power));

        // Hard stops to prevent breaking mechanism physically
        if (currentAngle >= HARD_STOP_BUFFER_DEG && power > STOP_POWER) power = STOP_POWER;
        if (currentAngle <= -HARD_STOP_BUFFER_DEG && power < STOP_POWER) power = STOP_POWER;

        // Apply power to servos
        servoA1.setPower(power);
        servoA2.setPower(power);
    }

    public void printCalibrationTelemetry(Telemetry telemetry) {
        telemetry.addData("Raw A1", (encoderA1.getVoltage() / ENCODER_MAX_VOLTAGE) * MAX_DEGREES);
        telemetry.addData("Raw A2", (encoderA2.getVoltage() / ENCODER_MAX_VOLTAGE) * MAX_DEGREES);
        telemetry.addData("Raw B", (encoderB.getVoltage() / ENCODER_MAX_VOLTAGE) * MAX_DEGREES);
        telemetry.addData("Current Turret Angle", getTurretAngle());
    }
}

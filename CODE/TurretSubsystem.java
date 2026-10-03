package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {

    // --- HARDWARE CONFIG NAMES ---
    private static final String NAME_SERVO_A1 = "servoA1";
    private static final String NAME_SERVO_A2 = "servoA2";
    private static final String NAME_ENCODER_A1 = "encoderA1";
    private static final String NAME_ENCODER_A2 = "encoderA2";
    private static final String NAME_ENCODER_B = "encoderB";

    // --- POLARITY / INVERSION CONSTANTS ---
    public static final boolean INVERT_SERVO_A1 = false;
    public static final boolean INVERT_SERVO_A2 = true;

    public static final boolean INVERT_ENCODER_A1 = false;
    public static final boolean INVERT_ENCODER_A2 = false;
    public static final boolean INVERT_ENCODER_B  = false;

    // --- ENCODER MATH CONSTANTS ---
    private static final double ENCODER_MAX_VOLTAGE = 3.3;
    private static final double MAX_DEGREES = 360.0;

    // Paste the outputs from the Calibration OpMode here
    public static final double OFFSET_A1 = 0.0;
    public static final double OFFSET_A2 = 0.0;
    public static final double OFFSET_B  = 0.0;

    // --- TURRET LIMIT CONSTANTS (SOFTWARE BUMPERS) ---
    // Safely restricted to prevent reaching the physical wire limits or the 360 math wrap
    public static final double MIN_TURRET_ANGLE_DEG = 10.0;
    public static final double MAX_TURRET_ANGLE_DEG = 350.0;

    // Absolute limits: Motor power is completely cut if momentum carries it here
    public static final double HARD_STOP_BUFFER_MIN = 5.0;
    public static final double HARD_STOP_BUFFER_MAX = 355.0;

    // --- NOISE FILTER & TOLERANCE CONSTANTS ---
    public static final double FILTER_ALPHA = 0.75;
    public static final double TARGET_TOLERANCE_DEG = 1.5;

    // --- MOTOR POWER CONSTANTS ---
    private static final double MAX_SERVO_POWER = 1.0;
    private static final double MIN_SERVO_POWER = -1.0;
    private static final double STOP_POWER = 0.0;

    // --- PID CONSTANTS ---
    public static final double kP = 0.015;
    public static final double kI = 0.000;
    public static final double kD = 0.0001;
    public static final double MAX_INTEGRAL_SUM = 20.0;
    public static final double INTEGRAL_ACTIVE_ZONE_DEG = 15.0;

    private final CRServo servoA1;
    private final CRServo servoA2;
    private final AnalogInput encoderA1;
    private final AnalogInput encoderA2;
    private final AnalogInput encoderB;

    private final ElapsedTime pidTimer;
    private double lastError = 0.0;
    private double integralSum = 0.0;

    private double prevRawAngle = 0.0;
    private double unwrappedAngle = 0.0;
    private double filteredTurretAngle = 0.0;
    private boolean angleInitialized = false;

    public TurretSubsystem(HardwareMap hwMap) {
        servoA1 = hwMap.get(CRServo.class, NAME_SERVO_A1);
        servoA2 = hwMap.get(CRServo.class, NAME_SERVO_A2);

        if (INVERT_SERVO_A1) servoA1.setDirection(DcMotorSimple.Direction.REVERSE);
        if (INVERT_SERVO_A2) servoA2.setDirection(DcMotorSimple.Direction.REVERSE);

        encoderA1 = hwMap.get(AnalogInput.class, NAME_ENCODER_A1);
        encoderA2 = hwMap.get(AnalogInput.class, NAME_ENCODER_A2);
        encoderB = hwMap.get(AnalogInput.class, NAME_ENCODER_B);

        pidTimer = new ElapsedTime();
    }

    private double trueModulo(double value, double mod) {
        return ((value % mod) + mod) % mod;
    }

    private double getNormalizedAngle(AnalogInput encoder, double offset, boolean invert) {
        double rawDeg = (encoder.getVoltage() / ENCODER_MAX_VOLTAGE) * MAX_DEGREES;
        if (invert) {
            rawDeg = MAX_DEGREES - rawDeg;
        }
        return trueModulo(rawDeg - offset, MAX_DEGREES);
    }

    private double getAverageAngleA() {
        double a1 = Math.toRadians(getNormalizedAngle(encoderA1, OFFSET_A1, INVERT_ENCODER_A1));
        double a2 = Math.toRadians(getNormalizedAngle(encoderA2, OFFSET_A2, INVERT_ENCODER_A2));

        double x = Math.cos(a1) + Math.cos(a2);
        double y = Math.sin(a1) + Math.sin(a2);

        double avgDeg = Math.toDegrees(Math.atan2(y, x));
        return trueModulo(avgDeg, MAX_DEGREES);
    }

    /**
     * Reads hardware sensors, unwraps, and applies filters.
     * MUST be called exactly once per loop in the OpMode.
     */
    public void readSensors() {
        double angleB = getNormalizedAngle(encoderB, OFFSET_B, INVERT_ENCODER_B);
        double avgAngleA = getAverageAngleA();

        double rawTurret = trueModulo(angleB - avgAngleA, MAX_DEGREES);

        if (!angleInitialized) {
            prevRawAngle = rawTurret;
            unwrappedAngle = rawTurret;
            filteredTurretAngle = rawTurret;
            angleInitialized = true;
        } else {
            double delta = rawTurret - prevRawAngle;

            if (delta > 180.0) delta -= MAX_DEGREES;
            else if (delta < -180.0) delta += MAX_DEGREES;

            unwrappedAngle += delta;
            prevRawAngle = rawTurret;
        }

        filteredTurretAngle = (FILTER_ALPHA * unwrappedAngle) + ((1.0 - FILTER_ALPHA) * filteredTurretAngle);
    }

    /**
     * Pure getter for the turret angle. Does not mutate state or trigger hardware reads.
     */
    public double getTurretAngle() {
        return filteredTurretAngle;
    }

    public void setTargetAngle(double targetAngle) {
        // Enforce the safe Software Bumpers
        targetAngle = Math.max(MIN_TURRET_ANGLE_DEG, Math.min(MAX_TURRET_ANGLE_DEG, targetAngle));

        double currentAngle = getTurretAngle();
        double error = targetAngle - currentAngle;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        if (Math.abs(error) <= TARGET_TOLERANCE_DEG) {
            servoA1.setPower(STOP_POWER);
            servoA2.setPower(STOP_POWER);
            integralSum = 0;
            lastError = error;
            return;
        }

        if (Math.abs(error) < INTEGRAL_ACTIVE_ZONE_DEG) {
            integralSum += (error * dt);
            integralSum = Math.max(-MAX_INTEGRAL_SUM, Math.min(MAX_INTEGRAL_SUM, integralSum));
        } else {
            integralSum = 0;
        }

        double derivative = 0;
        if (dt > 0.005) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        double power = (kP * error) + (kI * integralSum) + (kD * derivative);

        // DIRECTIONAL HARD CLAMP WITH WINDUP CLEAR
        if (currentAngle >= HARD_STOP_BUFFER_MAX) {
            if (power > STOP_POWER) integralSum = 0; // Prevent windup if clamping prevents reaching target
            power = Math.min(power, STOP_POWER);
        }
        if (currentAngle <= HARD_STOP_BUFFER_MIN) {
            if (power < STOP_POWER) integralSum = 0; // Prevent windup if clamping prevents reaching target
            power = Math.max(power, STOP_POWER);
        }

        power = Math.max(MIN_SERVO_POWER, Math.min(MAX_SERVO_POWER, power));

        servoA1.setPower(power);
        servoA2.setPower(power);
    }

    public void setRawMotorPowerA1(double power) { servoA1.setPower(power); }
    public void setRawMotorPowerA2(double power) { servoA2.setPower(power); }
    public void stopMotors() { servoA1.setPower(STOP_POWER); servoA2.setPower(STOP_POWER); }

    public double getRawDegreeA1() { return (encoderA1.getVoltage() / ENCODER_MAX_VOLTAGE) * MAX_DEGREES; }
    public double getRawDegreeA2() { return (encoderA2.getVoltage() / ENCODER_MAX_VOLTAGE) * MAX_DEGREES; }
    public double getRawDegreeB()  { return (encoderB.getVoltage() / ENCODER_MAX_VOLTAGE) * MAX_DEGREES; }

    public void printCalibrationTelemetry(Telemetry telemetry) {
        telemetry.addData("Raw A1", "%.2f", getRawDegreeA1());
        telemetry.addData("Raw A2", "%.2f", getRawDegreeA2());
        telemetry.addData("Raw B", "%.2f", getRawDegreeB());
        telemetry.addData("Filtered Turret Angle", "%.2f", getTurretAngle());
    }
}

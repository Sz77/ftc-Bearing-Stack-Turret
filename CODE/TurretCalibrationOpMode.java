package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Turret Calibration & Testing", group = "Testing")
public class TurretCalibrationOpMode extends LinearOpMode {

    private enum State {
        SERVO_TEST,
        ENCODER_POLARITY_TEST,
        OFFSET_CAPTURE,
        MANUAL_SWEEP
    }

    @Override
    public void runOpMode() {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
        State currentState = State.SERVO_TEST;

        double capturedOffsetA1 = 0.0;
        double capturedOffsetA2 = 0.0;
        double capturedOffsetB = 0.0;

        double maxSweepAngle = 180.0;
        double minSweepAngle = 180.0;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        turret.readSensors(); // Initialize filter states before loop

        telemetry.addLine("Turret Calibration OpMode Initialized.");
        telemetry.addLine("WARNING: Keep turret within safe wire boundaries!");
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // Run exactly once per loop to continuously track wrap-arounds regardless of state
            turret.readSensors();

            switch (currentState) {

                case SERVO_TEST:
                    telemetry.addLine("--- STEP 1: SERVO DIRECTION ---");
                    telemetry.addLine("Push BOTH sticks UP. The turret MUST spin in the SAME direction.");
                    telemetry.addLine("If they fight, flip INVERT_SERVO_A2 in Subsystem.");
                    telemetry.addLine("Press 'A' to move to next step.");

                    turret.setRawMotorPowerA1(-currentGamepad1.left_stick_y);
                    turret.setRawMotorPowerA2(-currentGamepad1.right_stick_y);

                    if (currentGamepad1.a && !previousGamepad1.a) {
                        turret.stopMotors();
                        currentState = State.ENCODER_POLARITY_TEST;
                    }
                    break;

                case ENCODER_POLARITY_TEST:
                    turret.stopMotors();
                    telemetry.addLine("--- STEP 2: ENCODER POLARITY ---");
                    telemetry.addLine("Turn the turret slowly by hand to the RIGHT.");
                    telemetry.addLine("ALL THREE numbers below should INCREASE.");
                    telemetry.addLine("If one decreases, change its INVERT_ENCODER const to true.");
                    telemetry.addLine("");
                    telemetry.addData("Raw A1", "%.2f", turret.getRawDegreeA1());
                    telemetry.addData("Raw A2", "%.2f", turret.getRawDegreeA2());
                    telemetry.addData("Raw B", "%.2f", turret.getRawDegreeB());
                    telemetry.addLine("");
                    telemetry.addLine("Press 'B' when all 3 increase together.");

                    if (currentGamepad1.b && !previousGamepad1.b) {
                        currentState = State.OFFSET_CAPTURE;
                    }
                    break;

                case OFFSET_CAPTURE:
                    turret.stopMotors();
                    telemetry.addLine("--- STEP 3: CAPTURE ZERO OFFSETS ---");
                    telemetry.addLine("Push the turret to the safest leftmost physical limit you want as '0'.");
                    telemetry.addLine("DO NOT wrap the wires!");
                    telemetry.addLine("Hold it, then press 'X' to capture offsets.");
                    telemetry.addLine("");
                    telemetry.addData("Live Raw A1", "%.2f", turret.getRawDegreeA1());
                    telemetry.addData("Live Raw A2", "%.2f", turret.getRawDegreeA2());
                    telemetry.addData("Live Raw B", "%.2f", turret.getRawDegreeB());

                    if (currentGamepad1.x && !previousGamepad1.x) {
                        capturedOffsetA1 = TurretSubsystem.INVERT_ENCODER_A1 ? (360.0 - turret.getRawDegreeA1()) : turret.getRawDegreeA1();
                        capturedOffsetA2 = TurretSubsystem.INVERT_ENCODER_A2 ? (360.0 - turret.getRawDegreeA2()) : turret.getRawDegreeA2();
                        capturedOffsetB  = TurretSubsystem.INVERT_ENCODER_B ? (360.0 - turret.getRawDegreeB()) : turret.getRawDegreeB();

                        // Seed max/min sweep bounds to current angle
                        maxSweepAngle = turret.getTurretAngle();
                        minSweepAngle = maxSweepAngle;

                        currentState = State.MANUAL_SWEEP;
                    }
                    break;

                case MANUAL_SWEEP:
                    turret.stopMotors();
                    double currentCalculatedAngle = turret.getTurretAngle();

                    if (currentCalculatedAngle > maxSweepAngle) maxSweepAngle = currentCalculatedAngle;
                    if (currentCalculatedAngle < minSweepAngle) minSweepAngle = currentCalculatedAngle;

                    telemetry.addLine("--- STEP 4: MANUAL SWEEP VERIFICATION ---");
                    telemetry.addLine("Sweep the turret back and forth within wire limits.");
                    telemetry.addLine("Check for smooth counting without random jumps.");
                    telemetry.addLine("");
                    telemetry.addData("Calculated Angle", "%.2f deg", currentCalculatedAngle);
                    telemetry.addData("Observed Min Angle", "%.2f deg", minSweepAngle);
                    telemetry.addData("Observed Max Angle", "%.2f deg", maxSweepAngle);
                    telemetry.addLine("");
                    telemetry.addLine("--- COPY THESE INTO TURRETSUBSYSTEM.JAVA ---");
                    telemetry.addData("OFFSET_A1", "%.2f", capturedOffsetA1);
                    telemetry.addData("OFFSET_A2", "%.2f", capturedOffsetA2);
                    telemetry.addData("OFFSET_B", "%.2f", capturedOffsetB);
                    break;
            }

            telemetry.update();
        }
    }
}

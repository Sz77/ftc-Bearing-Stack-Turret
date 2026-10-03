package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Turret TeleOp (0-360)", group = "Examples")
public class TurretTeleOp extends LinearOpMode {

    public static final double JOYSTICK_DEADZONE = 0.05;
    public static final double MAX_MANUAL_TURN_SPEED = 4.0;

    // Preset Angles for D-Pad
    public static final double ANGLE_CENTER = 180.0;
    public static final double ANGLE_LEFT = 90.0;
    public static final double ANGLE_RIGHT = 270.0;

    // Snap to our safe limits instead of 0/360
    public static final double ANGLE_LIMIT_MIN = TurretSubsystem.MIN_TURRET_ANGLE_DEG;
    public static final double ANGLE_LIMIT_MAX = TurretSubsystem.MAX_TURRET_ANGLE_DEG;

    @Override
    public void runOpMode() {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        // Safe Initialization: Read physical state to avoid aggressive startup snaps
        turret.readSensors();
        double targetAngle = turret.getTurretAngle();

        telemetry.addLine("Ready to Start.");
        telemetry.addLine("WARNING: Ensure turret is facing forward before starting!");
        turret.printCalibrationTelemetry(telemetry);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            // Single hardware read per loop
            turret.readSensors();

            // 1. Manual Control
            double joyX = currentGamepad1.left_stick_x;
            if (Math.abs(joyX) > JOYSTICK_DEADZONE) {
                double smoothedInput = joyX * joyX * Math.signum(joyX);
                targetAngle += smoothedInput * MAX_MANUAL_TURN_SPEED;
            }

            // 2. Preset Controls (Edge-detected to prevent fighting manual control)
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) targetAngle = ANGLE_CENTER;
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) targetAngle = ANGLE_LEFT;
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) targetAngle = ANGLE_RIGHT;

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                if (turret.getTurretAngle() < 180.0) {
                    targetAngle = ANGLE_LIMIT_MIN;
                } else {
                    targetAngle = ANGLE_LIMIT_MAX;
                }
            }

            // 3. Safety Constrain Target
            targetAngle = Math.max(TurretSubsystem.MIN_TURRET_ANGLE_DEG,
                                   Math.min(TurretSubsystem.MAX_TURRET_ANGLE_DEG, targetAngle));

            // 4. Update the subsystem
            turret.setTargetAngle(targetAngle);

            // 5. User Feedback
            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Actual Angle", "%.2f", turret.getTurretAngle());
            telemetry.addData("Error", "%.2f", targetAngle - turret.getTurretAngle());

            telemetry.update();
        }
    }
}

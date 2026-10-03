package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret TeleOp", group = "Examples")
public class TurretTeleOp extends LinearOpMode {

    // --- TELEOP CONSTANTS ---
    public static final double INITIAL_TARGET_ANGLE = 0.0;

    // Joystick tuning
    public static final double JOYSTICK_DEADZONE = 0.05;
    public static final double MANUAL_TURN_SPEED = 3.0; // Degrees to add per loop

    // Preset Angles
    public static final double ANGLE_FRONT = 0.0;
    public static final double ANGLE_RIGHT = -90.0;
    public static final double ANGLE_LEFT = 90.0;
    public static final double ANGLE_BACK_POS = 180.0;
    public static final double ANGLE_BACK_NEG = -180.0;

    @Override
    public void runOpMode() {
        TurretSubsystem turret = new TurretSubsystem(hardwareMap);
        double targetAngle = INITIAL_TARGET_ANGLE;

        telemetry.addLine("Ready to Start. Check calibration telemetry below if needed.");
        turret.printCalibrationTelemetry(telemetry);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Adjust target angle using the left joystick X axis (with deadzone)
            if (Math.abs(gamepad1.left_stick_x) > JOYSTICK_DEADZONE) {
                targetAngle += gamepad1.left_stick_x * MANUAL_TURN_SPEED;
            }

            // Snap to exact preset angles using the D-Pad
            if (gamepad1.dpad_up) targetAngle = ANGLE_FRONT;
            if (gamepad1.dpad_right) targetAngle = ANGLE_RIGHT;
            if (gamepad1.dpad_left) targetAngle = ANGLE_LEFT;
            // Map down to 180 or -180 based on which is closer to avoid full rotations if already back there
            if (gamepad1.dpad_down) {
                if (turret.getTurretAngle() < 0) {
                    targetAngle = ANGLE_BACK_NEG;
                } else {
                    targetAngle = ANGLE_BACK_POS;
                }
            }

            // Constrain target variable to the global hard limits defined in the subsystem
            targetAngle = Math.max(TurretSubsystem.MIN_TURRET_ANGLE_DEG,
                                   Math.min(TurretSubsystem.MAX_TURRET_ANGLE_DEG, targetAngle));

            // Run the PID controller
            turret.setTargetAngle(targetAngle);

            // Telemetry output
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Actual Angle", turret.getTurretAngle());
            turret.printCalibrationTelemetry(telemetry);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Hood Test", group = "Testing")
public class HoodTest extends OpMode {
    private Servo hood;
    private double hoodPosition = 1.0;

    private static final double FINE_INCREMENT = 0.01;
    private static final double COARSE_INCREMENT = 0.05;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean bumperLeftPressed = false;
    private boolean bumperRightPressed = false;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(hoodPosition);

        telemetry.addLine("=== HOOD TEST ===");
        telemetry.addLine();
        telemetry.addLine("DPAD UP/DOWN: ±0.05 (coarse)");
        telemetry.addLine("L1/R1: ±0.01 (fine)");
        telemetry.addLine();
        telemetry.addLine("Starting at position 1.0");
        telemetry.update();
    }

    @Override
    public void loop() {
        // COARSE: DPAD UP (increase) / DPAD DOWN (decrease)
        if (gamepad1.dpad_up && !dpadUpPressed) {
            hoodPosition += COARSE_INCREMENT;
            hoodPosition = Math.min(1.0, hoodPosition);
            hood.setPosition(hoodPosition);
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            hoodPosition -= COARSE_INCREMENT;
            hoodPosition = Math.max(0.0, hoodPosition);
            hood.setPosition(hoodPosition);
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // FINE: L1 (decrease) / R1 (increase)
        if (gamepad1.left_bumper && !bumperLeftPressed) {
            hoodPosition -= FINE_INCREMENT;
            hoodPosition = Math.max(0.0, hoodPosition);
            hood.setPosition(hoodPosition);
            bumperLeftPressed = true;
        } else if (!gamepad1.left_bumper) {
            bumperLeftPressed = false;
        }

        if (gamepad1.right_bumper && !bumperRightPressed) {
            hoodPosition += FINE_INCREMENT;
            hoodPosition = Math.min(1.0, hoodPosition);
            hood.setPosition(hoodPosition);
            bumperRightPressed = true;
        } else if (!gamepad1.right_bumper) {
            bumperRightPressed = false;
        }

        // Reset to 1.0 with Cross
        if (gamepad1.cross) {
            hoodPosition = 1.0;
            hood.setPosition(hoodPosition);
        }

        // Reset to 0.0 with Circle
        if (gamepad1.circle) {
            hoodPosition = 0.0;
            hood.setPosition(hoodPosition);
        }

        telemetry.addLine("=== HOOD TEST ===");
        telemetry.addLine();
        telemetry.addData("Hood Position", "%.3f", hoodPosition);
        telemetry.addLine();
        telemetry.addLine("DPAD UP/DOWN: ±0.05");
        telemetry.addLine("L1/R1: ±0.01");
        telemetry.addLine("Cross: Reset to 1.0");
        telemetry.addLine("Circle: Reset to 0.0");
        telemetry.update();
    }
}
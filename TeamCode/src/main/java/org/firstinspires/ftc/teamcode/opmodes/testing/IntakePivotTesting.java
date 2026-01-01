package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Intake Pivot Test", group="Testing")
public class IntakePivotTesting extends LinearOpMode {

    private Servo intakePivotLeft;
    private Servo intakePivotRight;

    private double position = 0.5;  // Start in middle

    // Store your three positions as you find them
    private double openPosition = 0.0;     // OPEN - good for intaking
    private double closedPosition = 0.0;   // CLOSED - down, balls can't enter
    private double upPosition = 0.0;       // UP - clear stuck balls

    private String currentMode = "MANUAL";

    @Override
    public void runOpMode() {
        // Initialize servos
        intakePivotLeft = hardwareMap.get(Servo.class, "intakePivotLeft");
        intakePivotRight = hardwareMap.get(Servo.class, "intakePivotRight");

        // REVERSE one servo so they mirror each other
        intakePivotRight.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Intake Pivot Position Finder");
        telemetry.addLine("================================");
        telemetry.addLine("MANUAL CONTROL:");
        telemetry.addLine("  DPAD UP/DOWN: Adjust position");
        telemetry.addLine();
        telemetry.addLine("SAVE POSITIONS:");
        telemetry.addLine("  A: Save OPEN position");
        telemetry.addLine("  B: Save CLOSED position");
        telemetry.addLine("  Y: Save UP position");
        telemetry.addLine();
        telemetry.addLine("TEST SAVED POSITIONS:");
        telemetry.addLine("  Left Bumper: Go to OPEN");
        telemetry.addLine("  Right Bumper: Go to CLOSED");
        telemetry.addLine("  Left Trigger: Go to UP");
        telemetry.update();

        waitForStart();

        // Set initial position
        intakePivotLeft.setPosition(position);
        intakePivotRight.setPosition(position);

        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        boolean aPressed = false;
        boolean bPressed = false;
        boolean yPressed = false;
        boolean leftBumperPressed = false;
        boolean rightBumperPressed = false;

        while (opModeIsActive()) {

            // MANUAL ADJUSTMENT
            if (gamepad1.dpad_up && !dpadUpPressed) {
                position += 0.05;
                if (position > 1.0) position = 1.0;
                currentMode = "MANUAL";
            }
            dpadUpPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownPressed) {
                position -= 0.05;
                if (position < 0.0) position = 0.0;
                currentMode = "MANUAL";
            }
            dpadDownPressed = gamepad1.dpad_down;

            // SAVE POSITIONS
            if (gamepad1.a && !aPressed) {
                openPosition = position;
                currentMode = "SAVED OPEN";
            }
            aPressed = gamepad1.a;

            if (gamepad1.b && !bPressed) {
                closedPosition = position;
                currentMode = "SAVED CLOSED";
            }
            bPressed = gamepad1.b;

            if (gamepad1.y && !yPressed) {
                upPosition = position;
                currentMode = "SAVED UP";
            }
            yPressed = gamepad1.y;

            // TEST SAVED POSITIONS
            if (gamepad1.left_bumper && !leftBumperPressed) {
                position = openPosition;
                currentMode = "TESTING OPEN";
            }
            leftBumperPressed = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !rightBumperPressed) {
                position = closedPosition;
                currentMode = "TESTING CLOSED";
            }
            rightBumperPressed = gamepad1.right_bumper;

            if (gamepad1.left_trigger > 0.5) {
                position = upPosition;
                currentMode = "TESTING UP";
            }

            // Apply position to both servos
            intakePivotLeft.setPosition(position);
            intakePivotRight.setPosition(position);

            // TELEMETRY
            telemetry.addLine("=== INTAKE PIVOT FINDER ===");
            telemetry.addData("Mode", currentMode);
            telemetry.addData("Current Position", "%.3f", position);
            telemetry.addLine();
            telemetry.addLine("--- SAVED POSITIONS ---");
            telemetry.addData("OPEN (intaking)", "%.3f", openPosition);
            telemetry.addData("CLOSED (locked)", "%.3f", closedPosition);
            telemetry.addData("UP (clear jams)", "%.3f", upPosition);
            telemetry.addLine();
            telemetry.addLine("======================");
            telemetry.addLine("WRITE THESE DOWN FOR Constants.java:");
            telemetry.addData("INTAKE_PIVOT_OPEN", "%.3f", openPosition);
            telemetry.addData("INTAKE_PIVOT_CLOSED", "%.3f", closedPosition);
            telemetry.addData("INTAKE_PIVOT_UP", "%.3f", upPosition);
            telemetry.update();

            sleep(50);
        }
    }
}
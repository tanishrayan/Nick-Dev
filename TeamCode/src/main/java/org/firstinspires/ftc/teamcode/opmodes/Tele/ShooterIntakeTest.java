package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "Shooter Intake Test", group = "Testing")
public class ShooterIntakeTest extends OpMode {

    // Shooter hardware
    private DcMotorEx topMotor;
    private DcMotorEx bottomMotor;
    private Servo hoodServo;

    // Intake hardware
    private DcMotor intakeMotor;
    private Servo rampServo;

    // Current values
    private double targetVelocity = 1000.0;
    private double hoodPosition = 1.0;
    private double rampPosition = 0.49;

    // PIDF values
    private double P = 240.0;
    private double F = 15.8;
    private double[] stepSizes = {100.0, 10.0, 1.0, 0.1, 0.01};
    private int stepIndex = 2; // Start at 1.0

    // Increments
    private static final double VELOCITY_INCREMENT = 100.0;
    private static final double VELOCITY_FINE_INCREMENT = 25.0;
    private static final double HOOD_INCREMENT = 0.05;
    private static final double HOOD_FINE_INCREMENT = 0.01;

    // Button states
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean l1Pressed = false;
    private boolean r1Pressed = false;
    private boolean l2Pressed = false;
    private boolean r2Pressed = false;
    private boolean crossPressed = false;
    private boolean circlePressed = false;
    private boolean trianglePressed = false;
    private boolean squarePressed = false;
    private boolean optionsPressed = false;
    private boolean sharePressed = false;

    // States
    private boolean flywheelRunning = false;
    private boolean intakeRunning = false;
    private boolean pidfTuningMode = false; // Toggle between normal controls and PIDF tuning

    @Override
    public void init() {
        // Initialize shooter motors
        topMotor = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");

        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set initial PIDF coefficients
        setPIDFCoefficients(P, F);

        // Initialize hood servo
        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(hoodPosition);

        // Initialize intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        //topMotor.setDirection(DcMotor.Direction.REVERSE);


        // Initialize ramp servo
        rampServo = hardwareMap.get(Servo.class, "rampServo");
        rampServo.setPosition(rampPosition);

        telemetry.addLine("=== SHOOTER/INTAKE ISOLATION TEST ===");
        telemetry.addLine();
        telemetry.addLine("FLYWHEEL:");
        telemetry.addLine("  Circle: Toggle ON/OFF");
        telemetry.addLine("  DPAD Up/Down: Velocity ±100 (normal mode)");
        telemetry.addLine("  L2/R2: Velocity ±25 (fine)");
        telemetry.addLine();
        telemetry.addLine("HOOD:");
        telemetry.addLine("  DPAD Left/Right: Position ±0.05 (normal mode)");
        telemetry.addLine("  L1/R1: Position ±0.01 (fine)");
        telemetry.addLine();
        telemetry.addLine("INTAKE:");
        telemetry.addLine("  Square (hold): Run intake");
        telemetry.addLine("  Triangle: Toggle ramp up/down");
        telemetry.addLine("  Cross: Toggle transfer mode");
        telemetry.addLine();
        telemetry.addLine("PIDF TUNING:");
        telemetry.addLine("  Share: Toggle PIDF tuning mode");
        telemetry.addLine("  Options: Cycle step size");
        telemetry.addLine("  DPAD Up/Down: Adjust P (tuning mode)");
        telemetry.addLine("  DPAD Left/Right: Adjust F (tuning mode)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ===== MODE TOGGLE (Share button - PS4 touchpad) =====
        if (gamepad1.share && !sharePressed) {
            pidfTuningMode = !pidfTuningMode;
            sharePressed = true;
        } else if (!gamepad1.share) {
            sharePressed = false;
        }

        // ===== STEP SIZE CYCLING (Options button) =====
        if (gamepad1.options && !optionsPressed) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
            optionsPressed = true;
        } else if (!gamepad1.options) {
            optionsPressed = false;
        }

        // ===== FLYWHEEL TOGGLE (Circle) =====
        if (gamepad1.circle && !circlePressed) {
            flywheelRunning = !flywheelRunning;
            if (flywheelRunning) {
                topMotor.setVelocity(targetVelocity);
                bottomMotor.setVelocity(-targetVelocity);
            } else {
                topMotor.setPower(0);
                bottomMotor.setPower(0);
            }
            circlePressed = true;
        } else if (!gamepad1.circle) {
            circlePressed = false;
        }

        if (pidfTuningMode) {
            // ===== PIDF TUNING MODE =====

            // DPAD Up/Down: Adjust P
            if (gamepad1.dpad_up && !dpadUpPressed) {
                P += stepSizes[stepIndex];
                setPIDFCoefficients(P, F);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            if (gamepad1.dpad_down && !dpadDownPressed) {
                P -= stepSizes[stepIndex];
                P = Math.max(0, P);
                setPIDFCoefficients(P, F);
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            // DPAD Left/Right: Adjust F
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                F -= stepSizes[stepIndex];
                F = Math.max(0, F);
                setPIDFCoefficients(P, F);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                F += stepSizes[stepIndex];
                setPIDFCoefficients(P, F);
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

        } else {
            // ===== NORMAL MODE =====

            // VELOCITY ADJUST (DPAD Up/Down - coarse)
            if (gamepad1.dpad_up && !dpadUpPressed) {
                targetVelocity += VELOCITY_INCREMENT;
                if (flywheelRunning) {
                    topMotor.setVelocity(targetVelocity);
                    bottomMotor.setVelocity(-targetVelocity);
                }
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }

            if (gamepad1.dpad_down && !dpadDownPressed) {
                targetVelocity -= VELOCITY_INCREMENT;
                targetVelocity = Math.max(0, targetVelocity);
                if (flywheelRunning) {
                    topMotor.setVelocity(targetVelocity);
                    bottomMotor.setVelocity(-targetVelocity);
                }
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }

            // HOOD ADJUST (DPAD Left/Right - coarse)
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                hoodPosition -= HOOD_INCREMENT;
                hoodPosition = Math.max(0.0, hoodPosition);
                hoodServo.setPosition(hoodPosition);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                hoodPosition += HOOD_INCREMENT;
                hoodPosition = Math.min(1.0, hoodPosition);
                hoodServo.setPosition(hoodPosition);
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }
        }

        // ===== VELOCITY FINE ADJUST (L2/R2) - Works in both modes =====
        if (gamepad1.left_trigger > 0.5 && !l2Pressed) {
            targetVelocity -= VELOCITY_FINE_INCREMENT;
            targetVelocity = Math.max(0, targetVelocity);
            if (flywheelRunning) {
                topMotor.setVelocity(targetVelocity);
                bottomMotor.setVelocity(-targetVelocity);
            }
            l2Pressed = true;
        } else if (gamepad1.left_trigger < 0.3) {
            l2Pressed = false;
        }

        if (gamepad1.right_trigger > 0.5 && !r2Pressed) {
            targetVelocity += VELOCITY_FINE_INCREMENT;
            if (flywheelRunning) {
                topMotor.setVelocity(targetVelocity);
                bottomMotor.setVelocity(-targetVelocity);
            }
            r2Pressed = true;
        } else if (gamepad1.right_trigger < 0.3) {
            r2Pressed = false;
        }

        // ===== HOOD FINE ADJUST (L1/R1) - Works in both modes =====
        if (gamepad1.left_bumper && !l1Pressed) {
            hoodPosition -= HOOD_FINE_INCREMENT;
            hoodPosition = Math.max(0.0, hoodPosition);
            hoodServo.setPosition(hoodPosition);
            l1Pressed = true;
        } else if (!gamepad1.left_bumper) {
            l1Pressed = false;
        }

        if (gamepad1.right_bumper && !r1Pressed) {
            hoodPosition += HOOD_FINE_INCREMENT;
            hoodPosition = Math.min(1.0, hoodPosition);
            hoodServo.setPosition(hoodPosition);
            r1Pressed = true;
        } else if (!gamepad1.right_bumper) {
            r1Pressed = false;
        }

        // ===== INTAKE (Square - hold to run) =====
        if (gamepad1.square) {
            intakeMotor.setPower(1.0);
            intakeRunning = true;
        } else if (!gamepad1.cross) {  // Don't stop if in transfer mode
            if (intakeRunning && !gamepad1.cross) {
                intakeMotor.setPower(0);
                intakeRunning = false;
            }
        }

        // ===== RAMP TOGGLE (Triangle) =====
        if (gamepad1.triangle && !trianglePressed) {
            if (rampPosition < 0.55) {
                rampPosition = 0.61;  // Up position
            } else {
                rampPosition = 0.49;  // Down position
            }
            rampServo.setPosition(rampPosition);
            trianglePressed = true;
        } else if (!gamepad1.triangle) {
            trianglePressed = false;
        }

        // ===== TRANSFER MODE (Cross - toggle) =====
        if (gamepad1.cross && !crossPressed) {
            // Ramp up + intake running
            rampPosition = 0.61;
            rampServo.setPosition(rampPosition);
            intakeMotor.setPower(1.0);
            intakeRunning = true;
            crossPressed = true;
        } else if (!gamepad1.cross) {
            crossPressed = false;
        }

        // ===== TELEMETRY =====
        telemetry.addLine("========== MODE ==========");
        telemetry.addData("Control Mode", pidfTuningMode ? ">>> PIDF TUNING <<<" : "NORMAL");
        telemetry.addLine();

        telemetry.addLine("========== SHOOTER ==========");
        telemetry.addData("Flywheel", flywheelRunning ? ">>> RUNNING <<<" : "STOPPED");
        telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocity);
        telemetry.addData("Top Motor Actual", "%.0f ticks/sec", topMotor.getVelocity());
        telemetry.addData("Bottom Motor Actual", "%.0f ticks/sec", bottomMotor.getVelocity());

        double avgVelocity = (Math.abs(topMotor.getVelocity()) + Math.abs(bottomMotor.getVelocity())) / 2.0;
        double error = targetVelocity - avgVelocity;
        telemetry.addData("Avg Velocity", "%.0f ticks/sec", avgVelocity);
        telemetry.addData("Velocity Error", "%.0f ticks/sec", error);
        telemetry.addLine();

        telemetry.addLine("========== PIDF VALUES ==========");
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.addLine();

        telemetry.addLine("========== HOOD ==========");
        telemetry.addData("Hood Position", "%.3f", hoodPosition);
        telemetry.addData("Hood Actual", "%.3f", hoodServo.getPosition());
        telemetry.addLine();

        telemetry.addLine("========== INTAKE ==========");
        telemetry.addData("Intake Motor", intakeRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Ramp Position", "%.3f (%s)", rampPosition, rampPosition > 0.55 ? "UP" : "DOWN");
        telemetry.addLine();

        if (pidfTuningMode) {
            telemetry.addLine("========== PIDF TUNING CONTROLS ==========");
            telemetry.addLine("DPAD ↑↓: Adjust P");
            telemetry.addLine("DPAD ←→: Adjust F");
            telemetry.addLine("Options: Cycle step size");
            telemetry.addLine("Share: Exit tuning mode");
        } else {
            telemetry.addLine("========== NORMAL CONTROLS ==========");
            telemetry.addLine("Circle: Flywheel ON/OFF");
            telemetry.addLine("DPAD ↑↓: Velocity ±100");
            telemetry.addLine("L2/R2: Velocity ±25");
            telemetry.addLine("DPAD ←→: Hood ±0.05");
            telemetry.addLine("L1/R1: Hood ±0.01");
            telemetry.addLine("Share: Enter PIDF tuning");
        }

        telemetry.update();
    }

    private void setPIDFCoefficients(double p, double f) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, 0, 0, f);
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void stop() {
        topMotor.setPower(0);
        bottomMotor.setPower(0);
        intakeMotor.setPower(0);
    }
}
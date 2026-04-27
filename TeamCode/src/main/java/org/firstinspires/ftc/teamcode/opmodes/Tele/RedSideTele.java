package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.SharedData;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp()
public class RedSideTele extends OpMode {
    private Drivetrain drivetrain;
    private IntakeAndTransfer intakeTransfer;
    private Launcher launcher;
    private Turret turret;

    private boolean squarePressed = false;

    // SHOOTER CALIBRATION VARIABLES
    private double currentVelocity = 1000.0;
    private double currentHoodPosition = 0.37;

    private static final double TURRET_INCREMENT = 5.0; // degrees

    // Deadband thresholds to prevent jitter
    private static final double HOOD_DEADBAND = 0.01;
    private static final double VELOCITY_DEADBAND = 25.0;

    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    private boolean shooterRunning = false;
    private boolean autoAimEnabled = false; // Toggle for auto-aim

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        intakeTransfer = new IntakeAndTransfer(hardwareMap);
        intakeTransfer.setTelemetry(telemetry);

        // ===== DEBUG SHARED DATA (SAME AS BLUE SIDE) =====
        telemetry.addLine("=============================");
        telemetry.addLine(">>> SHARED DATA CHECK (RED) <<<");
        telemetry.addData("hasAutonomousRun", SharedData.hasAutonomousRun);

        if (SharedData.lastKnownPose != null) {
            telemetry.addData("Pose exists", "YES");
            telemetry.addData("Stored X", "%.1f", SharedData.lastKnownPose.getX());
            telemetry.addData("Stored Y", "%.1f", SharedData.lastKnownPose.getY());
            telemetry.addData("Stored Heading", "%.1f", Math.toDegrees(SharedData.lastKnownPose.getHeading()));
        } else {
            telemetry.addLine(">>> lastKnownPose is NULL! <<<");
        }
        telemetry.addLine("=============================");
        telemetry.update();

        // Wait 2 seconds so you can read it
        try { Thread.sleep(2000); } catch (InterruptedException e) {}

        if (SharedData.hasAutonomousRun && SharedData.lastKnownPose != null) {
            // Use the pose from autonomous
            drivetrain.setPose(SharedData.lastKnownPose);
            Pose2D currentPose = drivetrain.getPosition();

            // Extract values and modify (MATCH BLUE SIDE EXACTLY)
            double newX = currentPose.getX(DistanceUnit.INCH) + 36.9;  // Note: positive!
            double newY = currentPose.getY(DistanceUnit.INCH) + 62.8;  // Note: negative!

            // Create and set new pose
            Pose adjustedPose = new Pose(newX, newY, currentPose.getHeading(AngleUnit.RADIANS));
            drivetrain.setPose(adjustedPose);

            telemetry.addLine(">>> LOADING POSE FROM AUTO <<<");
            telemetry.addData("Original X", "%.1f", SharedData.lastKnownPose.getX());
            telemetry.addData("Original Y", "%.1f", SharedData.lastKnownPose.getY());
            telemetry.addData("Adjusted X", "%.1f", newX);
            telemetry.addData("Adjusted Y", "%.1f", newY);
        } else {
            // Set default TeleOp starting position for RED SIDE
            Pose teleOpStartPose = new Pose(0, 0, Math.toRadians(180));
            drivetrain.setPose(teleOpStartPose);

            telemetry.addLine(">>> USING DEFAULT POSE (RED) <<<");
            telemetry.addLine("Reason:");
            if (!SharedData.hasAutonomousRun) {
                telemetry.addLine("  - hasAutonomousRun is FALSE");
            }
            if (SharedData.lastKnownPose == null) {
                telemetry.addLine("  - lastKnownPose is NULL");
            }
        }

        launcher = new Launcher(hardwareMap);
        launcher.setHoodRetracted();

        turret = new Turret(hardwareMap);
        turret.setTelemetry(telemetry);

        // RED SIDE TELEOP: Goal is opposite corner
        turret.setGoalPosition(72.0, 76.0);

        telemetry.addLine();
        telemetry.addLine("=== RED SIDE TELEOP ===");
        // ... rest of your telemetry ...
    }

    @Override
    public void loop() {
        // ===== DRIVETRAIN =====
        drivetrain.handleDrivetrain(gamepad1);

        // ===== POSITION RESET - Press SHARE to reset to field center =====
        if (gamepad1.share) {
            drivetrain.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 180));
            telemetry.addLine(">>> POSITION RESET TO CENTER (0, 0, 180°) <<<");
            try { Thread.sleep(300); } catch (InterruptedException e) {}
        }

        // ===== IMU RESET =====
        if (gamepad1.cross) {
            // Reset IMU to current heading as 0°
            drivetrain.resetIMU();

            // Reset odometry position to (0, 0, 0°)
            drivetrain.resetPosition();

            // Reset turret to front-facing position
            turret.setToFacingFront();

            // Clear the autonomous flag so next run starts fresh
            SharedData.hasAutonomousRun = false;

            // Visual feedback
            telemetry.addLine("=============================");
            telemetry.addLine(">>> FULL SYSTEM RESET <<<");
            telemetry.addLine("IMU: 0°");
            telemetry.addLine("Position: (0, 0)");
            telemetry.addLine("Turret: Front");
            telemetry.addLine("Auto Flag: Cleared");
            telemetry.addLine("=============================");
            telemetry.update();

            try { Thread.sleep(500); } catch (InterruptedException e) {}
        }

        // ===== INTAKE CONTROLS (GAMEPAD 2) =====
        if (gamepad2.square) {
            if (!squarePressed) {
                //intakeTransfer.setIntake();
                squarePressed = true;
            }
        } else {
            if (squarePressed) {
                //intakeTransfer.setStatic();
                squarePressed = false;
            }
        }

        if (gamepad2.triangle) {
            //intakeTransfer.setTransfer();
        }

        // ===== GET ROBOT STATE =====
        double robotX = drivetrain.getX();
        double robotY = drivetrain.getY();
        double robotHeading = drivetrain.getHeading();

        // ===== AUTO-CALCULATE SHOOTER PARAMETERS (ALWAYS RUNS) =====
        double distanceToGoal = turret.calculateDistanceToGoal(robotX, robotY);
        double newVelocity = launcher.calculateFlywheelVelocity(distanceToGoal);
        double newHoodPosition = launcher.calculateHoodAngle(distanceToGoal);

        // Only update hood if change is significant (prevents jitter from odometry noise)
        if (Math.abs(newHoodPosition - currentHoodPosition) > HOOD_DEADBAND) {
            currentHoodPosition = newHoodPosition;
            launcher.setHoodPosition(currentHoodPosition);
        }

        // Only update velocity if change is significant
        if (Math.abs(newVelocity - currentVelocity) > VELOCITY_DEADBAND) {
            currentVelocity = newVelocity;
            if (shooterRunning) {
                launcher.setFlywheelVelocity(currentVelocity);
            }
        }

        // ===== TURRET CONTROLS (GAMEPAD 1) =====

        // Toggle AUTO-AIM with Options button
        if (gamepad1.options) {
            autoAimEnabled = !autoAimEnabled;
            try { Thread.sleep(200); } catch (InterruptedException e) {}
        }

        if (autoAimEnabled) {
            // AUTO-AIM MODE: Turret aims automatically
            turret.aimAtGoal(robotX, robotY, robotHeading);
        } else {
            // MANUAL TURRET MODE: Manual turret angle control only
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                double currentAngle = turret.getCurrentAngle();
                turret.setTurretAngle(currentAngle - TURRET_INCREMENT);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                double currentAngle = turret.getCurrentAngle();
                turret.setTurretAngle(currentAngle + TURRET_INCREMENT);
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }
        }

        // ===== FLYWHEEL ON/OFF - CIRCLE (GAMEPAD 2) =====
        if (gamepad2.circle) {
            if (!shooterRunning) {
                launcher.setFlywheelVelocity(currentVelocity);
                shooterRunning = true;
            } else {
                launcher.stopFlywheel();
                shooterRunning = false;
            }
            try { Thread.sleep(200); } catch (InterruptedException e) {}
        }

        // ===== TELEMETRY =====
        telemetry.addLine("=== MODE ===");
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ENABLED ✓✓✓" : "MANUAL TURRET");

        telemetry.addLine();
        telemetry.addLine("=== INTAKE (Dual Motor) ===");
        intakeTransfer.updateTelemetry();

        telemetry.addLine();
        telemetry.addLine("=== SHOOTER (Auto Distance-Based) ===");
        telemetry.addData("Flywheel Status", shooterRunning ? "RUNNING ✓" : "STOPPED");
        telemetry.addData("Target Velocity", "%.0f RPM", currentVelocity);
        telemetry.addData("Actual Velocity", "%.0f RPM", launcher.getFlywheelVelocity());
        telemetry.addData("Hood Position", "%.3f", currentHoodPosition);
        telemetry.addData("Distance to Goal", "%.1f in", distanceToGoal);

        telemetry.addLine();
        telemetry.addLine("=== TURRET ===");
        if (autoAimEnabled) {
            //turret.updateAutoAimTelemetry(robotX, robotY, robotHeading);
        } else {
            turret.updateTelemetry();
            telemetry.addData("Manual Aim", "Use DPAD L/R");
        }

        telemetry.addLine();
        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("X Position", "%.1f in", robotX);
        telemetry.addData("Y Position", "%.1f in", robotY);
        telemetry.addData("Heading", "%.1f°", robotHeading);

        telemetry.update();
    }

    @Override
    public void stop() {
        launcher.stopFlywheel();
    }
}
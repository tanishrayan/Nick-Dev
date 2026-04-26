package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.SharedData;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.pedropathing.geometry.Pose;

import java.util.List;

@TeleOp(name = "BlueSideTele")
public class BlueSideTele extends OpMode {

    // ── Subsystems ─────────────────────────────────────────────
    private Drivetrain        drivetrain;
    private IntakeAndTransfer intakeTransfer;
    private Launcher          launcher;
    private Turret            turret;
    private Limelight3A       limelight;

    // ── Field coordinates (0,0 = field center) ────────────────
    private static final double GOAL_X = 72.0;
    private static final double GOAL_Y = -72.0;

    // ── Limelight distance constants (still used for distance calc) ──
    private static final double CAMERA_HEIGHT_INCHES   = 13.3125;
    private static final double TARGET_HEIGHT_INCHES   = 29.0;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 15.0;
    private static final int    TARGET_TAG_ID          = 20;

    // ── Shooter state ──────────────────────────────────────────
    private double currentVelocity     = 1000.0;
    private double currentHoodPosition = 0.37;
    private static final double HOOD_DEADBAND     = 0.01;
    private static final double VELOCITY_DEADBAND = 25.0;
    private boolean shooterRunning = false;
    private boolean autoAimEnabled = false;

    // ── Shooting sequence ──────────────────────────────────────
    private enum ShootState { IDLE, SPINUP, OPEN_LATCH, WAIT_BALL_GONE, CLOSE_LATCH }
    private ShootState  shootState = ShootState.IDLE;
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double BALL_CLEAR_WAIT_SEC  = 0.5;  // time after beam clear before closing
    private static final double CLOSE_LATCH_WAIT_SEC = 0.35; // recovery time between balls

    // ── Third ball confirmation ────────────────────────────────
    private boolean     fullLoadPending = false;
    private ElapsedTime fullLoadTimer   = new ElapsedTime();
    private static final double FULL_LOAD_CONFIRM_SEC = 1.0;

    // ── Dip button (gamepad1 triangle) ────────────────────────
    private boolean     dipping  = false;
    private ElapsedTime dipTimer = new ElapsedTime();
    private static final double DIP_DOWN_SEC  = 0.35;
    private static final double DIP_TOTAL_SEC = 0.65;

    // ── Turret manual ─────────────────────────────────────────
    private static final double TURRET_INCREMENT = 5.0;

    // ── Edge detection ─────────────────────────────────────────
    private boolean optionsWasPressed      = false;
    private boolean circleWasPressed       = false;
    private boolean rightTriggerWasPressed = false;
    private boolean triangleWasPressed     = false;
    private boolean dpadLeftPressed        = false;
    private boolean dpadRightPressed       = false;

    @Override
    public void init() {
        drivetrain     = new Drivetrain(hardwareMap);
        intakeTransfer = new IntakeAndTransfer(hardwareMap);
        launcher       = new Launcher(hardwareMap);
        turret         = new Turret(hardwareMap);

        intakeTransfer.setTelemetry(telemetry);
        launcher.setTelemetry(telemetry);
        turret.setTelemetry(telemetry);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        if (SharedData.hasAutonomousRun && SharedData.lastKnownPose != null) {
            drivetrain.setPose(SharedData.lastKnownPose);
        } else {
            drivetrain.setPose(new Pose(0, 0, 0));
        }

        turret.setGoalPosition(GOAL_X, GOAL_Y);
        launcher.setHoodRetracted();
        launcher.closeLatch();

        telemetry.addLine("Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {

        drivetrain.handleDrivetrain(gamepad1);

        if (gamepad1.cross) {
            drivetrain.resetIMU();
            drivetrain.resetPosition();
            turret.setToFacingFront();
            turret.resetEncoder();
            SharedData.hasAutonomousRun = false;
            autoAimEnabled = false;
        }

        double robotX       = drivetrain.getX();
        double robotY       = drivetrain.getY();
        double robotHeading = drivetrain.getHeading();

        // ── Distance — limelight if available, odometry fallback ──
        LLResult llResult = null;
        LLResultTypes.FiducialResult currentTag = null;
        try {
            llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> tags = llResult.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult t : tags) {
                        if (t.getFiducialId() == TARGET_TAG_ID) { currentTag = t; break; }
                    }
                }
            }
        } catch (Exception e) { /* skip frame */ }

        double distanceToGoal;
        if (currentTag != null) {
            distanceToGoal = calculateLimelightDistance(currentTag.getTargetYDegrees());
        } else {
            distanceToGoal = turret.calculateDistanceToGoal(robotX, robotY);
        }

        // ── Hood + velocity auto-calc ──────────────────────────
        double newVelocity     = launcher.calculateFlywheelVelocity(distanceToGoal);
        double newHoodPosition = launcher.calculateHoodAngle(distanceToGoal);

        if (Math.abs(newHoodPosition - currentHoodPosition) > HOOD_DEADBAND) {
            currentHoodPosition = newHoodPosition;
            launcher.setHoodPosition(currentHoodPosition);
        }
        if (Math.abs(newVelocity - currentVelocity) > VELOCITY_DEADBAND) {
            currentVelocity = newVelocity;
            if (shooterRunning) launcher.setFlywheelVelocity(currentVelocity);
        }

        // ── Auto-aim toggle (gamepad1 options) ────────────────
        boolean optionsNow = gamepad1.options;
        if (optionsNow && !optionsWasPressed) {
            autoAimEnabled = !autoAimEnabled;
            if (!autoAimEnabled) turret.setMotorPowerDirectly(0);
        }
        optionsWasPressed = optionsNow;

        // ── Turret — odometry only, no limelight ──────────────
        if (autoAimEnabled) {
            turret.aimAtGoal(robotX, robotY, robotHeading);
            turret.update();
        } else {
            // Manual dpad
            if (gamepad2.dpad_left && !dpadLeftPressed)
                turret.setTurretAngle(turret.getCurrentAngle() - TURRET_INCREMENT);
            dpadLeftPressed = gamepad2.dpad_left;

            if (gamepad2.dpad_right && !dpadRightPressed)
                turret.setTurretAngle(turret.getCurrentAngle() + TURRET_INCREMENT);
            dpadRightPressed = gamepad2.dpad_right;

            turret.update();
        }

        // ── Flywheel toggle (gamepad2 circle) ─────────────────
        boolean circleNow = gamepad2.circle;
        if (circleNow && !circleWasPressed) {
            if (!shooterRunning) {
                launcher.setFlywheelVelocity(currentVelocity);
                shooterRunning = true;
            } else {
                launcher.stopFlywheel();
                shooterRunning = false;
            }
        }
        circleWasPressed = circleNow;

        // ── Intake toggle (gamepad1 right trigger) ─────────────
        boolean rightTriggerNow = gamepad1.right_trigger > 0.5;
        if (rightTriggerNow && !rightTriggerWasPressed) intakeTransfer.toggleIntake();
        rightTriggerWasPressed = rightTriggerNow;

        // ── Dip button (gamepad1 triangle) ────────────────────
        boolean triangleNow = gamepad1.triangle;
        if (triangleNow && !triangleWasPressed && !dipping && shootState == ShootState.IDLE) {
            dipping = true;
            dipTimer.reset();
            intakeTransfer.setFloorIntaking();
        }
        triangleWasPressed = triangleNow;

        if (dipping) {
            if (dipTimer.seconds() >= DIP_DOWN_SEC)  intakeTransfer.setIntaking();
            if (dipTimer.seconds() >= DIP_TOTAL_SEC) dipping = false;
        }

        // ── Auto-stop intake when fully loaded ─────────────────
        if (intakeTransfer.isIntaking() && shootState == ShootState.IDLE && !dipping) {
            if (intakeTransfer.isFullyLoaded()) {
                if (!fullLoadPending) {
                    fullLoadPending = true;
                    fullLoadTimer.reset();
                } else if (fullLoadTimer.seconds() >= FULL_LOAD_CONFIRM_SEC) {
                    intakeTransfer.setIdle();
                    fullLoadPending = false;
                }
            }
        } else {
            fullLoadPending = false;
        }

        // ── Outtake (gamepad1 left trigger, hold) ─────────────
        if (gamepad1.left_trigger > 0.5) {
            intakeTransfer.setOuttaking();
        } else if (intakeTransfer.getState() == IntakeAndTransfer.IntakeState.OUTTAKING) {
            intakeTransfer.setIdle();
        }

        // ── Shoot sequence (GP1 RB = start, GP1 LB = abort) ───
        if (gamepad1.right_bumper && shootState == ShootState.IDLE) {
            startShootSequence();
        }
        if (gamepad1.left_bumper && shootState != ShootState.IDLE) {
            launcher.closeLatch();
            launcher.stopFlywheel();
            shooterRunning = false;
            intakeTransfer.setIdle();
            shootState = ShootState.IDLE;
        }
        updateShootSequence();

        // ── Telemetry ──────────────────────────────────────────
        telemetry.addLine("=== MODE ===");
        telemetry.addData("Auto-Aim",    autoAimEnabled ? "ENABLED (odometry)" : "MANUAL");
        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Dipping",     dipping ? "YES" : "NO");
        telemetry.addLine();
        telemetry.addLine("=== INTAKE ===");
        intakeTransfer.updateTelemetry();
        telemetry.addLine();
        telemetry.addLine("=== LAUNCHER ===");
        launcher.updateTelemetry();
        telemetry.addData("Distance",    "%.1f in", distanceToGoal);
        telemetry.addData("Dist source", currentTag != null ? "Limelight" : "Odometry");
        telemetry.addLine();
        telemetry.addLine("=== TURRET ===");
        turret.updateTelemetry();
        telemetry.addLine();
        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("X",       "%.1f in", robotX);
        telemetry.addData("Y",       "%.1f in", robotY);
        telemetry.addData("Heading", "%.1f°",   robotHeading);
        telemetry.update();
    }

    // ── Shoot sequence ─────────────────────────────────────────

    private void startShootSequence() {
        if (!shooterRunning) {
            launcher.setFlywheelVelocity(currentVelocity);
            shooterRunning = true;
        }
        shootState = ShootState.SPINUP;
        shootTimer.reset();
    }

    private void updateShootSequence() {
        switch (shootState) {
            case SPINUP:
                if (launcher.isFlywheelReady()) {
                    launcher.openLatch();
                    intakeTransfer.setIntaking();
                    shootState = ShootState.OPEN_LATCH;
                    shootTimer.reset();
                }
                break;

            case OPEN_LATCH:
                // Wait for front beam to clear (ball has entered flywheel)
                if (intakeTransfer.isFrontBeamClear()) {
                    shootState = ShootState.WAIT_BALL_GONE;
                    shootTimer.reset();
                }
                break;

            case WAIT_BALL_GONE:
                // Give extra time to make sure ball is fully through
                if (shootTimer.seconds() >= BALL_CLEAR_WAIT_SEC) {
                    launcher.closeLatch();
                    shootState = ShootState.CLOSE_LATCH;
                    shootTimer.reset();
                }
                break;

            case CLOSE_LATCH:
                // Give flywheel time to recover before next ball
                if (shootTimer.seconds() >= CLOSE_LATCH_WAIT_SEC) {
                    if (!intakeTransfer.isEmpty()) {
                        launcher.openLatch();
                        shootState = ShootState.OPEN_LATCH;
                        shootTimer.reset();
                    } else {
                        intakeTransfer.setIdle();
                        launcher.stopFlywheel();
                        shooterRunning = false;
                        shootState = ShootState.IDLE;
                    }
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    // ── Helpers ───────────────────────────────────────────────

    private double calculateLimelightDistance(double ty) {
        double angleToTargetRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty);
        return (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleToTargetRad);
    }

    @Override
    public void stop() {
        launcher.stopFlywheel();
        launcher.closeLatch();
        intakeTransfer.setIdle();
        limelight.stop();
    }
}
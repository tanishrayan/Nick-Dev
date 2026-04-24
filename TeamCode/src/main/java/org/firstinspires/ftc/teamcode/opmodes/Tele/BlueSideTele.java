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
    private static final double GOAL_X = 72.0;  // TODO: measure actual blue goal
    private static final double GOAL_Y = -72.0;

    // ── Limelight constants ────────────────────────────────────
    private static final double kP_LIMELIGHT          = 0.015;
    private static final double LL_MAX_SPEED          = 0.5;
    private static final double DEADBAND_DEG          = 0.5;
    private static final int    TARGET_TAG_ID         = 20;
    private static final double TAG_LOST_HOLD_SECONDS = 0.15;

    // ── Limelight distance constants ───────────────────────────
    private static final double CAMERA_HEIGHT_INCHES   = 13.3125;
    private static final double TARGET_HEIGHT_INCHES   = 29.0;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 15.0;

    // ── Heading compensation ───────────────────────────────────
    private static final double HEADING_COMPENSATION = 0.002;
    private static final double MAX_ANGULAR_VELOCITY = 10.0;

    // ── Limelight runtime state ────────────────────────────────
    private double      lastTurretPower       = 0.0;
    private boolean     limelightJustTookOver = false;
    private boolean     tagWasVisible         = false;
    private ElapsedTime tagLostTimer          = new ElapsedTime();
    private double      lastHeading           = 0.0;
    private double      angularVelocity       = 0.0;

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
    private static final double BALL_CLEAR_WAIT_SEC = 0.5;

    // ── Turret manual ─────────────────────────────────────────
    private static final double TURRET_INCREMENT = 5.0;

    // ── Edge detection ─────────────────────────────────────────
    private boolean optionsWasPressed      = false;
    private boolean circleWasPressed       = false;
    private boolean rightTriggerWasPressed = false;
    private boolean dpadLeftPressed        = false;
    private boolean dpadRightPressed       = false;

    // ── Init ───────────────────────────────────────────────────
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

        lastHeading = drivetrain.getHeading();

        telemetry.addLine("Ready.");
        telemetry.update();
    }

    // ── Main loop ──────────────────────────────────────────────
    @Override
    public void loop() {

        // ── Drivetrain ────────────────────────────────────────
        drivetrain.handleDrivetrain(gamepad1);

        // ── Full reset (gamepad1 cross) ────────────────────────
        if (gamepad1.cross) {
            drivetrain.resetIMU();
            drivetrain.resetPosition();
            turret.setToFacingFront();
            turret.resetEncoder();
            SharedData.hasAutonomousRun = false;
            autoAimEnabled        = false;
            lastTurretPower       = 0;
            tagWasVisible         = false;
            limelightJustTookOver = false;
            lastHeading           = 0;
            angularVelocity       = 0;
        }

        // ── Robot pose ─────────────────────────────────────────
        double robotX       = drivetrain.getX();
        double robotY       = drivetrain.getY();
        double robotHeading = drivetrain.getHeading();

        // ── Angular velocity (for heading compensation) ────────
        angularVelocity = robotHeading - lastHeading;
        lastHeading     = robotHeading;
        if (angularVelocity >  180) angularVelocity -= 360;
        if (angularVelocity < -180) angularVelocity += 360;
        angularVelocity = clamp(angularVelocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        // ── Limelight result ───────────────────────────────────
        LLResult llResult = limelight.getLatestResult();
        LLResultTypes.FiducialResult currentTag = null;
        if (llResult != null && llResult.isValid()) {
            currentTag = getBestTag(llResult.getFiducialResults());
        }

        // ── Distance — limelight if available, odometry fallback ──
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
            if (!autoAimEnabled) {
                lastTurretPower       = 0;
                tagWasVisible         = false;
                limelightJustTookOver = false;
                turret.setMotorPowerDirectly(0);
            }
        }
        optionsWasPressed = optionsNow;

        // ── Turret control ─────────────────────────────────────
        if (autoAimEnabled) {

            if (currentTag != null) {
                // ── Tag visible — limelight takes over ────────
                if (!tagWasVisible) {
                    turret.resetPID();
                    turret.setMotorPowerDirectly(0);
                    limelightJustTookOver = true;
                    tagWasVisible         = true;
                    tagLostTimer.reset();
                }

                limelightJustTookOver = false;
                tagWasVisible         = true;
                tagLostTimer.reset();

                double tx = currentTag.getTargetXDegrees() + 2.0; // 2° mounting offset

                if (Math.abs(tx) <= DEADBAND_DEG) {
                    // Locked — only apply heading compensation
                    double compensation = -HEADING_COMPENSATION * angularVelocity;
                    lastTurretPower = clamp(compensation, -LL_MAX_SPEED, LL_MAX_SPEED);
                } else {
                    double power = clamp(-kP_LIMELIGHT * tx, -LL_MAX_SPEED, LL_MAX_SPEED);
                    power += -HEADING_COMPENSATION * angularVelocity;
                    lastTurretPower = clamp(power, -LL_MAX_SPEED, LL_MAX_SPEED);
                }

                turret.setMotorPowerDirectly(lastTurretPower);

            } else {
                // ── Tag not visible — odometry takes over ─────
                if (tagWasVisible && tagLostTimer.seconds() < TAG_LOST_HOLD_SECONDS) {
                    // Brief hold at last power to avoid jerk
                    double power = lastTurretPower + (-HEADING_COMPENSATION * angularVelocity);
                    turret.setMotorPowerDirectly(clamp(power, -LL_MAX_SPEED, LL_MAX_SPEED));
                } else {
                    tagWasVisible         = false;
                    limelightJustTookOver = false;
                    lastTurretPower       = 0;

                    turret.aimAtGoal(robotX, robotY, robotHeading);
                    turret.update();

                    double feedforward = turret.getMotorPower()
                            + (-HEADING_COMPENSATION * angularVelocity);
                    turret.setMotorPowerDirectly(clamp(feedforward, -LL_MAX_SPEED, LL_MAX_SPEED));
                }
            }

        } else {
            // ── Manual turret — gamepad2 dpad ─────────────────
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

        // ── Auto-stop intake when fully loaded ─────────────────
        if (intakeTransfer.isIntaking() && intakeTransfer.isFullyLoaded()&& shootState == ShootState.IDLE) {
            intakeTransfer.setIdle();
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
            intakeTransfer.setIdle();
            shootState = ShootState.IDLE;
        }
        updateShootSequence();

        // ── Telemetry ──────────────────────────────────────────
        telemetry.addLine("=== MODE ===");
        telemetry.addData("Auto-Aim",    autoAimEnabled ? "ENABLED" : "MANUAL");
        telemetry.addData("Shoot State", shootState);

        telemetry.addLine();
        telemetry.addLine("=== INTAKE ===");
        intakeTransfer.updateTelemetry();

        telemetry.addLine();
        telemetry.addLine("=== LAUNCHER ===");
        launcher.updateTelemetry();
        telemetry.addData("Distance",       "%.1f in", distanceToGoal);
        telemetry.addData("Dist source",    currentTag != null ? "Limelight" : "Odometry");

        telemetry.addLine();
        telemetry.addLine("=== TURRET ===");
        if (autoAimEnabled) {
            telemetry.addData("Mode",        tagWasVisible ? "LIMELIGHT" : "ODOMETRY");
            telemetry.addData("Turret Angle","%.1f°", turret.getCurrentAngle());
            telemetry.addData("Last Power",  "%.3f",  lastTurretPower);
            if (currentTag != null) {
                telemetry.addData("tx raw",  "%.2f°", currentTag.getTargetXDegrees());
                telemetry.addData("ty",      "%.2f°", currentTag.getTargetYDegrees());
                telemetry.addData("Tag ID",  (int) currentTag.getFiducialId());
            }
        } else {
            turret.updateTelemetry();
        }

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
                if (intakeTransfer.isFrontBeamClear()) {
                    shootState = ShootState.WAIT_BALL_GONE;
                    shootTimer.reset();
                }
                break;
            case WAIT_BALL_GONE:
                if (shootTimer.seconds() >= BALL_CLEAR_WAIT_SEC) {
                    launcher.closeLatch();
                    intakeTransfer.setIdle();
                    shootState = ShootState.CLOSE_LATCH;
                    shootTimer.reset();
                }
                break;
            case CLOSE_LATCH:
                if (shootTimer.seconds() >= 0.1) shootState = ShootState.IDLE;
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

    private LLResultTypes.FiducialResult getBestTag(List<LLResultTypes.FiducialResult> tags) {
        if (tags == null || tags.isEmpty()) return null;
        for (LLResultTypes.FiducialResult t : tags)
            if (t.getFiducialId() == TARGET_TAG_ID) return t;
        return null;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ── Stop ───────────────────────────────────────────────────
    @Override
    public void stop() {
        launcher.stopFlywheel();
        launcher.closeLatch();
        intakeTransfer.setIdle();
        limelight.stop();
    }
}
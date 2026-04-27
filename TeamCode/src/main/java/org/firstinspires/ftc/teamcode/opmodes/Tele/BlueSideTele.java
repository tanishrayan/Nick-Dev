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

    // ── Field coordinates ─────────────────────────────────────
    private static final double GOAL_X = 72.0;
    private static final double GOAL_Y = -72.0;

    // ── Limelight constants ────────────────────────────────────
    private static final double kP_LIMELIGHT        = 0.03;
    private static final double LL_MAX_SPEED        = 0.3;
    private static final double DEADBAND_DEG        = 1.0;
    private static final double MOUNTING_OFFSET_DEG = -1.0;
    private static final int    TARGET_TAG_ID       = 20;

    // ── Limelight state ────────────────────────────────────────
    private boolean tagWasVisible = false;

    // ── Shooter state ──────────────────────────────────────────
    private double  currentVelocity = 1900.0;
    private boolean shooterRunning  = false;
    private boolean autoAimEnabled  = false;

    // ── Shooting sequence ──────────────────────────────────────
    private enum ShootState { IDLE, SPINUP, OPEN_LATCH, WAIT_BALL_GONE, CLOSE_LATCH }
    private ShootState  shootState = ShootState.IDLE;
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double BALL_CLEAR_WAIT_SEC  = 0.5;
    private static final double CLOSE_LATCH_WAIT_SEC = 0.35;

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
            tagWasVisible  = false;
        }

        double robotX       = drivetrain.getX();
        double robotY       = drivetrain.getY();
        double robotHeading = drivetrain.getHeading();

        // ── Auto-aim toggle (gamepad1 options) ────────────────
        boolean optionsNow = gamepad1.options;
        if (optionsNow && !optionsWasPressed) {
            autoAimEnabled = !autoAimEnabled;
            if (!autoAimEnabled) {
                turret.setMotorPowerDirectly(0);
                tagWasVisible = false;
            }
        }
        optionsWasPressed = optionsNow;

        // ── Turret control ─────────────────────────────────────
        if (autoAimEnabled) {
            LLResultTypes.FiducialResult tag = getTag();

            if (tagWasVisible && tag == null) {
                // Tag lost — release latch, go back to odometry
                tagWasVisible = false;
            }

            if (!tagWasVisible) {
                // Odometry coarse aim
                turret.aimAtGoal(robotX, robotY, robotHeading);
                turret.update();
                // Hand off to limelight once settled and tag visible
                if (turret.isAtTarget() && tag != null) {
                    tagWasVisible = true;
                }
            } else {
                // Limelight fine adjust
                double tx = tag.getTargetXDegrees() + MOUNTING_OFFSET_DEG;
                if (Math.abs(tx) > DEADBAND_DEG) {
                    turret.setMotorPowerDirectly(clamp(kP_LIMELIGHT * tx, -LL_MAX_SPEED, LL_MAX_SPEED));
                } else {
                    // Locked — correct encoder drift from belt skip
                    turret.setMotorPowerDirectly(0);
                    double trueAngle = turret.calculateAngleToGoal(robotX, robotY, robotHeading);
                    turret.correctEncoderFromLimelight(trueAngle);
                }
            }

        } else {
            // ── Manual dpad ───────────────────────────────────
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
        telemetry.addData("Auto-Aim",    autoAimEnabled ? "ENABLED" : "MANUAL");
        telemetry.addData("Aim Mode",    tagWasVisible  ? "LIMELIGHT" : "ODOMETRY");
        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Dipping",     dipping ? "YES" : "NO");
        telemetry.addLine();
        telemetry.addLine("=== INTAKE ===");
        intakeTransfer.updateTelemetry();
        telemetry.addLine();
        telemetry.addLine("=== LAUNCHER ===");
        launcher.updateTelemetry();
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

    // ── Limelight tag fetch ────────────────────────────────────
    private LLResultTypes.FiducialResult getTag() {
        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult t : tags)
                        if (t.getFiducialId() == TARGET_TAG_ID) return t;
                }
            }
        } catch (Exception e) { /* skip frame */ }
        return null;
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
                    shootState = ShootState.CLOSE_LATCH;
                    shootTimer.reset();
                }
                break;
            case CLOSE_LATCH:
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
    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public void stop() {
        launcher.stopFlywheel();
        launcher.closeLatch();
        intakeTransfer.setIdle();
        limelight.stop();
    }
}
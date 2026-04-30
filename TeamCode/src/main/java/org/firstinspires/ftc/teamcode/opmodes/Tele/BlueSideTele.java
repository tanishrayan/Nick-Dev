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
    private double  currentVelocity     = 1700.0;
    private double  currentHoodPosition = 0.6;
    private static final double HOOD_DEADBAND     = 0.005;
    private static final double VELOCITY_DEADBAND = 25.0;
    private boolean shooterRunning = false;
    private boolean autoAimEnabled = false;

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

    // ── Dip button (GP1 triangle) ─────────────────────────────
    private boolean     dipping  = false;
    private ElapsedTime dipTimer = new ElapsedTime();
    private static final double DIP_DOWN_SEC  = 0.35;
    private static final double DIP_TOTAL_SEC = 0.65;

    // ── Turret ─────────────────────────────────────────────────
    private static final double TURRET_INCREMENT = 5.0;

    // ── Edge detection ─────────────────────────────────────────
    private boolean optionsWasPressed      = false;
    private boolean circleWasPressed       = false;
    private boolean rightTriggerWasPressed = false;
    private boolean triangleWasPressed     = false;
    private boolean dpadLeftPressed        = false;
    private boolean dpadRightPressed       = false;
    private boolean gp2TriangleWasPressed  = false;

    // ── Distance cache ─────────────────────────────────────────
    private double distanceToGoal = 0.0;

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

        // Pose is set in start() not here — Drivetrain constructor calls
        // odo.resetPosAndIMU() which would wipe any pose set during init().
        // start() runs after all hardware is fully settled.

        turret.setGoalPosition(GOAL_X, GOAL_Y);
        launcher.setHoodPosition(currentHoodPosition);
        launcher.closeLatch();

        telemetry.addLine("Ready — waiting for start.");
        telemetry.addData("hasAutoRun", SharedData.hasAutonomousRun);
        telemetry.addData("savedPose",  SharedData.lastKnownPose);
        telemetry.update();
    }

    @Override
    public void start() {
        // Set pose here — after hardware has fully initialized and
        // odo.resetPosAndIMU() from Drivetrain constructor has completed.
        if (SharedData.hasAutonomousRun && SharedData.lastKnownPose != null) {
            drivetrain.setPose(SharedData.lastKnownPose);
        } else {
            drivetrain.setPose(new Pose(0, 0, 0));
        }
    }

    @Override
    public void loop() {

        drivetrain.handleDrivetrain(gamepad1);

        // ── Full reset (GP1 cross) ────────────────────────────
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

        // ── Distance → auto hood + velocity ───────────────────
        distanceToGoal    = turret.calculateDistanceToGoal(robotX, robotY);
        double newVelocity = launcher.calculateFlywheelVelocity(distanceToGoal);
        double newHood     = launcher.calculateHoodAngle(distanceToGoal);

        if (Math.abs(newHood - currentHoodPosition) > HOOD_DEADBAND) {
            currentHoodPosition = newHood;
            launcher.setHoodPosition(currentHoodPosition);
        }
        if (Math.abs(newVelocity - currentVelocity) > VELOCITY_DEADBAND) {
            currentVelocity = newVelocity;
            launcher.setFlywheelVelocity(currentVelocity);
        }

        // ── Bang-bang update (must run every loop) ─────────────
        launcher.update(distanceToGoal);

        // ── Auto-aim toggle (GP1 options) ─────────────────────
        boolean optionsNow = gamepad1.options;
        if (optionsNow && !optionsWasPressed) {
            autoAimEnabled = !autoAimEnabled;
            if (!autoAimEnabled) {
                turret.setMotorPowerDirectly(0);
                tagWasVisible = false;
            }
        }
        optionsWasPressed = optionsNow;

        // ── GP2 dpad — manual turret ──────────────────────────
        if (gamepad2.dpad_left && !dpadLeftPressed)
            turret.setTurretAngle(turret.getCurrentAngle() - TURRET_INCREMENT);
        dpadLeftPressed = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !dpadRightPressed)
            turret.setTurretAngle(turret.getCurrentAngle() + TURRET_INCREMENT);
        dpadRightPressed = gamepad2.dpad_right;

        // ── GP2 triangle — limelight encoder recal ────────────
        boolean gp2TriangleNow = gamepad2.triangle;
        if (gp2TriangleNow && !gp2TriangleWasPressed) {
            LLResultTypes.FiducialResult calTag = getTag();
            if (calTag != null) {
                double tx = calTag.getTargetXDegrees() + MOUNTING_OFFSET_DEG;
                double correctedAngle = turret.getCurrentAngle() + tx;
                turret.correctEncoderFromLimelight(correctedAngle);
                tagWasVisible = false;
            }
        }
        gp2TriangleWasPressed = gp2TriangleNow;

        // ── Turret auto-aim or hold ───────────────────────────
        if (autoAimEnabled) {
            LLResultTypes.FiducialResult tag = getTag();
            if (tagWasVisible && tag == null) tagWasVisible = false;

            if (!tagWasVisible) {
                turret.aimAtGoal(robotX, robotY, robotHeading);
                turret.update();
                if (turret.isAtTarget() && tag != null) tagWasVisible = true;
            } else {
                double tx = tag.getTargetXDegrees() + MOUNTING_OFFSET_DEG;
                if (Math.abs(tx) > DEADBAND_DEG) {
                    turret.setMotorPowerDirectly(
                            clamp(kP_LIMELIGHT * tx, -LL_MAX_SPEED, LL_MAX_SPEED));
                } else {
                    turret.setMotorPowerDirectly(0);
                    turret.correctEncoderFromLimelight(
                            turret.calculateAngleToGoal(robotX, robotY, robotHeading));
                }
            }
        } else {
            turret.update();
        }

        // ── Flywheel toggle (GP2 circle) ──────────────────────
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

        // ── Intake toggle (GP1 right trigger) ─────────────────
        boolean rightTriggerNow = gamepad1.right_trigger > 0.5;
        if (rightTriggerNow && !rightTriggerWasPressed) intakeTransfer.toggleIntake();
        rightTriggerWasPressed = rightTriggerNow;

        // ── Dip button (GP1 triangle) ─────────────────────────
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

        // ── Outtake (GP1 left trigger, hold) ──────────────────
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
            launcher.setBurstMode(false);
            shooterRunning = false;
            intakeTransfer.setIdle();
            shootState = ShootState.IDLE;
        }
        updateShootSequence();

        // ── Telemetry ──────────────────────────────────────────
        telemetry.addLine("=== DEBUG ===");
        telemetry.addData("Pose X",    "%.1f in", robotX);
        telemetry.addData("Pose Y",    "%.1f in", robotY);
        telemetry.addData("Distance",  "%.1f in", distanceToGoal);
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
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("GP2 Dpad = manual turret | GP2 Triangle = LL recal");
        telemetry.addLine("GP1 Options = auto-aim | GP2 Circle = flywheel");
        telemetry.addLine("GP1 RB = shoot | GP1 LB = abort | GP1 Cross = reset");
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
        launcher.setBurstMode(true);
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
                        // Go back to SPINUP — wait for flywheel to recover
                        // before opening latch for next ball.
                        shootState = ShootState.SPINUP;
                        shootTimer.reset();
                    } else {
                        intakeTransfer.setIdle();
                        launcher.stopFlywheel();
                        launcher.setBurstMode(false);
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
        launcher.setBurstMode(false);
        launcher.closeLatch();
        intakeTransfer.setIdle();
        limelight.stop();
    }
}
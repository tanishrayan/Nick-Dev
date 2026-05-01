package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeAndTransfer;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.opmodes.SharedData;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import java.util.List;

@Autonomous(name = "BlueFarTesting", group = "Autonomous")
@Configurable
public class BlueAutoFarSidePathingTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower         follower;
    private Timer            pathTimer, opmodeTimer;
    private int              pathState;

    private Drivetrain        drivetrain;
    private IntakeAndTransfer intakeTransfer;
    private Launcher          launcher;
    private Turret            turret;
    private Limelight3A       limelight;

    // ── Timing constants ──────────────────────────────────────
    private static final double BALL_CLEAR_WAIT_SEC  = 0.5;
    private static final double CLOSE_LATCH_WAIT_SEC = 0.35;

    // ── Limelight/turret constants ────────────────────────────
    private static final double kP_LIMELIGHT        = 0.03;
    private static final double LL_MAX_SPEED        = 0.3;
    private static final double DEADBAND_DEG        = 1.0;
    private static final double MOUNTING_OFFSET_DEG = -1.0;
    private static final int    TARGET_TAG_ID       = 20;
    private boolean tagWasVisible = false;

    // ── Shoot sequence ─────────────────────────────────────────
    private enum ShootSeqState { IDLE, SPINUP, OPEN_LATCH, WAIT_BALL_GONE, CLOSE_LATCH, DONE }
    private ShootSeqState shootSeqState = ShootSeqState.IDLE;
    private Timer         shootSeqTimer = new Timer();

    // ── Goal position ─────────────────────────────────────────
    private static final double GOAL_X = 72.0;
    private static final double GOAL_Y = -72.0;

    // ── Waypoints ─────────────────────────────────────────────
    private final Pose startShootPose  = new Pose(16.2,  64.2, Math.toRadians(0));
    private final Pose postIntakePose1 = new Pose(60.2,  37.2, Math.toRadians(0));
    private final Pose controlPose1    = new Pose(14.2,  35.2, Math.toRadians(0));
    private final Pose postIntakePose2 = new Pose(64.2,  64.2, Math.toRadians(0));
    private final Pose parkPose        = new Pose(14.2,  35.2, Math.toRadians(0));

    // ── Path chains ───────────────────────────────────────────
    private PathChain toIntakeCurve, toShootCurve, toIntakeLine, toShootLine, toPark;

    // ── Distance cache (updated every loop) ───────────────────
    private double distanceToGoal = 0.0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathTimer   = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        drivetrain     = new Drivetrain(hardwareMap);
        intakeTransfer = new IntakeAndTransfer(hardwareMap);
        launcher       = new Launcher(hardwareMap);
        turret         = new Turret(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        turret.setGoalPosition(GOAL_X, GOAL_Y);
        turret.resetEncoder();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        // drivetrain.setPose omitted — follower.setStartingPose() handles the
        // Pinpoint offset internally. Calling drivetrain.setPose() again
        // interferes with Pedro's localization and causes near-zero readings.

        SharedData.hasAutonomousRun = false;
        SharedData.lastKnownPose    = null;

        launcher.closeLatch();
        launcher.setHoodRetracted();
        intakeTransfer.setIdle();
        shootSeqState = ShootSeqState.IDLE;
        tagWasVisible = false;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.setStartingPose(startShootPose); // set after hardware fully settled
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // ── Distance + launcher update (every loop) ────────────
        // Must run before autonomousPathUpdate so isFlywheelReady()
        // reflects the current effectiveVelocity including burst offset.
        Pose currentPose = follower.getPose();
        distanceToGoal = turret.calculateDistanceToGoal(
                currentPose.getX(), currentPose.getY());
        launcher.update(distanceToGoal);

        updateTurretAim();
        autonomousPathUpdate();
        updateShootSequence();

        panelsTelemetry.debug("Path State",      pathState);
        panelsTelemetry.debug("Shoot Seq",       shootSeqState);
        panelsTelemetry.debug("X",               currentPose.getX());
        panelsTelemetry.debug("Y",               currentPose.getY());
        panelsTelemetry.debug("Distance",        distanceToGoal);
        panelsTelemetry.debug("Ball Count",      intakeTransfer.getBallCount());
        panelsTelemetry.debug("Flywheel Ready",  launcher.isFlywheelReady());
        panelsTelemetry.debug("Tag Visible",     tagWasVisible);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        launcher.stopFlywheel();
        launcher.setBurstMode(false);
        launcher.closeLatch();
        intakeTransfer.setIdle();
        limelight.stop();

        SharedData.lastKnownPose    = follower.getPose();
        SharedData.hasAutonomousRun = true;
    }

    // ── Path state machine ────────────────────────────────────

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Spin up for preload shot
                prepareShooter();
                setPathState(1);
                break;

            case 1: // Wait for flywheel ready then shoot
                prepareShooter();
                if (!follower.isBusy()
                        && shootSeqState == ShootSeqState.IDLE
                        && launcher.isFlywheelReady()) {
                    intakeTransfer.setIdle();
                    startShootSequence();
                    setPathState(2);
                }
                break;

            case 2:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking();
                    follower.followPath(toIntakeCurve, true);
                    setPathState(3);
                }
                break;

            case 3:
                prepareShooter();
                if (!follower.isBusy() || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIntaking();
                    follower.followPath(toShootCurve, true);
                    setPathState(4);
                }
                break;

            case 4:
                prepareShooter();
                if (!follower.isBusy()
                        && shootSeqState == ShootSeqState.IDLE
                        && launcher.isFlywheelReady()) {
                    intakeTransfer.setIdle();
                    startShootSequence();
                    setPathState(5);
                }
                break;

            case 5:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking();
                    follower.followPath(toIntakeLine, true);
                    setPathState(6);
                }
                break;

            case 6:
                prepareShooter();
                if (!follower.isBusy() || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIntaking();
                    follower.followPath(toShootLine, true);
                    setPathState(7);
                }
                break;

            case 7:
                prepareShooter();
                if (!follower.isBusy()
                        && shootSeqState == ShootSeqState.IDLE
                        && launcher.isFlywheelReady()) {
                    intakeTransfer.setIdle();
                    startShootSequence();
                    setPathState(8);
                }
                break;

            case 8:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking();
                    follower.followPath(toIntakeLine, true);
                    setPathState(9);
                }
                break;

            case 9:
                prepareShooter();
                if (!follower.isBusy() || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIntaking();
                    follower.followPath(toShootLine, true);
                    setPathState(10);
                }
                break;

            case 10:
                prepareShooter();
                if (!follower.isBusy()
                        && shootSeqState == ShootSeqState.IDLE
                        && launcher.isFlywheelReady()) {
                    intakeTransfer.setIdle();
                    startShootSequence();
                    setPathState(11);
                }
                break;

            case 11:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking();
                    follower.followPath(toIntakeLine, true);
                    setPathState(12);
                }
                break;

            case 12:
                prepareShooter();
                if (!follower.isBusy() || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIntaking();
                    follower.followPath(toShootLine, true);
                    setPathState(13);
                }
                break;

            case 13:
                prepareShooter();
                if (!follower.isBusy()
                        && shootSeqState == ShootSeqState.IDLE
                        && launcher.isFlywheelReady()) {
                    intakeTransfer.setIdle();
                    startShootSequence();
                    setPathState(14);
                }
                break;

            case 14:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    follower.followPath(toPark, true);
                    setPathState(-1);
                }
                break;
        }
    }

    // ── Turret ────────────────────────────────────────────────

    private void updateTurretAim() {
        Pose p   = follower.getPose();
        double x = p.getX();
        double y = p.getY();
        double h = Math.toDegrees(p.getHeading());

        LLResultTypes.FiducialResult tag = getTag();

        if (tagWasVisible && tag == null) tagWasVisible = false;

        if (!tagWasVisible) {
            turret.aimAtGoal(x, y, h);
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
                        turret.calculateAngleToGoal(x, y, h));
            }
        }
    }

    // ── Shoot sequence ────────────────────────────────────────

    private void startShootSequence() {
        // Enable burst mode — launcher.update() will now chase
        // effectiveVelocity = targetVelocity + burstOffset(distance).
        // SPINUP waits until isFlywheelReady() confirms the higher speed
        // is reached before opening latch.
        launcher.setBurstMode(true);
        shootSeqState = ShootSeqState.SPINUP;
        shootSeqTimer.resetTimer();
    }

    private void updateShootSequence() {
        switch (shootSeqState) {
            case SPINUP:
                // Wait for flywheel to reach effectiveVelocity (includes burst offset).
                if (launcher.isFlywheelReady()) {
                    launcher.openLatch();
                    intakeTransfer.setIntaking();
                    shootSeqState = ShootSeqState.OPEN_LATCH;
                    shootSeqTimer.resetTimer();
                }
                break;

            case OPEN_LATCH:
                if (intakeTransfer.isFrontBeamClear()) {
                    shootSeqState = ShootSeqState.WAIT_BALL_GONE;
                    shootSeqTimer.resetTimer();
                }
                break;

            case WAIT_BALL_GONE:
                if (shootSeqTimer.getElapsedTimeSeconds() >= BALL_CLEAR_WAIT_SEC) {
                    launcher.closeLatch();
                    shootSeqState = ShootSeqState.CLOSE_LATCH;
                    shootSeqTimer.resetTimer();
                }
                break;

            case CLOSE_LATCH:
                if (shootSeqTimer.getElapsedTimeSeconds() >= CLOSE_LATCH_WAIT_SEC) {
                    if (!intakeTransfer.isEmpty()) {
                        // More balls — open latch for next ball.
                        // Bang-bang + hood compensation continue between shots.
                        launcher.openLatch();
                        shootSeqState = ShootSeqState.OPEN_LATCH;
                        shootSeqTimer.resetTimer();
                    } else {
                        // All balls fired — disable burst, return to normal.
                        intakeTransfer.setIdle();
                        launcher.setBurstMode(false);
                        shootSeqState = ShootSeqState.DONE;
                    }
                }
                break;

            case IDLE:
            case DONE:
            default:
                break;
        }
    }

    // ── Helpers ───────────────────────────────────────────────

    private void prepareShooter() {
        // Uses cached distanceToGoal from loop() — no extra calculation needed.
        launcher.setFlywheelVelocity(launcher.calculateFlywheelVelocity(distanceToGoal));
        launcher.setHoodPosition(launcher.calculateHoodAngle(distanceToGoal));
    }

    private void stopShooter() {
        // stopFlywheel() already calls setBurstMode(false) internally.
        launcher.stopFlywheel();
        turret.setToFacingFront();
        turret.setMotorPowerDirectly(0);
        tagWasVisible = false;
    }

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

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ── Paths ─────────────────────────────────────────────────

    private void buildPaths() {
        toIntakeCurve = follower.pathBuilder()
                .addPath(new BezierCurve(startShootPose, controlPose1, postIntakePose1))
                .setLinearHeadingInterpolation(
                        startShootPose.getHeading(), postIntakePose1.getHeading())
                .build();

        toShootCurve = follower.pathBuilder()
                .addPath(new BezierCurve(postIntakePose1, controlPose1, startShootPose))
                .setLinearHeadingInterpolation(
                        postIntakePose1.getHeading(), startShootPose.getHeading())
                .build();

        toIntakeLine = follower.pathBuilder()
                .addPath(new BezierLine(startShootPose, postIntakePose2))
                .setLinearHeadingInterpolation(
                        startShootPose.getHeading(), postIntakePose2.getHeading())
                .build();

        toShootLine = follower.pathBuilder()
                .addPath(new BezierLine(postIntakePose2, startShootPose))
                .setLinearHeadingInterpolation(
                        postIntakePose2.getHeading(), startShootPose.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(startShootPose, parkPose))
                .setLinearHeadingInterpolation(
                        startShootPose.getHeading(), parkPose.getHeading())
                .build();
    }
}
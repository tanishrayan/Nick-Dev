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

@Autonomous(name = "RedTesting", group = "Autonomous")
@Configurable
public class RedAutoNearSidePathingTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower         follower;
    private Timer            pathTimer, opmodeTimer;
    private int              pathState;

    private Drivetrain        drivetrain;
    private IntakeAndTransfer intakeTransfer;
    private Launcher          launcher;
    private Turret            turret;
    private Limelight3A       limelight;

    // ── Hardcoded shooter values ───────────────────────────────
    private static final double HARDCODED_RPM  = 1600.0;
    private static final double HARDCODED_HOOD = 0.6;

    // ── Limelight constants ────────────────────────────────────
    private static final double kP_LIMELIGHT  = 0.015;
    private static final double LL_MAX_SPEED  = 0.5;
    private static final double DEADBAND_DEG  = 0.5;
    private static final int    TARGET_TAG_ID = 20;

    // ── Shoot sequence ─────────────────────────────────────────
    private enum ShootSeqState { IDLE, SPINUP, OPEN_LATCH, WAIT_BALL_GONE, CLOSE_LATCH, DONE }
    private ShootSeqState shootSeqState = ShootSeqState.IDLE;
    private Timer         shootSeqTimer = new Timer();
    private static final double BALL_CLEAR_WAIT_SEC  = 0.25;
    private static final double CLOSE_LATCH_WAIT_SEC = 0.15;
    private static final double SPINUP_TIMEOUT_SEC   = 3.0;

    // ── Gate intake dwell time ─────────────────────────────────
    private static final double GATE_INTAKE_SEC = 2.0;

    // ── Waypoints ─────────────────────────────────────────────
    private final Pose startPose       = new Pose(0,   0,    Math.toRadians(0));
    private final Pose shootPose1      = new Pose(-22, -50,   Math.toRadians(0));
    private final Pose controlPose1    = new Pose(-21, -77,   Math.toRadians(0));
    private final Pose postIntakePose1 = new Pose(16,  -74.5, Math.toRadians(0));
    private final Pose shootPose2      = new Pose(-22, -50,   Math.toRadians(0));
    private final Pose gateOpener      = new Pose(22,  -70.5, Math.toRadians(40));
    private final Pose shootPose3      = new Pose(-22, -50,   Math.toRadians(0));
    private final Pose shootPose4      = new Pose(-22, -50,   Math.toRadians(0));
    private final Pose postIntakePose2 = new Pose(16,  -50,   Math.toRadians(0));
    private final Pose shootPose5      = new Pose(-22, -50,   Math.toRadians(0));
    private final Pose parkPose        = new Pose(5,   -50,   Math.toRadians(0));

    // ── Path chains ───────────────────────────────────────────
    private PathChain toShoot1, toIntake1, toShoot2;
    private PathChain openGate1, toShoot3, openGate2, toShoot4;
    private PathChain toIntake2, toShoot5, toPark;

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

        turret.setGoalPosition(32, -15);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        launcher.closeLatch();
        launcher.setHoodPosition(HARDCODED_HOOD);
        intakeTransfer.setIdle();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        updateTurretAim();
        autonomousPathUpdate();
        updateShootSequence();

        panelsTelemetry.debug("Path State",     pathState);
        panelsTelemetry.debug("Shoot Seq",      shootSeqState);
        panelsTelemetry.debug("X",              follower.getPose().getX());
        panelsTelemetry.debug("Y",              follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)",  Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Ball Count",     intakeTransfer.getBallCount());
        panelsTelemetry.debug("Flywheel Ready", launcher.isFlywheelReady());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        launcher.stopFlywheel();
        launcher.closeLatch();
        intakeTransfer.setIdle();
        limelight.stop();

        SharedData.lastKnownPose    = follower.getPose();
        SharedData.hasAutonomousRun = true;
    }

    // ── Path state machine ────────────────────────────────────
    //
    // State map:
    //  0  — drive to shoot 1 (spin up on way)
    //  1  — wait to arrive, start shoot
    //  2  — wait shoot done, start intake, drive to intake 1
    //  3  — intaking at intake 1, wait for full or path done
    //  4  — drive to shoot 2 (spin up on way)
    //  5  — wait to arrive, start shoot
    //  6  — wait shoot done, drive to gate 1
    //  7  — arrived at gate 1, start intake, wait 2s
    //  8  — drive to shoot 3 (spin up on way)
    //  9  — wait to arrive, start shoot
    // 10  — wait shoot done, drive to gate 2
    // 11  — arrived at gate 2, start intake, wait 2s
    // 12  — drive to shoot 4 (spin up on way)
    // 13  — wait to arrive, start shoot
    // 14  — wait shoot done, start intake, drive to intake 2
    // 15  — intaking at intake 2, wait for full or path done
    // 16  — drive to shoot 5 (spin up on way)
    // 17  — wait to arrive, start shoot
    // 18  — wait shoot done, park
    // 19  — wait to park, done

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(toShoot1);
                prepareShooter();
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    startShootSequence();
                    setPathState(2);
                }
                break;

            case 2:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking();
                    follower.followPath(toIntake1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIdle();
                    prepareShooter();
                    follower.followPath(toShoot2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    startShootSequence();
                    setPathState(5);
                }
                break;

            case 5:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    follower.followPath(openGate1, true);
                    setPathState(6);
                }
                break;

            case 6: // Arrived at gate 1 — start intake and wait
                if (!follower.isBusy()) {
                    intakeTransfer.setIntaking();
                    setPathState(7);
                }
                break;

            case 7: // Dwell at gate 1 for 2 seconds intaking
                if (pathTimer.getElapsedTimeSeconds() >= GATE_INTAKE_SEC || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIdle();
                    prepareShooter();
                    follower.followPath(toShoot3, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    startShootSequence();
                    setPathState(9);
                }
                break;

            case 9:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    follower.followPath(openGate2, true);
                    setPathState(10);
                }
                break;

            case 10: // Arrived at gate 2 — start intake and wait
                if (!follower.isBusy()) {
                    intakeTransfer.setIntaking();
                    setPathState(11);
                }
                break;

            case 11: // Dwell at gate 2 for 2 seconds intaking
                if (pathTimer.getElapsedTimeSeconds() >= GATE_INTAKE_SEC || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIdle();
                    prepareShooter();
                    follower.followPath(toShoot4, true);
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    startShootSequence();
                    setPathState(13);
                }
                break;

            case 13:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking();
                    follower.followPath(toIntake2, true);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy() || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIdle();
                    prepareShooter();
                    follower.followPath(toShoot5, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    startShootSequence();
                    setPathState(16);
                }
                break;

            case 16:
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    follower.followPath(toPark, true);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    // ── Turret aim — limelight if visible, odometry fallback ──

    private void updateTurretAim() {
        LLResultTypes.FiducialResult tag = getTag();
        Pose p = follower.getPose();

        if (tag != null) {
            double tx = tag.getTargetXDegrees() + 2.0;
            if (Math.abs(tx) <= DEADBAND_DEG) {
                turret.setMotorPowerDirectly(0);
            } else {
                double power = clamp(-kP_LIMELIGHT * tx, -LL_MAX_SPEED, LL_MAX_SPEED);
                turret.setMotorPowerDirectly(power);
            }
        } else {
            turret.aimAtGoal(p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
            turret.update();
        }
    }

    private LLResultTypes.FiducialResult getTag() {
        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult t : tags) {
                        if (t.getFiducialId() == TARGET_TAG_ID) return t;
                    }
                }
            }
        } catch (Exception e) {
            // limelight hiccuped, fall through to odometry
        }
        return null;
    }

    // ── Shoot sequence ─────────────────────────────────────────

    private void startShootSequence() {
        shootSeqState = ShootSeqState.SPINUP;
        shootSeqTimer.resetTimer();
    }

    private void updateShootSequence() {
        switch (shootSeqState) {
            case SPINUP:
                if (launcher.isFlywheelReady() || shootSeqTimer.getElapsedTimeSeconds() >= SPINUP_TIMEOUT_SEC) {
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
                        launcher.openLatch();
                        shootSeqState = ShootSeqState.OPEN_LATCH;
                        shootSeqTimer.resetTimer();
                    } else {
                        intakeTransfer.setIdle();
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
        launcher.setFlywheelVelocity(HARDCODED_RPM);
        launcher.setHoodPosition(HARDCODED_HOOD);
    }

    private void stopShooter() {
        launcher.stopFlywheel();
        turret.setToFacingFront();
        turret.setMotorPowerDirectly(0);
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
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        toIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, controlPose1, postIntakePose1))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), postIntakePose1.getHeading())
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(postIntakePose1, controlPose1, shootPose2))
                .setLinearHeadingInterpolation(postIntakePose1.getHeading(), shootPose2.getHeading())
                .build();

        openGate1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, controlPose1, gateOpener))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gateOpener.getHeading())
                .build();

        toShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(gateOpener, controlPose1, shootPose3))
                .setLinearHeadingInterpolation(gateOpener.getHeading(), shootPose3.getHeading())
                .build();

        openGate2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose3, controlPose1, gateOpener))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), gateOpener.getHeading())
                .build();

        toShoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(gateOpener, controlPose1, shootPose4))
                .setLinearHeadingInterpolation(gateOpener.getHeading(), shootPose4.getHeading())
                .build();

        toIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose4, postIntakePose2))
                .setLinearHeadingInterpolation(shootPose4.getHeading(), postIntakePose2.getHeading())
                .build();

        toShoot5 = follower.pathBuilder()
                .addPath(new BezierLine(postIntakePose2, shootPose5))
                .setLinearHeadingInterpolation(postIntakePose2.getHeading(), shootPose5.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose5, parkPose))
                .setLinearHeadingInterpolation(shootPose5.getHeading(), parkPose.getHeading())
                .build();
    }
}
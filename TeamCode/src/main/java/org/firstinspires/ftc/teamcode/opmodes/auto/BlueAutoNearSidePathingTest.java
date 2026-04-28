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

@Autonomous(name = "BlueNearTesting", group = "Autonomous")
@Configurable
public class BlueAutoNearSidePathingTest extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower         follower;
    private Timer            pathTimer, opmodeTimer;
    private int              pathState;

    // ── Fixed shooter values ───────────────────────────────────
    private static final double FIXED_RPM        = 1700.0;
    private static final double FIXED_HOOD       = 0.6;
    private static final double FIXED_TURRET_DEG = -45.6;

    private Drivetrain        drivetrain;
    private IntakeAndTransfer intakeTransfer;
    private Launcher          launcher;
    private Turret            turret;
    private Limelight3A       limelight;

    // ── Timing constants ──────────────────────────────────────
    private static final double SPINUP_TIMEOUT_SEC   = 3.0;
    private static final double BALL_CLEAR_WAIT_SEC  = 0.5;
    private static final double CLOSE_LATCH_WAIT_SEC = 0.35;
    private static final double GATE_INTAKE_SEC      = 1.0;

    // ── Limelight/turret constants ────────────────────────────
    private static final double kP_LIMELIGHT        = 0.03;
    private static final double LL_MAX_SPEED        = 0.3;
    private static final double DEADBAND_DEG        = 1.0;
    private static final double MOUNTING_OFFSET_DEG = -1.0;
    private static final int    TARGET_TAG_ID       = 20;
    private boolean tagWasVisible = false;

    // ── Shoot sequence ─────────────────────────────────────────
    private enum ShootSeqState { IDLE, OPEN_LATCH, WAIT_BALL_GONE, CLOSE_LATCH, DONE }
    private ShootSeqState shootSeqState = ShootSeqState.IDLE;
    private Timer         shootSeqTimer = new Timer();

    // ── Goal position ─────────────────────────────────────────
    private static final double GOAL_X = 72.0;
    private static final double GOAL_Y = -72.0;

    // ── Waypoints ─────────────────────────────────────────────
    private final Pose startPose       = new Pose(35.3,  -62.1, Math.toRadians(0)); //-35.3, +62.1
    private final Pose shootPose1      = new Pose(13.3,  -12.1, Math.toRadians(0));
    private final Pose controlPose1    = new Pose(14.3,   14.9, Math.toRadians(0));
    private final Pose postIntakePose1 = new Pose(62.0,    7.9, Math.toRadians(0));
    private final Pose shootPose2      = new Pose(13.3,  -12.1, Math.toRadians(0));
    private final Pose gateOpener      = new Pose(60.2 ,   10.5, Math.toRadians(-45.5));
    private final Pose shootPose3      = new Pose(13.3,  -12.1, Math.toRadians(0));
    private final Pose shootPose4      = new Pose(13.3,  -12.1, Math.toRadians(0));
    private final Pose postIntakePose2 = new Pose(52.5,  -16.1, Math.toRadians(0));
    private final Pose shootPose5      = new Pose(13.3,  -12.1, Math.toRadians(0));
    private final Pose parkPose        = new Pose(40.3,  -12.1, Math.toRadians(0));

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

        turret.setGoalPosition(GOAL_X, GOAL_Y);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        launcher.closeLatch();
        launcher.setHoodRetracted();
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
        panelsTelemetry.debug("Ball Count",     intakeTransfer.getBallCount());
        panelsTelemetry.debug("Flywheel Ready", launcher.isFlywheelReady());
        panelsTelemetry.debug("Tag Visible",    tagWasVisible);
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

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Drive to shoot 1, spin up immediately
                follower.followPath(toShoot1);
                prepareShooter();
                setPathState(1);
                break;

            case 1: // Spin up every loop, shoot when arrived + ready
                prepareShooter();
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    startShootSequence();
                    setPathState(2);
                }
                break;

            case 2: // Wait shoot done, start intake, drive to intake 1
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking(); // intake on the way
                    follower.followPath(toIntake1, true);
                    setPathState(3);
                }
                break;

            case 3: // Driving to intake 1 with intake running, spin up on the way back
                prepareShooter();
                if (!follower.isBusy() || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIntaking(); // keep intake running and pivot down while driving back
                    follower.followPath(toShoot2, true);
                    setPathState(4);
                }
                break;

            case 4: // Spin up every loop, shoot when arrived + ready
                prepareShooter();
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    intakeTransfer.setIdle(); // now safe to retract
                    startShootSequence();
                    setPathState(5);
                }
                break;

            case 5: // Wait shoot done, start intake, drive to gate 1 with intake running
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking(); // intake running while driving to gate
                    follower.followPath(openGate1, true);
                    setPathState(6);
                }
                break;

            case 6: // Arrived at gate 1 — intake already running, just wait
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

            case 7: // Dwell at gate 1 with intake running, spin up while waiting
                prepareShooter();
                if (pathTimer.getElapsedTimeSeconds() >= GATE_INTAKE_SEC || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIdle();
                    follower.followPath(toShoot3, true);
                    setPathState(8);
                }
                break;

            case 8: // Spin up every loop, shoot when arrived + ready
                prepareShooter();
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    startShootSequence();
                    setPathState(9);
                }
                break;

            case 9: // Wait shoot done, start intake, drive to gate 2 with intake running
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking(); // intake running while driving to gate
                    follower.followPath(openGate2, true);
                    setPathState(10);
                }
                break;

            case 10: // Arrived at gate 2 — intake already running, just wait
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;

            case 11: // Dwell at gate 2 with intake running, spin up while waiting
                prepareShooter();
                if (pathTimer.getElapsedTimeSeconds() >= GATE_INTAKE_SEC || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIdle();
                    follower.followPath(toShoot4, true);
                    setPathState(12);
                }
                break;

            case 12: // Spin up every loop, shoot when arrived + ready
                prepareShooter();
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    startShootSequence();
                    setPathState(13);
                }
                break;

            case 13: // Wait shoot done, start intake, drive to intake 2 with intake running
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    intakeTransfer.setIntaking(); // intake on the way
                    follower.followPath(toIntake2, true);
                    setPathState(14);
                }
                break;

            case 14: // Driving to intake 2 with intake running, spin up on the way back
                prepareShooter();
                if (!follower.isBusy() || intakeTransfer.isFullyLoaded()) {
                    intakeTransfer.setIntaking(); // keep intake running and pivot down while driving back
                    follower.followPath(toShoot5, true);
                    setPathState(4);
                }
                break;

            case 15: // Spin up every loop, shoot when arrived + ready
                prepareShooter();
                if (!follower.isBusy() && shootSeqState == ShootSeqState.IDLE) {
                    intakeTransfer.setIdle(); // now safe to retract
                    startShootSequence();
                    setPathState(5);
                }
                break;

            case 16: // Wait shoot done, park
                if (shootSeqState == ShootSeqState.DONE) {
                    shootSeqState = ShootSeqState.IDLE;
                    stopShooter();
                    follower.followPath(toPark, true);
                    setPathState(17);
                }
                break;

            case 17: // Wait to park
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    // ── Turret — limelight fine, odometry coarse ──────────────

    private void updateTurretAim() {
        Pose p   = follower.getPose();
        double x = p.getX();
        double y = p.getY();
        double h = Math.toDegrees(p.getHeading());

        LLResultTypes.FiducialResult tag = getTag();

        if (tagWasVisible && tag == null) {
            tagWasVisible = false;
        }

        if (!tagWasVisible) {
            turret.aimAtGoal(x, y, h);
            turret.update();
            if (turret.isAtTarget() && tag != null) {
                tagWasVisible = true;
            }
        } else {
            double tx = tag.getTargetXDegrees() + MOUNTING_OFFSET_DEG;
            if (Math.abs(tx) > DEADBAND_DEG) {
                turret.setMotorPowerDirectly(clamp(kP_LIMELIGHT * tx, -LL_MAX_SPEED, LL_MAX_SPEED));
            } else {
                turret.setMotorPowerDirectly(0);
                turret.correctEncoderFromLimelight(turret.calculateAngleToGoal(x, y, h));
            }
        }
    }

    // ── Shoot sequence ────────────────────────────────────────

    private void startShootSequence() {
        // Flywheel already running from prepareShooter() — skip spinup, open immediately
        launcher.openLatch();
        intakeTransfer.setIntaking();
        shootSeqState = ShootSeqState.OPEN_LATCH;
        shootSeqTimer.resetTimer();
    }

    private void updateShootSequence() {
        switch (shootSeqState) {
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
        Pose p = follower.getPose();
        double distance = turret.calculateDistanceToGoal(p.getX(), p.getY());
        launcher.setFlywheelVelocity(launcher.calculateFlywheelVelocity(distance));
        launcher.setHoodPosition(launcher.calculateHoodAngle(distance));
    }

    private void stopShooter() {
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
        } catch (Exception e) { /* skip */ }
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
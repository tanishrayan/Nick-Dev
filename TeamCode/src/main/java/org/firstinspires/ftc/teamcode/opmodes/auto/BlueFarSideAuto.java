package org.firstinspires.ftc.teamcode.opmodes.auto;

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

@Autonomous(name = "Blue Far Side Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class BlueFarSideAuto extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private static final double HARDCODED_RPM = 2200.0;      // Set your RPM here
    private static final double HARDCODED_HOOD = 0.6;
    private static final double HARDCODED_TURRET = 0.53;

    private Drivetrain drivetrain;
    private IntakeAndTransfer intakeTransfer;
    private Launcher launcher;
    private Turret turret;

    // Timing constants
    private static final double FLYWHEEL_SPINUP_TIME = 0.5;
    private static final double TRANSFER_TIME = 1.8;

    // Define waypoints
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose leavePose = new Pose(20, 0, Math.toRadians(0));
    private final Pose preIntakePose1 = new Pose(-35, 71.5, Math.toRadians(0));
    private final Pose intakePose1 = new Pose(21.5, 71.5, Math.toRadians(0));
    private final Pose shootPose2 = new Pose(-22, 53, Math.toRadians(0));
    private final Pose preGateOpener = new Pose(-22, 65, Math.toRadians(0));
    private final Pose gateOpener = new Pose(18, 67, Math.toRadians(-40));
    private final Pose gateBackOff = new Pose(10, 74, Math.toRadians(0));
    private final Pose gateComeIn = new Pose(24, 74, Math.toRadians(0));
    private final Pose strafeLeft = new Pose(24, 82, Math.toRadians(0));
    private final Pose shootPose3 = new Pose(-22, 53, Math.toRadians(0));
    private final Pose preIntakePose2 = new Pose(-28, 47, Math.toRadians(0));
    private final Pose intakePose2 = new Pose(15, 47, Math.toRadians(0));
    private final Pose shootPose4 = new Pose(-22 , 53, Math.toRadians(0));
    private final Pose parkPose = new Pose(5, 53, Math.toRadians(0));

    // Path chains
    private PathChain leaveZone, toPreIntake1, toIntake1, toShoot2;
    private PathChain gate0, gate1, gate2, gate3, gate4, toShoot3;
    private PathChain toPreIntake2, toIntake2, toShoot4, toPark;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize subsystems
        drivetrain = new Drivetrain(hardwareMap);
        intakeTransfer = new IntakeAndTransfer(hardwareMap);
        launcher = new Launcher(hardwareMap);
        turret = new Turret(hardwareMap);

        // Set goal position (basket coordinates)
        turret.setGoalPosition(32, -15);

        // Create follower and build paths
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

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
        autonomousPathUpdate();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.update(telemetry);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Save final pose for TeleOp
        SharedData.lastKnownPose = follower.getPose();
        SharedData.hasAutonomousRun = true;

        telemetry.addData("Final Pose Saved", "X=%.1f Y=%.1f H=%.1f",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
    private void buildPaths() {
        leaveZone = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leavePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leavePose.getHeading())
                .build();

        toIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(preIntakePose1, intakePose1))
                .setLinearHeadingInterpolation(preIntakePose1.getHeading(), intakePose1.getHeading())
                .build();

        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose1, shootPose2))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), shootPose2.getHeading())
                .build();

        gate0 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, preGateOpener))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), preGateOpener.getHeading())
                .build();

        gate1 = follower.pathBuilder()
                .addPath(new BezierLine(preGateOpener, gateOpener))
                .setLinearHeadingInterpolation(preGateOpener.getHeading(), gateOpener.getHeading())
                .build();

        //shootPose2, gateOpener, gateBackOff, gateComeIn, ShootPose3

        gate2 = follower.pathBuilder()
                .addPath(new BezierLine(gateOpener, gateBackOff))
                .setLinearHeadingInterpolation(gateOpener.getHeading(), gateBackOff.getHeading())
                .build();

        gate3 = follower.pathBuilder()
                .addPath(new BezierLine(gateBackOff, gateComeIn))
                .setLinearHeadingInterpolation(gateBackOff.getHeading(), gateComeIn.getHeading())
                .build();

        gate4 = follower.pathBuilder()
                .addPath(new BezierLine(gateComeIn, strafeLeft))
                .setLinearHeadingInterpolation(gateComeIn.getHeading(), strafeLeft.getHeading())
                .build();

        toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(strafeLeft, shootPose3))
                .setLinearHeadingInterpolation(strafeLeft.getHeading(), shootPose3.getHeading())
                .build();

        toPreIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, preIntakePose2))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), preIntakePose2.getHeading())
                .build();

        toIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(preIntakePose2, intakePose2))
                .setLinearHeadingInterpolation(preIntakePose2.getHeading(), intakePose2.getHeading())
                .build();

        toShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(intakePose2, shootPose4))
                .setLinearHeadingInterpolation(intakePose2.getHeading(), shootPose4.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose3, parkPose))
                .setLinearHeadingInterpolation(shootPose4.getHeading(), parkPose.getHeading())
                .build();
    }

    //private PathChain toShoot1, toPreIntake1, toIntake1, toShoot2;
    //    private PathChain gate1, gate2, gate3, toShoot3;
    //    private PathChain toPreIntake2, toIntake2, toShoot4, toPark;
    private void autonomousPathUpdate() {
        switch (pathState) {
            // ===== CYCLE 1: PRELOAD SHOOTING =====
            case 0:
                SharedData.lastKnownPose = follower.getPose();
                SharedData.hasAutonomousRun = true;
                // Prepare shooter at shooting position
                if (!follower.isBusy()) {
                    prepareShooter();
                    setPathState(1);
                }
                break;

            case 1:
                // Wait for flywheel spinup, then shoot
                if (pathTimer.getElapsedTimeSeconds() >= FLYWHEEL_SPINUP_TIME) {
                    //intakeTransfer.setTransfer();
                    setPathState(2);
                } else {
                    // Keep updating aim during spinup
                    updateAim();
                }
                break;

            case 2:
                // Wait for transfer to complete, then move to intake
                if (pathTimer.getElapsedTimeSeconds() >= TRANSFER_TIME) {
                    stopShooter();
                    follower.followPath(leaveZone, true);
                    setPathState(3);
                }
                break;

            // ===== CYCLE 2: FIRST SAMPLE =====
            case 3:
                SharedData.lastKnownPose = follower.getPose();
                SharedData.hasAutonomousRun = true;
                // Wait for parking to complete
                if (!follower.isBusy()) {
                    setPathState(-1);  // Done
                }
                break;
        }
    }

    // ===== HELPER METHODS =====

    /**
     * Prepare shooter systems with calculated parameters
     */
    private void prepareShooter() {
        Pose currentPose = follower.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeading = Math.toDegrees(currentPose.getHeading());

        double distance = turret.calculateDistanceToGoal(robotX, robotY);
        double targetVelocity = launcher.calculateFlywheelVelocity(distance);
        double targetHood = launcher.calculateHoodAngle(distance);
        double angleToGoal = turret.calculateAngleToGoal(robotX, robotY, robotHeading);

        launcher.setFlywheelVelocity(HARDCODED_RPM);
        launcher.setHoodPosition(HARDCODED_HOOD);
        //turret.setRawServoPosition(HARDCODED_TURRET);

        // DEBUG: Check deadzone status
        telemetry.addData("Goal Position", "(%.1f, %.1f)", 32.0, 50.0);
        telemetry.addData("Robot X", "%.1f", robotX);
        telemetry.addData("Robot Y", "%.1f", robotY);
        telemetry.addData("Robot Heading", "%.1f°", robotHeading);
        telemetry.addData("Calculated Turret Angle", "%.1f°", angleToGoal);
        telemetry.addData("Actual Turret Angle", "%.1f°", turret.getCurrentAngle());
        //telemetry.addData("Turret Servo Pos", "%.3f", turret.getCurrentPosition());
        telemetry.addData("In Deadzone?", turret.isInDeadzone(angleToGoal) ? "YES" : "NO");
        telemetry.update();
    }

    /**
     * Update turret aim (for continuous tracking during spinup)
     */
    private void updateAim() {
        Pose currentPose = follower.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeading = Math.toDegrees(currentPose.getHeading());
        //turret.setRawServoPosition(HARDCODED_TURRET);
    }

    /**
     * Stop all shooter systems
     */
    private void stopShooter() {
        //intakeTransfer.setStatic();
        launcher.stopFlywheel();
    }

    /**
     * Set path state and reset timer
     */
    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
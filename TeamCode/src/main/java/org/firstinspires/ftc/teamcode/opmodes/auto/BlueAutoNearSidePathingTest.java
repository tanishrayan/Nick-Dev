package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "Blue Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class BlueAutoNearSidePathingTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private static final double HARDCODED_RPM = 1600.0;      // Set your RPM here
    private static final double HARDCODED_HOOD = 0.6;
    private static final double HARDCODED_TURRET = 0.42;

    private Drivetrain drivetrain;
    private IntakeAndTransfer intakeTransfer;
    private Launcher launcher;
    private Turret turret;

    // Timing constants
    private static final double FLYWHEEL_SPINUP_TIME = 0.5;
    private static final double TRANSFER_TIME = 1.8;

    // Define waypoints
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose shootPose1 = new Pose(-22, 50, Math.toRadians(0));
    private final Pose controlPose1 = new Pose(-21, 77, Math.toRadians(0));
    private final Pose postIntakePose1 = new Pose(16, 74.5, Math.toRadians(0));
    private final Pose shootPose2 = new Pose(-22, 50, Math.toRadians(0));
    private final Pose gateOpener = new Pose(22, 70.5, Math.toRadians(-40));
    private final Pose shootPose3 = new Pose(-22, 50, Math.toRadians(0));
    private final Pose shootPose4 = new Pose(-22 , 50, Math.toRadians(0));
    private final Pose postIntakePose2 = new Pose(16, 50, Math.toRadians(0));
    private final Pose shootPose5 = new Pose(-22 , 50, Math.toRadians(0));
    private final Pose parkPose = new Pose(5, 50, Math.toRadians(0));


    // Path chains
    private PathChain toShoot1, toIntake1, toShoot2;
    private PathChain openGate1, openGate2, toShoot3;
    private PathChain toIntake2, toShoot4, toShoot5, toPark;


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
                .addPath(new BezierLine(shootPose3, parkPose))
                .setLinearHeadingInterpolation(shootPose4.getHeading(), parkPose.getHeading())
                .build();
    }
    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Drive to shoot position 1
                follower.followPath(toShoot1);
                setPathState(1);
                break;

            case 1: // Arrived at shoot 1 -> intake 1
                if (!follower.isBusy()) {
                    follower.followPath(toIntake1, true);
                    setPathState(2);
                }
                break;

            case 2: // Finished intake 1 -> shoot 2
                if (!follower.isBusy()) {
                    follower.followPath(toShoot2, true);
                    setPathState(3);
                }
                break;

            case 3: // Arrived at shoot 2 -> open gate 1
                if (!follower.isBusy()) {
                    follower.followPath(openGate1, true);
                    setPathState(4);
                }
                break;

            case 4: // Finished gate 1 -> shoot 3
                if (!follower.isBusy()) {
                    follower.followPath(toShoot3, true);
                    setPathState(5);
                }
                break;

            case 5: // Arrived at shoot 3 -> open gate 2
                if (!follower.isBusy()) {
                    follower.followPath(openGate2, true);
                    setPathState(6);
                }
                break;

            case 6: // Finished gate 2 -> shoot 4
                if (!follower.isBusy()) {
                    follower.followPath(toShoot4, true);
                    setPathState(7);
                }
                break;

            case 7: // Arrived at shoot 4 -> intake 2
                if (!follower.isBusy()) {
                    follower.followPath(toIntake2, true);
                    setPathState(8);
                }
                break;

            case 8: // Finished intake 2 -> shoot 5
                if (!follower.isBusy()) {
                    follower.followPath(toShoot5, true);
                    setPathState(9);
                }
                break;

            case 9: // Arrived at shoot 5 -> park
                if (!follower.isBusy()) {
                    follower.followPath(toPark, true);
                    setPathState(10);
                }
                break;

            case 10: // Done
                if (!follower.isBusy()) {
                    setPathState(-1);
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
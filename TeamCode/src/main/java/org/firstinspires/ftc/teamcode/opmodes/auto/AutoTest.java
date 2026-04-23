package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "Forward Test", group = "Test")
public class AutoTest extends OpMode {
    private Follower follower;
    private PathChain forwardPath;

    private PathChain rightPath;
    private int pathState = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Start at origin, facing 0 degrees (straight ahead)
        Pose startPose = new Pose(0, 24, Math.toRadians(0));
        follower.setStartingPose(startPose);

        // Move straight forward 24 inches (only Y changes, X stays 0, heading stays 0)
        Pose endPose = new Pose(0, 0, Math.toRadians(0));

        Pose hehe = new Pose(24, 0, Math.toRadians(0));

        // Build the path
        forwardPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        rightPath = follower.pathBuilder()
                .addPath(new BezierLine(endPose, hehe))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Path", "Forward 24 inches");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                // Start moving forward
                follower.followPath(forwardPath);
                pathState = 1;
                break;

            case 1:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    follower.followPath(rightPath);
                    pathState = 2; // Done
                }
                break;
            case 2:
                // Wait for path to complete
                if (!follower.isBusy()) {
                    pathState = 3; // Done
                }
                break;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("Is Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
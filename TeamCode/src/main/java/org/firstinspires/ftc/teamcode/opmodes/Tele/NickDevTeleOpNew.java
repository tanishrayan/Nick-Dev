package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "nick dev teleop")
public class NickDevTeleOpNew extends OpMode {
    private Follower follower;
    public static Pose startPose;

    public Pose currentBotPose;
    private DriveSubsystem drive;

    private double angleToGoalFieldRad = 0;

    private double angleToGoalRobotRelativeDegrees = 0;


    // need to add coords for blue goal
    // js set up blue side first and get that working
    private double blueGoalXPosition;
    private double blueGoalYPosition;

    private double distanceToGoal = 0;


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        // start in left bottom corner of the field before u initialize the program
        // that will be around 8,8 coord but you can fine tune this
        // later youll need to pass in end pose from auto but for testing use this
        follower.setStartingPose(new Pose(8, 8, Math.toRadians(90)));
        follower.update();
        drive = new DriveSubsystem(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            // you can add reset imu for driver
        }

        // add ur drive stuff using controller
        //placeholders but change so u can drive and test localization
        drive.drive(0,0,0,0);

        follower.update();

        currentBotPose = follower.getPose();

        double xDiff = (blueGoalXPosition - currentBotPose.getX());
        double yDiff = (blueGoalYPosition - currentBotPose.getY());
        double term1 = xDiff * xDiff;
        double term2 = yDiff * yDiff;
        double sum = term1 + term2;
        distanceToGoal = Math.sqrt(sum);

        angleToGoalFieldRad = Math.atan2(yDiff, xDiff);

        angleToGoalRobotRelativeDegrees = Math.toDegrees(AngleUnit.normalizeRadians(angleToGoalFieldRad));


        telemetry.addData("distance to goal", distanceToGoal);
        telemetry.addData("angle to goal degrees", angleToGoalRobotRelativeDegrees);
        telemetry.addData("x", currentBotPose.getX());
        telemetry.addData("y", currentBotPose.getY());
        telemetry.addData("heading degrees", Math.toDegrees(currentBotPose.getHeading()));



    }
}

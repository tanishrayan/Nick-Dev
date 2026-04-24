package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain {

    // ── Speed cap ──────────────────────────────────────────────
    private static final double MAX_SPEED = 0.8; // 0.0 to 1.0

    // ── Hardware ───────────────────────────────────────────────
    private DcMotor frontRight, frontLeft, backRight, backLeft;
    public IMU imu;
    private GoBildaPinpointDriver odo;

    double botHeadingIMU;

    public Drivetrain(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Initialize Pinpoint odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(81.3, 63.4, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        botHeadingIMU = imu.getRobotYawPitchRollAngles().getYaw();
    }

    public void handleDrivetrain(Gamepad gp1) {
        odo.update();

        double rightStickX = gp1.right_stick_x;
        if (Math.abs(rightStickX) < 0.05) rightStickX = 0;

        double x  = -gp1.left_stick_x;
        double y  = -gp1.left_stick_y;
        double rx = rightStickX;

        drive(y, x, rx);
    }

    public void drive(double forward, double strafe, double rotate) {
        double theta   = Math.atan2(forward, strafe);
        double r       = Math.hypot(strafe, forward);
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotatedTheta = AngleUnit.normalizeRadians(theta - heading);

        double f = r * Math.sin(rotatedTheta);
        double s = r * Math.cos(rotatedTheta);

        double frontLeftPower  = f + s + rotate;
        double backLeftPower   = f - s + rotate;
        double frontRightPower = f - s - rotate;
        double backRightPower  = f + s - rotate;

        // Normalize so no value exceeds 1.0, then apply speed cap
        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.abs(backRightPower)))));

        frontLeft.setPower(frontLeftPower   / max * MAX_SPEED);
        backLeft.setPower(backLeftPower     / max * MAX_SPEED);
        frontRight.setPower(frontRightPower / max * MAX_SPEED);
        backRight.setPower(backRightPower   / max * MAX_SPEED);
    }

    public void setPose(Pose pose) {
        if (odo != null) {
            odo.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    pose.getX(),
                    pose.getY(),
                    AngleUnit.RADIANS,
                    pose.getHeading()
            ));
            odo.update();
        }
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public Pose2D getPosition() {
        return odo.getPosition();
    }

    public double getX() {
        return odo.getPosition().getX(DistanceUnit.INCH);
    }

    public double getY() {
        return odo.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public double getHeadingRadians() {
        return odo.getPosition().getHeading(AngleUnit.RADIANS);
    }

    public void resetPosition() {
        odo.resetPosAndIMU();
    }

    public void setPosition(Pose2D newPosition) {
        odo.setPosition(newPosition);
    }
}
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    private DcMotorEx turretMotor;
    private Telemetry telemetry;

    // ── Encoder constants ─────────────────────────────────────
    private static final double TICKS_PER_TURRET_REV = 1253.0; // measured physically

    // ── Soft limits ───────────────────────────────────────────
    private static final double MAX_ANGLE = 180.0;
    private static final double MIN_ANGLE = -178.0;

    // ── PID ───────────────────────────────────────────────────
    private static final double kP        = 0.025;
    private static final double kI        = 0.0;
    private static final double kD        = 0.0;
    private static final double MAX_POWER = 0.65;

    // Two-zone min power
    private static final double MIN_POWER_FAR   = 0.1;  // error > FAR_THRESHOLD ticks
    private static final double MIN_POWER_CLOSE = 0.08;  // error <= FAR_THRESHOLD ticks
    private static final double FAR_THRESHOLD   = 100.0; // ticks

    private static final int AT_TARGET_THRESHOLD = 8; // ticks deadband

    // ── Goal ──────────────────────────────────────────────────
    private double goalX = -72.0;
    private double goalY = 72.0;
    private static final double TURRET_OFFSET_X = 3.0;

    // ── PID state ─────────────────────────────────────────────
    private double      targetAngleDegrees = 0.0;
    private double      lastError          = 0.0;
    private double      integralSum        = 0.0;
    private ElapsedTime pidTimer           = new ElapsedTime();

    // ── Constructor ───────────────────────────────────────────
    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidTimer.reset();
    }

    // ── Call every loop ───────────────────────────────────────
    public void update() {
        double dt = pidTimer.seconds();
        pidTimer.reset();

        int    currentTicks = turretMotor.getCurrentPosition();
        double error        = angleToTicks(targetAngleDegrees) - currentTicks;

        if (Math.abs(error) < 50) {
            integralSum += error * dt;
        } else {
            integralSum = 0;
        }

        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        double power = (kP * error) + (kI * integralSum) + (kD * derivative);
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        if (Math.abs(error) > AT_TARGET_THRESHOLD) {
            double minPow = Math.abs(error) > FAR_THRESHOLD ? MIN_POWER_FAR : MIN_POWER_CLOSE;
            if (power > 0 && power < minPow)  power =  minPow;
            if (power < 0 && power > -minPow) power = -minPow;
        } else {
            power = 0;
        }

        double currentAngle = ticksToAngle(currentTicks);
        if (currentAngle >= MAX_ANGLE && power < 0) power = 0;
        if (currentAngle <= MIN_ANGLE && power > 0) power = 0;

        turretMotor.setPower(power);
    }

    public void resetPID() {
        integralSum = 0;
        lastError   = 0;
        pidTimer.reset();
    }

    public void setMotorPowerDirectly(double power) {
        double currentAngle = getCurrentAngle();
        if (currentAngle >= MAX_ANGLE && power < 0) power = 0;
        if (currentAngle <= MIN_ANGLE && power > 0) power = 0;
        turretMotor.setPower(power);
    }

    public double getMotorPower() { return turretMotor.getPower(); }

    // ── Angle control ─────────────────────────────────────────
    public void setTurretAngle(double degrees) {
        degrees = normalizeAngle(degrees);
        degrees = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, degrees));
        targetAngleDegrees = degrees;
    }

    public void setToFacingFront() { setTurretAngle(0); }

    // ── Odometry aim ─────────────────────────────────────────
    public void setGoalPosition(double x, double y) { goalX = x; goalY = y; }

    public void aimAtGoal(double robotX, double robotY, double robotHeading) {
        double angleToGoal = calculateAngleToGoal(robotX, robotY, robotHeading);
        setTurretAngle(angleToGoal);
    }

    public double calculateAngleToGoal(double robotX, double robotY, double robotHeading) {
        double headingRad   = Math.toRadians(robotHeading);
        double turretFieldX = robotX + (TURRET_OFFSET_X * Math.cos(headingRad));
        double turretFieldY = robotY + (TURRET_OFFSET_X * Math.sin(headingRad));
        double deltaX       = goalX - turretFieldX;
        double deltaY       = goalY - turretFieldY;
        double fieldAngle   = Math.toDegrees(Math.atan2(deltaY, deltaX));
        return normalizeAngle(fieldAngle - robotHeading);
    }

    public double calculateDistanceToGoal(double robotX, double robotY) {
        return Math.hypot(goalX - robotX, goalY - robotY);
    }

    public boolean isInDeadzone(double angle) {
        return angle > MAX_ANGLE - 10.0 || angle < MIN_ANGLE + 10.0;
    }

    // ── Encoder reset ─────────────────────────────────────────
    public void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetAngleDegrees = 0;
        integralSum        = 0;
        lastError          = 0;
    }

    // ── Helpers ───────────────────────────────────────────────
    private double normalizeAngle(double degrees) {
        return ((degrees % 360) + 540) % 360 - 180;
    }

    private double angleToTicks(double degrees) {
        return -(degrees / 360.0) * TICKS_PER_TURRET_REV;
    }

    private double ticksToAngle(int ticks) {
        return -(ticks / TICKS_PER_TURRET_REV) * 360.0;
    }
    public void correctEncoderFromLimelight(double trueAngleDegrees) {
        // Called when limelight confirms turret is locked on goal
        // Resets encoder to match the known true angle
        int correctTicks = (int) angleToTicks(trueAngleDegrees);
        // We can't reset to arbitrary value easily, so we just update target
        // and force the internal state to match
        targetAngleDegrees = trueAngleDegrees;
        lastError = 0;
        integralSum = 0;
    }

    // ── Getters ───────────────────────────────────────────────
    public double  getCurrentAngle() { return ticksToAngle(turretMotor.getCurrentPosition()); }
    public int     getCurrentTicks() { return turretMotor.getCurrentPosition(); }
    public double  getTargetAngle()  { return targetAngleDegrees; }
    public int     getTargetTicks()  { return (int) angleToTicks(targetAngleDegrees); }

    public boolean isAtTarget() {
        return Math.abs(angleToTicks(targetAngleDegrees) - turretMotor.getCurrentPosition())
                < AT_TARGET_THRESHOLD;
    }

    // ── Telemetry ─────────────────────────────────────────────
    public void setTelemetry(Telemetry telem) { this.telemetry = telem; }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Turret Angle", "%.1f°", getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", targetAngleDegrees);
        telemetry.addData("Ticks",        getCurrentTicks());
        telemetry.addData("At Target?",   isAtTarget() ? "YES ✓" : "NO");
        telemetry.addData("Motor Power",  "%.3f", turretMotor.getPower());
    }

    public void updateAutoAimTelemetry(double robotX, double robotY, double robotHeading) {
        if (telemetry == null) return;
        telemetry.addData("Turret Angle",     "%.1f°",   getCurrentAngle());
        telemetry.addData("Target Angle",     "%.1f°",   targetAngleDegrees);
        telemetry.addData("Angle to Goal",    "%.1f°",   calculateAngleToGoal(robotX, robotY, robotHeading));
        telemetry.addData("Distance to Goal", "%.1f in", calculateDistanceToGoal(robotX, robotY));
        telemetry.addData("In Deadzone?",     isInDeadzone(getCurrentAngle()) ? "YES" : "NO");
        telemetry.addData("At Target?",       isAtTarget() ? "YES ✓" : "NO");
        telemetry.addData("Motor Power",      "%.3f",    turretMotor.getPower());
    }
}
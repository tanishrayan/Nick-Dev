package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {

    // ── Hardware ───────────────────────────────────────────────
    private DcMotorEx turretMotor;

    // ── Motor/Encoder Constants ────────────────────────────────
    private static final double TICKS_PER_MOTOR_REV  = 28.0;
    private static final double GEAR_REDUCTION        = (50.0/20.0) * (74.0/20.0) * (120.0/25.0);
    private static final double TICKS_PER_TURRET_REV  = TICKS_PER_MOTOR_REV * GEAR_REDUCTION;

    // ── Soft limits — from physical hard stop testing ──────────
    private static final double MAX_ANGLE          =  172.0;  // right hard stop with buffer
    private static final double MIN_ANGLE          =  -85.0;  // left hard stop with buffer
    private static final double DEADZONE_THRESHOLD =   10.0;

    // ── PID Coefficients ───────────────────────────────────────
    private static final double kP        = 0.029;
    private static final double kI        = 0.0;
    private static final double kD        = 0.0;
    private static final double MAX_POWER = 1.0;

    private static final int AT_TARGET_THRESHOLD = 5;

    // ── Turret offset from robot center (inches) ───────────────
    private static final double TURRET_OFFSET_X = 3.0;
    private static final double TURRET_OFFSET_Y = 0.0;

    // ── Goal position on field (inches, 0,0 = field center) ───
    private double goalX = 60.0;
    private double goalY = 60.0;

    // ── PID state ─────────────────────────────────────────────
    private double      targetAngleDegrees = 0.0;
    private double      lastError          = 0.0;
    private double      integralSum        = 0.0;
    private ElapsedTime pidTimer           = new ElapsedTime();

    // ── Limelight correction ───────────────────────────────────
    private double limelightCorrectionDeg = 0.0;

    private Telemetry telemetry;

    // ── Constructor ────────────────────────────────────────────
    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidTimer.reset();
    }

    // ── PID update — MUST be called every loop when using odometry aim ──
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
        if (Math.abs(error) < AT_TARGET_THRESHOLD) power = 0;

        // Hard stop enforcement
        double currentAngle = ticksToAngle(currentTicks);
        if (currentAngle >= MAX_ANGLE && power > 0) power = 0;
        if (currentAngle <= MIN_ANGLE && power < 0) power = 0;

        turretMotor.setPower(power);
    }

    // ── Reset PID state — call when switching from odometry to limelight ──
    public void resetPID() {
        integralSum = 0;
        lastError   = 0;
        pidTimer.reset();
    }

    /**
     * Bypasses PID — used by limelight direct control and heading compensation.
     * Still enforces hard stop limits.
     */
    public void setMotorPowerDirectly(double power) {
        double currentAngle = getCurrentAngle();
        if (currentAngle >= MAX_ANGLE && power > 0) power = 0;
        if (currentAngle <= MIN_ANGLE && power < 0) power = 0;
        turretMotor.setPower(power);
    }

    public double getMotorPower() { return turretMotor.getPower(); }

    // ── Angle control ──────────────────────────────────────────

    public void setTurretAngle(double angleRelativeToRobot) {
        angleRelativeToRobot = normalizeAngle(angleRelativeToRobot);
        angleRelativeToRobot = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleRelativeToRobot));
        targetAngleDegrees   = angleRelativeToRobot;
    }

    public void setToFacingFront() { setTurretAngle(0); }

    // ── Field aim ─────────────────────────────────────────────

    public void aimAtGoal(double robotX, double robotY, double robotHeading) {
        double angleToGoal     = calculateAngleToGoal(robotX, robotY, robotHeading);
        double correctedAngle  = normalizeAngle(angleToGoal + limelightCorrectionDeg);
        if (isInDeadzone(correctedAngle)) return;
        setTurretAngle(correctedAngle);
    }

    // ── Limelight correction hook ──────────────────────────────
    public void applyLimelightCorrection(double correctionDegrees) {
        limelightCorrectionDeg = correctionDegrees;
    }

    public void clearLimelightCorrection() { limelightCorrectionDeg = 0.0; }

    // ── Goal math ─────────────────────────────────────────────

    public double calculateAngleToGoal(double robotX, double robotY, double robotHeading) {
        double headingRad   = Math.toRadians(robotHeading);
        double turretFieldX = robotX + (TURRET_OFFSET_X * Math.cos(headingRad))
                - (TURRET_OFFSET_Y * Math.sin(headingRad));
        double turretFieldY = robotY + (TURRET_OFFSET_X * Math.sin(headingRad))
                + (TURRET_OFFSET_Y * Math.cos(headingRad));

        double deltaX           = goalX - turretFieldX;
        double deltaY           = goalY - turretFieldY;
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return normalizeAngle(fieldAngleToGoal - robotHeading);
    }

    public double calculateDistanceToGoal(double robotX, double robotY) {
        return Math.hypot(goalX - robotX, goalY - robotY);
    }

    public void setGoalPosition(double x, double y) {
        this.goalX = x;
        this.goalY = y;
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

    public boolean isInDeadzone(double angle) {
        return angle > (MAX_ANGLE - DEADZONE_THRESHOLD)
                || angle < (MIN_ANGLE + DEADZONE_THRESHOLD);
    }

    private double angleToTicks(double degrees) {
        return (degrees / 360.0) * TICKS_PER_TURRET_REV;
    }

    private double ticksToAngle(int ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 360.0;
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
        telemetry.addData("Turret Angle",     "%.1f°",  getCurrentAngle());
        telemetry.addData("Target Angle",     "%.1f°",  targetAngleDegrees);
        telemetry.addData("Angle to Goal",    "%.1f°",  calculateAngleToGoal(robotX, robotY, robotHeading));
        telemetry.addData("Limelight Offset", "%.1f°",  limelightCorrectionDeg);
        telemetry.addData("Distance to Goal", "%.1f in", calculateDistanceToGoal(robotX, robotY));
        telemetry.addData("In Deadzone?",     isInDeadzone(getCurrentAngle()) ? "YES" : "NO");
        telemetry.addData("At Target?",       isAtTarget() ? "YES ✓" : "NO");
    }
}
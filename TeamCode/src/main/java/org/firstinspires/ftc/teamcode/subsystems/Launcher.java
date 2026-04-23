package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {

    // ── Hardware ───────────────────────────────────────────────
    private DcMotorEx topMotor, bottomMotor;
    private Servo     adjustableHood;
    private Servo     latch;

    // ── Motor direction multipliers ────────────────────────────
    private static final double TOP_MULTIPLIER    =  1.0;
    private static final double BOTTOM_MULTIPLIER = -1.0;

    // ── Flywheel PIDF (tune on new robot) ─────────────────────
    private static final double kP = 160.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 15.8;

    // ── Hood positions (tune on new robot) ────────────────────
    private static final double LATCH_CLOSED   = 0.919;
    private static final double LATCH_OPEN     = 0.6;
    private static final double HOOD_RETRACTED = 1.0;
    private static final double HOOD_EXTENDED  = 0.32;

    // ── Flywheel ready threshold ───────────────────────────────
    private static final double FLYWHEEL_READY_THRESHOLD = 50.0; // RPM tolerance
    private double targetVelocity = 0.0;

    private Telemetry telemetry;

    // ── Constructor ────────────────────────────────────────────
    public Launcher(HardwareMap hardwareMap) {
        topMotor    = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        adjustableHood = hardwareMap.get(Servo.class, "hood");
        latch          = hardwareMap.get(Servo.class, "latch");

        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Uncomment after tuning PIDF on new robot:
        // PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        // topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        // bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        closeLatch();
    }

    // ── Flywheel ──────────────────────────────────────────────

    public void setFlywheelVelocity(double velocity) {
        targetVelocity = velocity;
        topMotor.setVelocity(velocity * TOP_MULTIPLIER);
        bottomMotor.setVelocity(velocity * BOTTOM_MULTIPLIER);
    }

    public void stopFlywheel() {
        targetVelocity = 0.0;
        topMotor.setPower(0);
        bottomMotor.setPower(0);
    }

    public double getFlywheelVelocity() {
        double top    = topMotor.getVelocity()    * TOP_MULTIPLIER;
        double bottom = bottomMotor.getVelocity() * BOTTOM_MULTIPLIER;
        return (top + bottom) / 2.0;
    }

    /**
     * Returns true when flywheel is within threshold of target velocity.
     * Gate the latch open on this in the shooting sequence.
     */
    public boolean isFlywheelReady() {
        if (targetVelocity == 0) return false;
        return Math.abs(getFlywheelVelocity() - targetVelocity) < FLYWHEEL_READY_THRESHOLD;
    }

    // ── Hood ──────────────────────────────────────────────────

    public void setHoodPosition(double position) {
        adjustableHood.setPosition(position);
    }

    public double getHoodPosition() {
        return adjustableHood.getPosition();
    }

    public void setHoodRetracted() { adjustableHood.setPosition(HOOD_RETRACTED); }
    public void setHoodExtended()  { adjustableHood.setPosition(HOOD_EXTENDED); }

    // ── Latch ─────────────────────────────────────────────────

    public void openLatch()  { latch.setPosition(LATCH_OPEN); }
    public void closeLatch() { latch.setPosition(LATCH_CLOSED); }

    public double getLatchPosition() { return latch.getPosition(); }

    // ── Distance-based calculations ───────────────────────────

    public double calculateFlywheelVelocity(double distanceToGoal) {
        // Quadratic regression — R² = 0.991
        double d = distanceToGoal - 20;
        return 0.05 * d * d + (-0.3669) * d + 1450.0528;
    }

    public double calculateHoodAngle(double distanceToGoal) {
        // Piecewise linear — R² = 0.970
        if (distanceToGoal <= 67) {
            return 1.0;
        } else if (distanceToGoal > 80 && distanceToGoal <= 115) {
            return 0.65;
        } else {
            return -0.01119 * (distanceToGoal - 20) + 1.65;
        }
    }

    // ── Telemetry ─────────────────────────────────────────────

    public void setTelemetry(Telemetry telem) { this.telemetry = telem; }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Flywheel Velocity", "%.0f RPM", getFlywheelVelocity());
        telemetry.addData("Target Velocity",   "%.0f RPM", targetVelocity);
        telemetry.addData("Flywheel Ready?",   isFlywheelReady() ? "YES ✓" : "NO");
        telemetry.addData("Hood Position",     "%.3f", adjustableHood.getPosition());
        telemetry.addData("Latch Position",    "%.3f", latch.getPosition());
    }
}
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private static final double kP = 150.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 13.2;

    // ── Hood positions (tune on new robot) ────────────────────
    private static final double LATCH_CLOSED   = 0.843;
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
        PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

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
        position = Math.max(0.269, Math.min(0.89, position));
        adjustableHood.setPosition(position);
    }

    public double getHoodPosition() {
        return adjustableHood.getPosition();
    }

    public void setHoodRetracted() { adjustableHood.setPosition(0.269); }
    public void setHoodExtended()  { adjustableHood.setPosition(0.89); }

    // ── Latch ─────────────────────────────────────────────────

    public void openLatch()  { latch.setPosition(LATCH_OPEN); }
    public void closeLatch() { latch.setPosition(LATCH_CLOSED); }

    public double getLatchPosition() { return latch.getPosition(); }

    // ── Distance-based calculations ───────────────────────────

    public double calculateFlywheelVelocity(double distanceToGoal) {
        double d = distanceToGoal;
        return 0.1028 * d * d - 10.40 * d + 1691.3;
    }

    public double calculateHoodAngle(double distanceToGoal) {
        double position;
        if (distanceToGoal <= 85) {
            position = 0.669;
        } else if (distanceToGoal <= 102) {
            double t = (distanceToGoal - 85) / (102 - 85);
            position = 0.669 + t * (0.569 - 0.669);
        } else if (distanceToGoal <= 119) {
            double t = (distanceToGoal - 102) / (119 - 102);
            position = 0.569 + t * (0.32 - 0.569);
        } else if (distanceToGoal <= 140) {
            double t = (distanceToGoal - 119) / (140 - 119);
            position = 0.32 + t * (0.280 - 0.32);
        } else {
            position = 0.280;
        }
        return Math.max(0.269, Math.min(0.89, position));
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
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {

    // ── Hardware ───────────────────────────────────────────────
    private DcMotorEx topMotor, bottomMotor;
    private Servo     adjustableHood;
    private Servo     latch;

    // ── Latch positions ────────────────────────────────────────
    private static final double LATCH_CLOSED = 0.93;
    private static final double LATCH_OPEN   = 0.6;

    // ── Hood limits ────────────────────────────────────────────
    private static final double HOOD_MIN = 0.269; // fully up
    private static final double HOOD_MAX = 0.89;  // fully down

    // ── Bang-bang ──────────────────────────────────────────────
    private static final double BANG_BANG_DEADBAND = 0.0;

    // ── Flywheel ready threshold ───────────────────────────────
    private static final double FLYWHEEL_READY_THRESHOLD = 50.0;

    // ── Burst offset ───────────────────────────────────────────
    // 50 ticks for close (≤120in), 200 ticks for far (≥150in), interpolated.
    private static final double BURST_OFFSET_CLOSE    = 0.0;
    private static final double BURST_OFFSET_FAR      = 50.0;
    private static final double BURST_CLOSE_THRESHOLD = 120.0;
    private static final double BURST_FAR_THRESHOLD   = 150.0;

    // ── Empirical hood compensation ────────────────────────────
    // During shoot sequence, nudges hood down proportional to speed error.
    // Positive correction = higher servo value = lower hood = flatter shot
    // = compensates for ball falling short at reduced speed.
    // Raise if ball 3 still short, lower if overcompensating.
    private static final double HOOD_COMPENSATION_SCALAR = 0.0006;

    // ── State ──────────────────────────────────────────────────
    private double  targetVelocity     = 0.0;
    private double  effectiveVelocity  = 0.0;
    private boolean burstModeActive    = false;
    private double  baseHoodPosition   = 0.6;
    private double  lastHoodCorrection = 0.0;

    private Telemetry telemetry;

    // ── Constructor ────────────────────────────────────────────
    public Launcher(HardwareMap hardwareMap) {
        topMotor       = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor    = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        adjustableHood = hardwareMap.get(Servo.class, "hood");
        latch          = hardwareMap.get(Servo.class, "latch");

        // Both motors FORWARD direction. Bottom receives setPower(-1) to spin
        // opposite to top, matching ShooterIntakeTest which uses
        // setVelocity(-targetVelocity) for bottom. setDirection(REVERSE) is
        // NOT used because in RUN_WITHOUT_ENCODER it causes erratic behavior.
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // RUN_WITHOUT_ENCODER: setPower() = direct voltage, no internal PIDF.
        // getVelocity() still works for bang-bang feedback.
        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        closeLatch();
    }

    // ── Main update (call every loop) ─────────────────────────
    /**
     * Must be called every loop in BlueSideTele and all autos.
     * Runs bang-bang against effectiveVelocity.
     * During burst also applies empirical hood compensation:
     * nudges hood down proportional to speed error so ball still reaches
     * target at reduced flywheel speed between shots.
     */
    public void update(double distanceToGoal) {
        effectiveVelocity = targetVelocity
                + (burstModeActive ? getBurstOffset(distanceToGoal) : 0.0);

        if (targetVelocity <= 0) {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
            lastHoodCorrection = 0.0;
            return;
        }

        // ── Bang-bang ──────────────────────────────────────────
        double actual = getFlywheelVelocity();
        if (actual < effectiveVelocity - BANG_BANG_DEADBAND) {
            topMotor.setPower(1);
            bottomMotor.setPower(-1); // negative — spins opposite to top
        } else {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
        }

        // ── Empirical hood compensation ────────────────────────
        if (burstModeActive) {
            double speedError      = effectiveVelocity - actual;
            lastHoodCorrection     = speedError * HOOD_COMPENSATION_SCALAR;
            double compensatedHood = baseHoodPosition + lastHoodCorrection;
            adjustableHood.setPosition(
                    Math.max(HOOD_MIN, Math.min(HOOD_MAX, compensatedHood)));
        } else {
            lastHoodCorrection = 0.0;
            adjustableHood.setPosition(
                    Math.max(HOOD_MIN, Math.min(HOOD_MAX, baseHoodPosition)));
        }
    }

    // ── Flywheel control ──────────────────────────────────────

    public void setFlywheelVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public void stopFlywheel() {
        targetVelocity     = 0.0;
        effectiveVelocity  = 0.0;
        burstModeActive    = false;
        lastHoodCorrection = 0.0;
        topMotor.setPower(0);
        bottomMotor.setPower(0);
    }

    /**
     * Enable at start of shoot sequence, disable when complete or aborted.
     * Resets hood to base position on disable.
     */
    public void setBurstMode(boolean active) {
        burstModeActive = active;
        if (!active) {
            lastHoodCorrection = 0.0;
            adjustableHood.setPosition(
                    Math.max(HOOD_MIN, Math.min(HOOD_MAX, baseHoodPosition)));
        }
    }

    /**
     * Returns average of both motors, normalized so both read positive
     * when spinning correctly. Top reads positive naturally. Bottom reads
     * negative (spinning opposite direction) so we negate it.
     */
    public double getFlywheelVelocity() {
        return (topMotor.getVelocity() + (-bottomMotor.getVelocity())) / 2.0;
    }

    /**
     * True when flywheel is within threshold of effectiveVelocity.
     * Guards against false-ready on first loop after RB press:
     *   - targetVelocity must be > 0 (flywheel actually on)
     *   - effectiveVelocity must be >= 100 (update() has run with burst active)
     */
    public boolean isFlywheelReady() {
        if (targetVelocity <= 0)     return false;
        if (effectiveVelocity < 100) return false;
        return Math.abs(getFlywheelVelocity() - effectiveVelocity) < FLYWHEEL_READY_THRESHOLD;
    }

    // ── Hood control ──────────────────────────────────────────

    /**
     * Called when distance changes. Stores base position so update()
     * can apply compensation relative to it during burst.
     */
    public void setHoodPosition(double position) {
        baseHoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, position));
        if (!burstModeActive) {
            adjustableHood.setPosition(baseHoodPosition);
        }
    }

    public double getHoodPosition()  { return adjustableHood.getPosition(); }
    public void   setHoodRetracted() { setHoodPosition(HOOD_MIN); }
    public void   setHoodExtended()  { setHoodPosition(HOOD_MAX); }

    // ── Latch ─────────────────────────────────────────────────
    public void   openLatch()        { latch.setPosition(LATCH_OPEN);   }
    public void   closeLatch()       { latch.setPosition(LATCH_CLOSED); }
    public double getLatchPosition() { return latch.getPosition();      }

    // ── Distance-based calculations ───────────────────────────

    /**
     * Velocity model:
     *   ≤120in   → original quadratic (close side, tuned and perfect)
     *   120-150in → linear ramp from quadratic at 120 up to 2200
     *   ≥150in   → flat 2200 (confirmed hitting from far side)
     */
    public double calculateFlywheelVelocity(double d) {
        if (d <= 120) {
            return 0.116 * d * d - 13.35 * d + 1780.0;
        } else if (d <= 150) {
            double velAt120 = 0.116 * 120 * 120 - 13.35 * 120 + 1830.0;
            double t = (d - 120.0) / (150.0 - 120.0);
            return velAt120 + t * (2200.0 - velAt120);
        } else {
            return 2200.0;
        }
    }

    /**
     * Hood angle model — unchanged from original.
     * Close side tuned and perfect. Far side at 0.280 confirmed at 150in.
     */
    public double calculateHoodAngle(double d) {
        double position;
        if (d <= 102) {
            position = 0.69;
        } else if (d <= 119) {
            double t = (d - 102) / (119 - 102);
            position = 0.73 + t * (0.32 - 0.669);
        } else if (d <= 140) {
            double t = (d - 119) / (140 - 119);
            position = 0.32 + t * (0.280 - 0.32);
        } else {
            position = 0.280;
        }
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, position));
    }

    // ── Burst offset interpolation ─────────────────────────────
    private double getBurstOffset(double d) {
        if (d <= BURST_CLOSE_THRESHOLD) return BURST_OFFSET_CLOSE;
        if (d >= BURST_FAR_THRESHOLD)   return BURST_OFFSET_FAR;
        double t = (d - BURST_CLOSE_THRESHOLD)
                / (BURST_FAR_THRESHOLD - BURST_CLOSE_THRESHOLD);
        return BURST_OFFSET_CLOSE + t * (BURST_OFFSET_FAR - BURST_OFFSET_CLOSE);
    }

    // ── Telemetry ─────────────────────────────────────────────
    public void setTelemetry(Telemetry telem) { this.telemetry = telem; }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Flywheel Velocity",  "%.0f ticks/s", getFlywheelVelocity());
        telemetry.addData("Top Motor",          "%.0f ticks/s", topMotor.getVelocity());
        telemetry.addData("Bottom Motor",       "%.0f ticks/s", -bottomMotor.getVelocity());
        telemetry.addData("Base Target",        "%.0f ticks/s", targetVelocity);
        telemetry.addData("Effective Target",   "%.0f ticks/s%s", effectiveVelocity,
                burstModeActive ? "  (BURST)" : "");
        telemetry.addData("Flywheel Ready?",    isFlywheelReady() ? "YES ✓" : "NO");
        telemetry.addData("Burst Mode",         burstModeActive ? "ON" : "OFF");
        telemetry.addData("Hood Base",          "%.3f", baseHoodPosition);
        telemetry.addData("Hood Correction",    "%.4f", lastHoodCorrection);
        telemetry.addData("Hood Actual",        "%.3f", adjustableHood.getPosition());
        telemetry.addData("Latch Position",     "%.3f", latch.getPosition());
    }
}
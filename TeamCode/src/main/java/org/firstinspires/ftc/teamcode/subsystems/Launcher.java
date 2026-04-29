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
    private static final double LATCH_CLOSED = 0.843;
    private static final double LATCH_OPEN   = 0.6;

    // ── Bang-bang ──────────────────────────────────────────────
    // Deadband 0 = aggressive: full power the instant speed drops below target,
    // cuts power the instant it hits target. Raise only if motor flickers badly.
    private static final double BANG_BANG_DEADBAND = 0.0;

    // ── Flywheel ready threshold ───────────────────────────────
    // How close actual velocity must be to effectiveVelocity before latch opens.
    private static final double FLYWHEEL_READY_THRESHOLD = 50.0;

    // ── Burst offset ───────────────────────────────────────────
    // Pre-spins flywheel above base target before burst so ball 3 still fires
    // above minimum threshold after compounding speed drops per ball.
    // 50 ticks for close (≤120in), 150 ticks for far (≥150in), interpolated between.
    private static final double BURST_OFFSET_CLOSE    = 50.0;
    private static final double BURST_OFFSET_FAR      = 150.0;
    private static final double BURST_CLOSE_THRESHOLD = 120.0;
    private static final double BURST_FAR_THRESHOLD   = 150.0;

    // ── State ──────────────────────────────────────────────────
    private double  targetVelocity    = 0.0;
    private double  effectiveVelocity = 0.0;
    private boolean burstModeActive   = false;

    private Telemetry telemetry;

    // ── Constructor ────────────────────────────────────────────
    public Launcher(HardwareMap hardwareMap) {
        topMotor       = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor    = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        adjustableHood = hardwareMap.get(Servo.class, "hood");
        latch          = hardwareMap.get(Servo.class, "latch");

        // Top FORWARD, bottom REVERSE: both setPower(1) spins them opposite
        // physically so both wheels grip the ball from each side.
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // RUN_WITHOUT_ENCODER: setPower() = direct voltage.
        // getVelocity() still works for bang-bang feedback.
        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        closeLatch();
    }

    // ── Bang-bang update (call every loop) ────────────────────
    /**
     * Must be called every loop in BlueSideTele.
     * Computes effectiveVelocity = targetVelocity + burstOffset when active,
     * then runs bang-bang against it. If targetVelocity is 0, motors coast.
     */
    public void update(double distanceToGoal) {
        effectiveVelocity = targetVelocity
                + (burstModeActive ? getBurstOffset(distanceToGoal) : 0.0);

        if (targetVelocity <= 0) {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
            return;
        }

        double actual = getFlywheelVelocity();
        if (actual < effectiveVelocity - BANG_BANG_DEADBAND) {
            topMotor.setPower(1);
            bottomMotor.setPower(1);
        } else {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
        }
    }

    // ── Flywheel control ──────────────────────────────────────

    /**
     * Sets base target velocity. update() handles actual bang-bang control.
     * Call when distance changes or flywheel is toggled on.
     */
    public void setFlywheelVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public void stopFlywheel() {
        targetVelocity    = 0.0;
        effectiveVelocity = 0.0;
        burstModeActive   = false;
        topMotor.setPower(0);
        bottomMotor.setPower(0);
    }

    /**
     * Enable at the start of a shoot sequence, disable when complete or aborted.
     * update() automatically applies the correct burst offset for current distance.
     */
    public void setBurstMode(boolean active) {
        burstModeActive = active;
    }

    public double getFlywheelVelocity() {
        // Both readings positive after setDirection — average for stability.
        return (topMotor.getVelocity() + bottomMotor.getVelocity()) / 2.0;
    }

    /**
     * True when flywheel is within threshold of effectiveVelocity.
     * Guards against false-ready on the first loop after RB press:
     *   - targetVelocity must be set (flywheel actually on)
     *   - effectiveVelocity must be meaningful (update() has run at least once
     *     after burst mode was enabled, so effectiveVelocity is not stale zero)
     */
    public boolean isFlywheelReady() {
        if (targetVelocity <= 0) return false;
        if (effectiveVelocity < 100) return false; // catches stale zero on first loop
        return Math.abs(getFlywheelVelocity() - effectiveVelocity) < FLYWHEEL_READY_THRESHOLD;
    }

    // ── Burst offset interpolation ─────────────────────────────
    private double getBurstOffset(double d) {
        if (d <= BURST_CLOSE_THRESHOLD) return BURST_OFFSET_CLOSE;
        if (d >= BURST_FAR_THRESHOLD)   return BURST_OFFSET_FAR;
        double t = (d - BURST_CLOSE_THRESHOLD)
                / (BURST_FAR_THRESHOLD - BURST_CLOSE_THRESHOLD);
        return BURST_OFFSET_CLOSE + t * (BURST_OFFSET_FAR - BURST_OFFSET_CLOSE);
    }

    // ── Hood ──────────────────────────────────────────────────
    public void setHoodPosition(double position) {
        position = Math.max(0.269, Math.min(0.89, position));
        adjustableHood.setPosition(position);
    }

    public double getHoodPosition()  { return adjustableHood.getPosition(); }
    public void   setHoodRetracted() { adjustableHood.setPosition(0.269);  }
    public void   setHoodExtended()  { adjustableHood.setPosition(0.89);   }

    // ── Latch ─────────────────────────────────────────────────
    public void   openLatch()         { latch.setPosition(LATCH_OPEN);   }
    public void   closeLatch()        { latch.setPosition(LATCH_CLOSED); }
    public double getLatchPosition()  { return latch.getPosition();      }

    // ── Distance-based calculations ───────────────────────────

    /**
     * Velocity model:
     *   ≤120in   → original quadratic (close side, already tuned and perfect)
     *   120-150in → linear ramp from quadratic value at 120 up to 2200
     *   ≥150in   → flat 2200 (confirmed hitting at this speed from far side)
     */
    public double calculateFlywheelVelocity(double d) {
        if (d <= 120) {
            return 0.116 * d * d - 13.35 * d + 1848.0;
        } else if (d <= 150) {
            double velAt120 = 0.116 * 120 * 120 - 13.35 * 120 + 1848.0; // ~1916 ticks/s
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
            position = 0.669;
        } else if (d <= 119) {
            double t = (d - 102) / (119 - 102);
            position = 0.669 + t * (0.32 - 0.669);
        } else if (d <= 140) {
            double t = (d - 119) / (140 - 119);
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
        telemetry.addData("Flywheel Velocity",  "%.0f ticks/s", getFlywheelVelocity());
        telemetry.addData("Base Target",        "%.0f ticks/s", targetVelocity);
        telemetry.addData("Effective Target",   "%.0f ticks/s%s", effectiveVelocity,
                burstModeActive ? "  (BURST)" : "");
        telemetry.addData("Flywheel Ready?",    isFlywheelReady() ? "YES ✓" : "NO");
        telemetry.addData("Burst Mode",         burstModeActive ? "ON" : "OFF");
        telemetry.addData("Hood Position",      "%.3f", adjustableHood.getPosition());
        telemetry.addData("Latch Position",     "%.3f", latch.getPosition());
    }
}
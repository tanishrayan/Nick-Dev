package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TurretPIDFTuner", group = "testing")
public class TurretPIDFTuner extends LinearOpMode {

    // ── Constants ──────────────────────────────────────────────
    static final double TICKS_PER_MOTOR_REV  = 28.0;
    static final double GEAR_REDUCTION       = (50.0/20.0) * (74.0/20.0) * (120.0/25.0);
    static final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * GEAR_REDUCTION;

    static final int POS_CENTER = 0;
    static final int POS_LEFT   = (int)( TICKS_PER_TURRET_REV * 0.25);  // +90°
    static final int POS_RIGHT  = (int)(-TICKS_PER_TURRET_REV * 0.25);  // -90°

    // ── Starting PID values — tune from here ───────────────────
    double kP = 0.01;   // start low, increase until it moves to target
    double kD = 0.001;  // dampens overshoot, increase to remove creak
    double kI = 0.0;    // leave at 0 unless steady-state error persists

    // Adjustment steps
    final double P_STEP = 0.001;
    final double D_STEP = 0.0005;

    // Max power clamp — raise this for more speed
    final double MAX_POWER = 1.0;

    // ── Hardware ───────────────────────────────────────────────
    DcMotorEx turretMotor;

    @Override
    public void runOpMode() {

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("TurretPIDFTuner ready");
        telemetry.addLine("──────────────────────────────");
        telemetry.addLine("GP1 A             → CENTER (0°)");
        telemetry.addLine("GP1 X             → LEFT  (+90°)");
        telemetry.addLine("GP1 B             → RIGHT (-90°)");
        telemetry.addLine("GP1 Y             → re-zero encoder");
        telemetry.addLine("DPad Up/Down      → P ± 0.001");
        telemetry.addLine("DPad Right/Left   → D ± 0.0005");
        telemetry.update();

        waitForStart();

        int    targetPos    = POS_CENTER;
        double lastError    = 0;
        double integralSum  = 0;

        ElapsedTime loopTimer   = new ElapsedTime();
        ElapsedTime buttonTimer = new ElapsedTime();

        while (opModeIsActive()) {

            double dt = loopTimer.seconds();
            loopTimer.reset();

            // ── Position targets ────────────────────────────────
            if (gamepad1.a) {
                targetPos = POS_CENTER;
            } else if (gamepad1.x) {
                targetPos = POS_LEFT;
            } else if (gamepad1.b) {
                targetPos = POS_RIGHT;
            }

            // ── Re-zero ─────────────────────────────────────────
            if (gamepad1.y) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetPos    = 0;
                integralSum  = 0;
                lastError    = 0;
            }

            // ── PIDF adjustment (debounced 150ms) ───────────────
            if (buttonTimer.milliseconds() > 150) {
                if (gamepad1.dpad_up) {
                    kP += P_STEP; buttonTimer.reset();
                } else if (gamepad1.dpad_down) {
                    kP = Math.max(0, kP - P_STEP); buttonTimer.reset();
                } else if (gamepad1.dpad_right) {
                    kD += D_STEP; buttonTimer.reset();
                } else if (gamepad1.dpad_left) {
                    kD = Math.max(0, kD - D_STEP); buttonTimer.reset();
                }
            }

            // ── Manual PID ──────────────────────────────────────
            int    currentPos = turretMotor.getCurrentPosition();
            double error      = targetPos - currentPos;

            // Integral — only accumulate when close to target to avoid windup
            if (Math.abs(error) < 50) {
                integralSum += error * dt;
            } else {
                integralSum = 0;
            }

            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            lastError = error;

            double power = (kP * error) + (kI * integralSum) + (kD * derivative);
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));  // clamp

            // Stop buzzing at target
            if (Math.abs(error) < 5) power = 0;

            turretMotor.setPower(power);

            // ── Telemetry ───────────────────────────────────────
            double degrees   = (currentPos / TICKS_PER_TURRET_REV) * 360.0;
            double targetDeg = (targetPos  / TICKS_PER_TURRET_REV) * 360.0;

            telemetry.addLine("── PID Values ────────────────");
            telemetry.addData("  kP", "%.4f  (DPad Up/Down)",  kP);
            telemetry.addData("  kI", "%.4f",                  kI);
            telemetry.addData("  kD", "%.4f  (DPad L/R)",      kD);
            telemetry.addLine("── Position ──────────────────");
            telemetry.addData("  Target",   "%.1f°  (%d ticks)", targetDeg, targetPos);
            telemetry.addData("  Current",  "%.1f°  (%d ticks)", degrees,   currentPos);
            telemetry.addData("  Error",    "%.0f ticks",         error);
            telemetry.addData("  At Target?", Math.abs(error) < 10 ? "YES ✓" : "NO");
            telemetry.addLine("── Motor ─────────────────────");
            telemetry.addData("  Power",     "%.3f", power);
            telemetry.addData("  Integral",  "%.3f", integralSum);
            telemetry.update();
        }

        turretMotor.setPower(0);
    }
}
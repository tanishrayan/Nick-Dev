package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * TURRET ENCODER FINDER
 *
 * Lets you manually drive the turret and read encoder ticks + computed
 * angle at any position. Use this to verify the gear ratio is correct
 * and to find physical hard-stop tick values.
 *
 * GAMEPAD 1:
 *   Left stick X    — direct power control of turret motor
 *   A               — reset encoder to 0 at current physical position
 *   Y               — print current ticks and angle to telemetry log
 *
 * Watch telemetry to record:
 *   - Ticks at left hard stop  → set as MIN_TICKS in Turret.java
 *   - Ticks at right hard stop → set as MAX_TICKS in Turret.java
 *   - Confirm angle math: 1 full revolution should read 360°
 */
@TeleOp(name = "TEST: Turret Encoder", group = "testing")
public class TurretEncoderFinder extends OpMode {

    private DcMotorEx turretMotor;

    // Must match your gear reduction in Turret.java
    private static final double TICKS_PER_MOTOR_REV  = 28.0;
    private static final double GEAR_REDUCTION        = (50.0 / 20.0) * (74.0 / 20.0) * (120.0 / 25.0);
    private static final double TICKS_PER_TURRET_REV  = TICKS_PER_MOTOR_REV * GEAR_REDUCTION;

    private static final double MANUAL_POWER_SCALE = 0.3; // slow for safety

    private boolean aWasPressed = false;

    // Recorded positions for reference
    private int leftHardStopTicks  = 0;
    private int rightHardStopTicks = 0;
    private boolean leftRecorded   = false;
    private boolean rightRecorded  = false;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Turret Encoder Finder ready.");
        telemetry.addLine("Left stick X = drive turret");
        telemetry.addLine("A = zero encoder at current position");
        telemetry.addLine("Left bumper = record left hard stop");
        telemetry.addLine("Right bumper = record right hard stop");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ── Manual drive ───────────────────────────────────────
        double stick = gamepad1.left_stick_x;
        if (Math.abs(stick) < 0.05) stick = 0; // deadband
        turretMotor.setPower(stick * MANUAL_POWER_SCALE);

        // ── Zero encoder ───────────────────────────────────────
        boolean aNow = gamepad1.a;
        if (aNow && !aWasPressed) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        aWasPressed = aNow;

        // ── Record hard stops ──────────────────────────────────
        if (gamepad1.left_bumper) {
            leftHardStopTicks = turretMotor.getCurrentPosition();
            leftRecorded = true;
        }
        if (gamepad1.right_bumper) {
            rightHardStopTicks = turretMotor.getCurrentPosition();
            rightRecorded = true;
        }

        // ── Computed values ────────────────────────────────────
        int    ticks        = turretMotor.getCurrentPosition();
        double angleDeg     = (ticks / TICKS_PER_TURRET_REV) * 360.0;
        double ticksPerDeg  = TICKS_PER_TURRET_REV / 360.0;

        // ── Telemetry ──────────────────────────────────────────
        telemetry.addLine("=== LIVE ===");
        telemetry.addData("Ticks",          ticks);
        telemetry.addData("Angle",          "%.2f°", angleDeg);
        telemetry.addData("Motor Power",    "%.2f", turretMotor.getPower());
        telemetry.addLine();

        telemetry.addLine("=== CONSTANTS CHECK ===");
        telemetry.addData("Gear Reduction",       "%.4f : 1", GEAR_REDUCTION);
        telemetry.addData("Ticks/Revolution",     "%.2f", TICKS_PER_TURRET_REV);
        telemetry.addData("Ticks/Degree",         "%.3f", ticksPerDeg);
        telemetry.addLine();

        telemetry.addLine("=== RECORDED HARD STOPS ===");
        if (leftRecorded) {
            telemetry.addData("Left stop (ticks)",  leftHardStopTicks);
            telemetry.addData("Left stop (angle)",  "%.1f°",
                    (leftHardStopTicks / TICKS_PER_TURRET_REV) * 360.0);
        } else {
            telemetry.addLine("Left stop: not recorded (LB to record)");
        }
        if (rightRecorded) {
            telemetry.addData("Right stop (ticks)", rightHardStopTicks);
            telemetry.addData("Right stop (angle)", "%.1f°",
                    (rightHardStopTicks / TICKS_PER_TURRET_REV) * 360.0);
        } else {
            telemetry.addLine("Right stop: not recorded (RB to record)");
        }

        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left stick X = drive  |  A = zero encoder");
        telemetry.addLine("LB = record left stop  |  RB = record right stop");
        telemetry.update();
    }

    @Override
    public void stop() {
        turretMotor.setPower(0);
    }
}
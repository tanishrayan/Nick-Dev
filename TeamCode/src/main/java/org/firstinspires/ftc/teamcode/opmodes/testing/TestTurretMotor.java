package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TestTurretMotor", group = "testing")
public class TestTurretMotor extends LinearOpMode {

    // ── Constants ──────────────────────────────────────────────
    static final double TICKS_PER_MOTOR_REV = 28.0;   // 28 PPR × 4 quadrature
    static final double GEAR_REDUCTION      = (50.0/20.0) * (74.0/20.0) * (120.0/25.0); // 44.4 : 1
    static final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * GEAR_REDUCTION;    // ~4973

    // Position targets (ticks) — adjust to match your physical turret limits
    static final int POS_CENTER =  0;
    static final int POS_LEFT   = (int)( TICKS_PER_TURRET_REV * 0.25);   // +90°
    static final int POS_RIGHT  = (int)(-TICKS_PER_TURRET_REV * 0.25);   // -90°

    // ── Hardware ───────────────────────────────────────────────
    DcMotorEx turretMotor;

    @Override
    public void runOpMode() {

        // ── Init ────────────────────────────────────────────────
        // TODO: replace "turretMotor" with your hardwareMap name
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD); // flip to REVERSE if needed
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("TestTurretMotor ready");
        telemetry.addLine("──────────────────────────────");
        telemetry.addLine("GP1 Left Stick Y  → raw power");
        telemetry.addLine("GP1 A             → go to CENTER (0°)");
        telemetry.addLine("GP1 X             → go to LEFT  (+90°)");
        telemetry.addLine("GP1 B             → go to RIGHT (-90°)");
        telemetry.addLine("GP1 Y             → STOP / RUN_WITHOUT_ENCODER");
        telemetry.update();

        waitForStart();

        // ── Mode tracking ───────────────────────────────────────
        boolean positionMode = false;

        while (opModeIsActive()) {

            // ── Button: position presets ────────────────────────
            if (gamepad1.a) {
                goToPosition(POS_CENTER);
                positionMode = true;
            } else if (gamepad1.x) {
                goToPosition(POS_LEFT);
                positionMode = true;
            } else if (gamepad1.b) {
                goToPosition(POS_RIGHT);
                positionMode = true;
            } else if (gamepad1.y) {
                // Return to manual mode
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotor.setPower(0);
                positionMode = false;
            }

            // ── Left stick: raw power (only in manual mode) ─────
            if (!positionMode) {
                double stick = -gamepad1.left_stick_y;   // push up = positive
                // small deadband to avoid drift
                if (Math.abs(stick) < 0.05) stick = 0.0;
                turretMotor.setPower(stick);
            }

            // ── Telemetry ───────────────────────────────────────
            int    ticks   = turretMotor.getCurrentPosition();
            double degrees = (ticks / TICKS_PER_TURRET_REV) * 360.0;
            double power   = turretMotor.getPower();
            double rpm     = turretMotor.getVelocity() / TICKS_PER_MOTOR_REV * 60.0; // motor shaft RPM

            telemetry.addData("Mode",           positionMode ? "POSITION" : "RAW POWER");
            telemetry.addLine("──────────────────────────────");
            telemetry.addData("Encoder Ticks",  ticks);
            telemetry.addData("Turret Angle",   "%.1f°", degrees);
            telemetry.addData("Motor Power",    "%.2f",  power);
            telemetry.addData("Motor RPM",      "%.0f",  rpm);
            telemetry.addLine("──────────────────────────────");
            telemetry.addData("Ticks/Rev (calc)", "%.1f", TICKS_PER_TURRET_REV);
            telemetry.addData("Gear Reduction",   "%.2f : 1", GEAR_REDUCTION);
            telemetry.update();
        }

        turretMotor.setPower(0);
    }

    /** Drive to a tick target using RUN_TO_POSITION at moderate speed */
    private void goToPosition(int targetTicks) {
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.4);   // ~40% — safe for testing, increase later
    }
}

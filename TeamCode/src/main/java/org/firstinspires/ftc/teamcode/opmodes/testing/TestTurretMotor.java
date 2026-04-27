package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TestTurretMotor", group = "testing")
public class TestTurretMotor extends LinearOpMode {

    static final double TICKS_PER_MOTOR_REV  = 28.0;
    static final double GEAR_REDUCTION       = (50.0/20.0) * (74.0/20.0) * (120.0/25.0);
    static final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * GEAR_REDUCTION;

    DcMotorEx turretMotor;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Point turret straight forward then press START");
        telemetry.update();
        waitForStart();

        // ── Step size options ──────────────────────────────────
        int[] stepSizes = {1, 5, 10, 50, 100};
        int stepIndex   = 2; // start at 10 ticks

        // ── Target tick tracking ───────────────────────────────
        int targetTicks = 0;

        // ── Edge detection ─────────────────────────────────────
        boolean dpadUpLast    = false;
        boolean dpadDownLast  = false;
        boolean dpadLeftLast  = false;
        boolean dpadRightLast = false;
        boolean aLast         = false;
        boolean yLast         = false;

        while (opModeIsActive()) {

            // ── A = reset encoder to 0 (mark this as front) ───
            boolean aNow = gamepad1.a;
            if (aNow && !aLast) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetTicks = 0;
            }
            aLast = aNow;

            // ── Y = cycle step size ────────────────────────────
            boolean yNow = gamepad1.y;
            if (yNow && !yLast) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            yLast = yNow;

            int step = stepSizes[stepIndex];

            // ── Dpad right = move right (positive ticks) ───────
            boolean dRightNow = gamepad1.dpad_right;
            if (dRightNow && !dpadRightLast) {
                targetTicks += step;
            }
            dpadRightLast = dRightNow;

            // ── Dpad left = move left (negative ticks) ─────────
            boolean dLeftNow = gamepad1.dpad_left;
            if (dLeftNow && !dpadLeftLast) {
                targetTicks -= step;
            }
            dpadLeftLast = dLeftNow;

            // ── Drive to target with simple P ─────────────────
            int    currentTicks = turretMotor.getCurrentPosition();
            double error        = targetTicks - currentTicks;
            double power        = error * 0.003;
            power = Math.max(-0.5, Math.min(0.5, power));
            if (Math.abs(error) < 5) power = 0;
            turretMotor.setPower(power);

            // ── Calculated angle based on current gear ratio ───
            double calculatedDegrees = (currentTicks / TICKS_PER_TURRET_REV) * 360.0;

            // ── Telemetry ──────────────────────────────────────
            telemetry.addLine("=== TURRET TICK FINDER ===");
            telemetry.addData("Step Size",         "%d ticks  (Y to change)", step);
            telemetry.addLine();
            telemetry.addData("Target Ticks",      targetTicks);
            telemetry.addData("Current Ticks",     currentTicks);
            telemetry.addData("Calculated Angle",  "%.1f°", calculatedDegrees);
            telemetry.addLine();
            telemetry.addLine("=== RECORD THESE ===");
            telemetry.addLine("Move to TRUE 90° RIGHT, note Current Ticks");
            telemetry.addLine("Move to TRUE 90° LEFT,  note Current Ticks");
            telemetry.addLine();
            telemetry.addData("Ticks for 90° (calc)", "%.0f", TICKS_PER_TURRET_REV * 0.25);
            telemetry.addData("Ticks/Rev (calc)",     "%.1f", TICKS_PER_TURRET_REV);
            telemetry.addLine();
            telemetry.addLine("CONTROLS:");
            telemetry.addLine("Dpad R/L = move  Y = step size  A = zero here");
            telemetry.update();
        }

        turretMotor.setPower(0);
    }
}
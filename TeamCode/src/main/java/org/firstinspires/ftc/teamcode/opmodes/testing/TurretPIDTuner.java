package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TurretPIDTuner", group = "testing")
public class TurretPIDTuner extends LinearOpMode {

    static final double TICKS_PER_MOTOR_REV  = 28.0;
    static final double GEAR_REDUCTION       = (50.0/20.0) * (74.0/21.0) * (120.0/25.0);
    static final double TICKS_PER_TURRET_REV = TICKS_PER_MOTOR_REV * GEAR_REDUCTION;

    double kP     = 0.04;
    double minPow = 0.12;
    int    deadband = 8;

    double[] kPSteps = {0.001, 0.005, 0.01, 0.05};
    double[] mpSteps = {0.005, 0.01,  0.02};
    int      kPStep  = 1;
    int      mpStep  = 1;

    DcMotorEx turretMotor;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Point turret forward then press START");
        telemetry.update();
        waitForStart();

        int targetTicks = 0;
        double ticksPer5Deg = (TICKS_PER_TURRET_REV / 360.0) * 5.0;

        // ── Edge detection ─────────────────────────────────────
        boolean rbLast  = false, lbLast  = false;
        boolean aLast   = false, bLast   = false;
        boolean xLast   = false, yLast   = false;
        boolean rtLast  = false, ltLast  = false;
        boolean dULast  = false, dDLast  = false;

        while (opModeIsActive()) {

            // ── Move target: LEFT STICK left/right ────────────
            double stick = gamepad1.left_stick_x;
            if (Math.abs(stick) > 0.5) {
                targetTicks += (int)(stick * ticksPer5Deg * 0.1);
            }

            // ── Also move target with bumpers for precise steps ─
            // RB = step right, LB = step left
            boolean rbNow = gamepad1.right_bumper;
            boolean lbNow = gamepad1.left_bumper;
            if (rbNow && !rbLast) targetTicks += (int) ticksPer5Deg;
            if (lbNow && !lbLast) targetTicks -= (int) ticksPer5Deg;
            rbLast = rbNow;
            lbLast = lbNow;

            // ── Zero encoder (A) ──────────────────────────────
            boolean aNow = gamepad1.a;
            if (aNow && !aLast) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetTicks = 0;
            }
            aLast = aNow;

            // ── kP up/down: DPAD UP/DOWN ──────────────────────
            boolean dUNow = gamepad1.dpad_up;
            boolean dDNow = gamepad1.dpad_down;
            if (dUNow && !dULast) kP += kPSteps[kPStep];
            if (dDNow && !dDLast) kP  = Math.max(0, kP - kPSteps[kPStep]);
            dULast = dUNow;
            dDLast = dDNow;

            // ── kP step size: X ───────────────────────────────
            boolean xNow = gamepad1.x;
            if (xNow && !xLast) kPStep = (kPStep + 1) % kPSteps.length;
            xLast = xNow;

            // ── minPow up/down: right/left trigger ────────────
            boolean rtNow = gamepad1.right_trigger > 0.5;
            boolean ltNow = gamepad1.left_trigger  > 0.5;
            if (rtNow && !rtLast) minPow += mpSteps[mpStep];
            if (ltNow && !ltLast) minPow  = Math.max(0, minPow - mpSteps[mpStep]);
            rtLast = rtNow;
            ltLast = ltNow;

            // ── minPow step size: Y ───────────────────────────
            boolean yNow = gamepad1.y;
            if (yNow && !yLast) mpStep = (mpStep + 1) % mpSteps.length;
            yLast = yNow;

            // ── PID ───────────────────────────────────────────
            int    currentTicks = turretMotor.getCurrentPosition();
            double error        = targetTicks - currentTicks;
            double power        = kP * error;
            power = Math.max(-1.0, Math.min(1.0, power));

            if (Math.abs(error) > deadband) {
                if (power > 0 && power < minPow)  power =  minPow;
                if (power < 0 && power > -minPow) power = -minPow;
            } else {
                power = 0;
            }

            turretMotor.setPower(power);

            double currentDeg = (currentTicks / TICKS_PER_TURRET_REV) * 360.0;
            double targetDeg  = (targetTicks  / TICKS_PER_TURRET_REV) * 360.0;

            telemetry.addLine("=== TURRET PID TUNER ===");
            telemetry.addLine();
            telemetry.addData("kP",      "%.4f   DPAD U/D | X=step %.3f", kP, kPSteps[kPStep]);
            telemetry.addData("minPow",  "%.3f   RT/LT    | Y=step %.3f", minPow, mpSteps[mpStep]);
            telemetry.addLine();
            telemetry.addData("Target",  "%d ticks  %.1f°", targetTicks, targetDeg);
            telemetry.addData("Current", "%d ticks  %.1f°", currentTicks, currentDeg);
            telemetry.addData("Error",   "%.0f ticks", error);
            telemetry.addData("Power",   "%.3f", power);
            telemetry.addLine();
            telemetry.addLine("LStick X = move smooth | LB/RB = move 5° steps | A = zero");
            telemetry.update();
        }

        turretMotor.setPower(0);
    }
}
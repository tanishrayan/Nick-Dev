package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp(name = "LimelightTurretTest", group = "testing")
public class LimelightTurretTest extends LinearOpMode {

    // ── Turret constants ───────────────────────────────────────
    static final double TICKS_PER_TURRET_REV = 1253.0;
    static final double kP_TURRET            = 0.008;
    static final double MAX_POWER            = 0.65;
    static final double MIN_POWER_FAR        = 0.14;
    static final double MIN_POWER_CLOSE      = 0.06;
    static final int    DEADBAND_TICKS       = 15;
    static final double FAR_THRESHOLD        = 100.0;

    // ── Limelight constants — tune these ──────────────────────
    double kP_LL        = 0.008;
    double llMaxSpeed   = 0.15;
    double deadbandDeg  = 2.0;
    double mountOffset  = 2.0;  // positive = camera points left of turret center
    static final int TARGET_TAG_ID = 20;

    DcMotorEx   turretMotor;
    Limelight3A limelight;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Point turret FORWARD then START");
        telemetry.update();
        waitForStart();

        // ── Modes ─────────────────────────────────────────────
        // false = manual stick control
        // true  = limelight correction active
        boolean llActive = false;

        int    targetTicks = 0;
        int    selected    = 0; // 0=kP_LL 1=llMaxSpeed 2=deadbandDeg 3=mountOffset

        boolean aLast  = false, bLast  = false;
        boolean rbLast = false, lbLast = false;
        boolean dULast = false, dDLast = false;
        boolean yLast  = false;

        while (opModeIsActive()) {

            // ── A = zero encoder ──────────────────────────────
            boolean aNow = gamepad1.a;
            if (aNow && !aLast) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetTicks = 0;
            }
            aLast = aNow;

            // ── B = toggle limelight correction on/off ────────
            boolean bNow = gamepad1.b;
            if (bNow && !bLast) llActive = !llActive;
            bLast = bNow;

            // ── Y = nudge turret off by +20° (to test correction) ─
            boolean yNow = gamepad1.y;
            if (yNow && !yLast) {
                targetTicks += (int)(TICKS_PER_TURRET_REV * (20.0 / 360.0));
            }
            yLast = yNow;

            // ── Left stick = manual move when LL off ──────────
            if (!llActive) {
                double stick = gamepad1.left_stick_x;
                if (Math.abs(stick) > 0.1) targetTicks += (int)(stick * 3);
            }

            // ── RB/LB = cycle selected LL variable ────────────
            boolean rbNow = gamepad1.right_bumper;
            boolean lbNow = gamepad1.left_bumper;
            if (rbNow && !rbLast) selected = (selected + 1) % 4;
            if (lbNow && !lbLast) selected = (selected + 3) % 4;
            rbLast = rbNow; lbLast = lbNow;

            // ── Dpad up/down = adjust selected ────────────────
            boolean dUNow = gamepad1.dpad_up;
            boolean dDNow = gamepad1.dpad_down;
            if (dUNow && !dULast) {
                if      (selected == 0) kP_LL       += 0.001;
                else if (selected == 1) llMaxSpeed  += 0.01;
                else if (selected == 2) deadbandDeg += 0.25;
                else if (selected == 3) mountOffset += 0.5;
            }
            if (dDNow && !dDLast) {
                if      (selected == 0) kP_LL       = Math.max(0, kP_LL - 0.001);
                else if (selected == 1) llMaxSpeed  = Math.max(0, llMaxSpeed - 0.01);
                else if (selected == 2) deadbandDeg = Math.max(0, deadbandDeg - 0.25);
                else if (selected == 3) mountOffset -= 0.5;
            }
            dULast = dUNow; dDLast = dDNow;

            // ── Get limelight tag ─────────────────────────────
            LLResultTypes.FiducialResult tag = getTag();

            // ── Motor control ─────────────────────────────────
            double power = 0;

            if (llActive && tag != null) {
                // Limelight correction
                double tx = tag.getTargetXDegrees() + mountOffset;
                if (Math.abs(tx) > deadbandDeg) {
                    power = clamp(kP_LL * tx, -llMaxSpeed, llMaxSpeed);
                } else {
                    power = 0; // locked
                }
                turretMotor.setPower(power);
            } else {
                // Manual PID to target ticks
                int    currentTicks = turretMotor.getCurrentPosition();
                double error        = targetTicks - currentTicks;
                power = kP_TURRET * error;
                power = clamp(power, -MAX_POWER, MAX_POWER);

                if (Math.abs(error) > DEADBAND_TICKS) {
                    double minPow = Math.abs(error) > FAR_THRESHOLD ? MIN_POWER_FAR : MIN_POWER_CLOSE;
                    if (power > 0 && power < minPow)  power =  minPow;
                    if (power < 0 && power > -minPow) power = -minPow;
                } else {
                    power = 0;
                }
                turretMotor.setPower(power);
            }

            // ── Telemetry ─────────────────────────────────────
            int    currentTicks = turretMotor.getCurrentPosition();
            double currentDeg   = -(currentTicks / TICKS_PER_TURRET_REV) * 360.0;

            String[] names = {"kP_LL", "llMaxSpeed", "deadbandDeg", "mountOffset"};

            telemetry.addLine("=== LIMELIGHT TURRET TEST ===");
            telemetry.addData("Mode",        llActive ? "LIMELIGHT ACTIVE" : "MANUAL");
            telemetry.addData("Tag Visible", tag != null ? "YES" : "NO");
            telemetry.addLine();
            telemetry.addData(">> Editing",  names[selected] + "  (RB/LB switch, DPAD U/D change)");
            telemetry.addData(selected==0?">> kP_LL"      :"   kP_LL",       "%.4f", kP_LL);
            telemetry.addData(selected==1?">> llMaxSpeed" :"   llMaxSpeed",  "%.3f", llMaxSpeed);
            telemetry.addData(selected==2?">> deadband"   :"   deadband",    "%.2f°", deadbandDeg);
            telemetry.addData(selected==3?">> mountOffset":"   mountOffset", "%.1f°", mountOffset);
            telemetry.addLine();
            if (tag != null) {
                telemetry.addData("tx raw",      "%.2f°", tag.getTargetXDegrees());
                telemetry.addData("tx adjusted", "%.2f°", tag.getTargetXDegrees() + mountOffset);
            }
            telemetry.addData("Turret Angle", "%.1f°", currentDeg);
            telemetry.addData("Motor Power",  "%.3f",  power);
            telemetry.addLine();
            telemetry.addLine("B=toggle LL  Y=nudge +20°  A=zero  Stick=manual");
            telemetry.update();
        }

        turretMotor.setPower(0);
        limelight.stop();
    }

    private LLResultTypes.FiducialResult getTag() {
        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult t : tags)
                        if (t.getFiducialId() == TARGET_TAG_ID) return t;
                }
            }
        } catch (Exception e) { /* skip */ }
        return null;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
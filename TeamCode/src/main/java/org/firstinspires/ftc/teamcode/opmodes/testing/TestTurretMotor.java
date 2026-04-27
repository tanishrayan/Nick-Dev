package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TestTurretMotor", group = "testing")
public class TestTurretMotor extends LinearOpMode {

    static double TICKS_PER_TURRET_REV = 1103.0;

    double kP          = 0.008;
    double minPowFar   = 0.14;
    double minPowClose = 0.09;
    double maxPow      = 0.7;
    int    deadband    = 15;

    DcMotorEx turretMotor;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Point turret FORWARD then START");
        telemetry.update();
        waitForStart();

        int targetTicks = 0;
        int selected    = 0; // 0=kP 1=minFar 2=minClose 3=maxPow 4=ticksPerRev

        boolean aLast  = false, bLast  = false, xLast  = false;
        boolean rbLast = false, lbLast = false;
        boolean dULast = false, dDLast = false;

        while (opModeIsActive()) {

            // A = forward, B = +90, X = -90
            boolean aNow = gamepad1.a;
            boolean bNow = gamepad1.b;
            boolean xNow = gamepad1.x;
            if (aNow && !aLast) targetTicks = 0;
            if (bNow && !bLast) targetTicks =  (int)(TICKS_PER_TURRET_REV * 0.25);
            if (xNow && !xLast) targetTicks = -(int)(TICKS_PER_TURRET_REV * 0.25);
            aLast = aNow; bLast = bNow; xLast = xNow;

            // Y = zero encoder
            if (gamepad1.y) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                targetTicks = 0;
            }

            // RB/LB = cycle selected variable
            boolean rbNow = gamepad1.right_bumper;
            boolean lbNow = gamepad1.left_bumper;
            if (rbNow && !rbLast) selected = (selected + 1) % 5;
            if (lbNow && !lbLast) selected = (selected + 4) % 5;
            rbLast = rbNow; lbLast = lbNow;

            // Dpad up/down = adjust selected
            boolean dUNow = gamepad1.dpad_up;
            boolean dDNow = gamepad1.dpad_down;
            if (dUNow && !dULast) {
                if      (selected == 0) kP          += 0.001;
                else if (selected == 1) minPowFar   += 0.005;
                else if (selected == 2) minPowClose += 0.005;
                else if (selected == 3) maxPow       = Math.min(1.0, maxPow + 0.05);
                else if (selected == 4) TICKS_PER_TURRET_REV += 5;
            }
            if (dDNow && !dDLast) {
                if      (selected == 0) kP          = Math.max(0, kP - 0.001);
                else if (selected == 1) minPowFar   = Math.max(0, minPowFar - 0.005);
                else if (selected == 2) minPowClose = Math.max(0, minPowClose - 0.005);
                else if (selected == 3) maxPow      = Math.max(0, maxPow - 0.05);
                else if (selected == 4) TICKS_PER_TURRET_REV = Math.max(100, TICKS_PER_TURRET_REV - 5);
            }
            dULast = dUNow; dDLast = dDNow;

            // PID
            int    currentTicks = turretMotor.getCurrentPosition();
            double error        = targetTicks - currentTicks;
            double power        = kP * error;
            power = Math.max(-maxPow, Math.min(maxPow, power));

            if (Math.abs(error) > deadband) {
                double minPow = Math.abs(error) > 100 ? minPowFar : minPowClose;
                if (power > 0 && power < minPow)  power =  minPow;
                if (power < 0 && power > -minPow) power = -minPow;
            } else {
                power = 0;
            }

            turretMotor.setPower(power);

            double currentDeg = (currentTicks / TICKS_PER_TURRET_REV) * 360.0;
            double targetDeg  = (targetTicks  / TICKS_PER_TURRET_REV) * 360.0;

            String[] names = {"kP", "minPowFar", "minPowClose", "maxPow", "ticksPerRev"};
            telemetry.addLine("=== TURRET TUNER ===");
            telemetry.addData(">> Editing", names[selected] + "  (RB/LB switch, DPAD U/D change)");
            telemetry.addLine();
            telemetry.addData(selected==0?">> kP"         :"   kP",          "%.4f", kP);
            telemetry.addData(selected==1?">> minPowFar"  :"   minPowFar",   "%.3f", minPowFar);
            telemetry.addData(selected==2?">> minPowClose":"   minPowClose",  "%.3f", minPowClose);
            telemetry.addData(selected==3?">> maxPow"     :"   maxPow",      "%.2f", maxPow);
            telemetry.addData(selected==4?">> ticksPerRev":"   ticksPerRev", "%.0f", TICKS_PER_TURRET_REV);
            telemetry.addLine();
            telemetry.addData("Target",  "%.1f°  (%d ticks)", targetDeg, targetTicks);
            telemetry.addData("Current", "%.1f°  (%d ticks)", currentDeg, currentTicks);
            telemetry.addData("Error",   "%.0f ticks", error);
            telemetry.addData("Power",   "%.3f", power);
            telemetry.addLine();
            telemetry.addLine("A=0°  B=+90°  X=-90°  Y=zero encoder");
            telemetry.update();
        }

        turretMotor.setPower(0);
    }
}
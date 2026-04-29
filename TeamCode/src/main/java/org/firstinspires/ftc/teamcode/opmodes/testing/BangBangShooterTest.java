package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BangBang Shooter Test", group = "Testing")
public class BangBangShooterTest extends OpMode {

    private DcMotorEx topMotor, bottomMotor;
    private Servo     hoodServo, latchServo;
    private DcMotorEx intakeMotor;
    private Servo     intakePivot, intakePivot2;

    private static final double HOOD_MIN        = 0.269;
    private static final double HOOD_MAX        = 0.89;
    private static final double LATCH_CLOSED    = 0.843;
    private static final double LATCH_OPEN      = 0.6;
    private static final double PIVOT_RETRACTED = 0.44;
    private static final double PIVOT_DEPLOYED  = 0.125;

    private double  bangBangDeadband = 0.0;
    private double  burstOffset      = 200.0;
    private boolean burstModeActive  = false;

    private double  targetVelocity  = 1200.0;
    private double  hoodPosition    = 0.269;
    private boolean flywheelRunning = false;
    private boolean latchOpen       = false;
    private boolean intakeDeployed  = false;

    private enum TuningMode { VELOCITY, DEADBAND, BURST_OFFSET }
    private TuningMode tuningMode = TuningMode.VELOCITY;

    private double[] stepSizes = {50.0, 10.0, 5.0, 1.0};
    private int      stepIndex = 1;

    private double  peakAbove     = 0;
    private double  peakBelow     = 0;
    private double  oscResetTimer = 0;
    private boolean wasBelow      = false;
    private double  recoveryStart = 0;
    private double  lastRecoveryMs = 0;

    private static final double VELOCITY_INCREMENT      = 100.0;
    private static final double VELOCITY_FINE_INCREMENT = 25.0;
    private static final double HOOD_INCREMENT          = 0.05;

    private boolean dpadUpPressed    = false;
    private boolean dpadDownPressed  = false;
    private boolean dpadLeftPressed  = false;
    private boolean dpadRightPressed = false;
    private boolean l1Pressed        = false;
    private boolean r1Pressed        = false;
    private boolean circlePressed    = false;
    private boolean squarePressed    = false;
    private boolean rbPressed        = false;
    private boolean lbPressed        = false;
    private boolean sharePressed     = false;
    private boolean optionsPressed   = false;

    @Override
    public void init() {
        topMotor    = hardwareMap.get(DcMotorEx.class, "topMotor");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottomMotor");
        hoodServo   = hardwareMap.get(Servo.class, "hood");
        latchServo  = hardwareMap.get(Servo.class, "latch");

        intakeMotor  = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakePivot  = hardwareMap.get(Servo.class, "intakePivot");
        intakePivot2 = hardwareMap.get(Servo.class, "intakePivot2");

        // Top FORWARD, bottom REVERSE — opposite directions so both wheels
        // grip the ball from each side. Matches ShooterIntakeTest which uses
        // setVelocity(+target) for top and setVelocity(-target) for bottom.
        topMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hoodServo.setPosition(hoodPosition);
        latchServo.setPosition(LATCH_CLOSED);
        setIntakeRetracted();

        telemetry.addLine("BangBang Shooter Test ready.");
        telemetry.addLine("Top=FORWARD  Bottom=REVERSE  Deadband=0 (aggressive)");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Hold Cross to activate burst pre-spin: raises effective target by
        // burstOffset so flywheel starts higher before firing 3 balls.
        // Even after 3 compounding drops, ball 3 should still be above minimum.
        burstModeActive = gamepad1.cross;
        double effectiveTarget = burstModeActive
                ? targetVelocity + burstOffset
                : targetVelocity;

        // Both getVelocity() should read positive — setDirection handles orientation.
        double topVel    = topMotor.getVelocity();
        double bottomVel = bottomMotor.getVelocity();
        double error     = effectiveTarget - topVel;

        // Aggressive bang-bang: zero deadband, full power the instant speed
        // drops below effectiveTarget, cuts power the instant it hits target.
        // Both setPower(1) since setDirection already handles spin orientation.
        if (flywheelRunning) {
            if (topVel > effectiveTarget + bangBangDeadband) {
                topMotor.setPower(0);
                bottomMotor.setPower(0);
            } else if (topVel < effectiveTarget - bangBangDeadband) {
                topMotor.setPower(1);
                bottomMotor.setPower(1);
            } else {
                topMotor.setPower(0);
                bottomMotor.setPower(0);
            }
        } else {
            topMotor.setPower(0);
            bottomMotor.setPower(0);
        }

        // Oscillation tracking — rolling 3s window.
        oscResetTimer += 0.02;
        if (oscResetTimer > 3.0) {
            peakAbove = 0; peakBelow = 0; oscResetTimer = 0;
        }
        if (flywheelRunning) {
            if (topVel - effectiveTarget > peakAbove) peakAbove = topVel - effectiveTarget;
            if (effectiveTarget - topVel > peakBelow) peakBelow = effectiveTarget - topVel;
        }

        // Recovery timer: stamps ms from dip below target to return.
        boolean isBelow = topVel < effectiveTarget - Math.max(bangBangDeadband, 10);
        if (isBelow && !wasBelow)  recoveryStart = System.currentTimeMillis();
        if (!isBelow && wasBelow)  lastRecoveryMs = System.currentTimeMillis() - recoveryStart;
        wasBelow = isBelow;

        // Share: cycle tuning mode for DPAD U/D
        boolean shareNow = gamepad1.share;
        if (shareNow && !sharePressed) {
            switch (tuningMode) {
                case VELOCITY:     tuningMode = TuningMode.DEADBAND;      break;
                case DEADBAND:     tuningMode = TuningMode.BURST_OFFSET;  break;
                case BURST_OFFSET: tuningMode = TuningMode.VELOCITY;      break;
            }
        }
        sharePressed = shareNow;

        // Options: cycle step size
        boolean optNow = gamepad1.options;
        if (optNow && !optionsPressed) stepIndex = (stepIndex + 1) % stepSizes.length;
        optionsPressed = optNow;

        // Circle: flywheel toggle
        boolean circleNow = gamepad1.circle;
        if (circleNow && !circlePressed) {
            flywheelRunning = !flywheelRunning;
            if (!flywheelRunning) { topMotor.setPower(0); bottomMotor.setPower(0); }
            peakAbove = 0; peakBelow = 0; oscResetTimer = 0; lastRecoveryMs = 0;
        }
        circlePressed = circleNow;

        // Square: intake pivot toggle
        boolean squareNow = gamepad1.square;
        if (squareNow && !squarePressed) {
            intakeDeployed = !intakeDeployed;
            if (intakeDeployed) setIntakeDeployed(); else setIntakeRetracted();
        }
        squarePressed = squareNow;

        // Triangle hold: intake motor
        intakeMotor.setPower(gamepad1.triangle ? 1.0 : 0.0);

        // RB: toggle latch
        boolean rbNow = gamepad1.right_bumper;
        if (rbNow && !rbPressed) {
            latchOpen = !latchOpen;
            latchServo.setPosition(latchOpen ? LATCH_OPEN : LATCH_CLOSED);
        }
        rbPressed = rbNow;

        // LB: force close latch
        boolean lbNow = gamepad1.left_bumper;
        if (lbNow && !lbPressed) { latchOpen = false; latchServo.setPosition(LATCH_CLOSED); }
        lbPressed = lbNow;

        // DPAD U/D: tune whichever mode is active
        boolean dUpNow = gamepad1.dpad_up, dDownNow = gamepad1.dpad_down;
        if (dUpNow && !dpadUpPressed) {
            switch (tuningMode) {
                case VELOCITY:     targetVelocity   += VELOCITY_INCREMENT;    break;
                case DEADBAND:     bangBangDeadband += stepSizes[stepIndex];  break;
                case BURST_OFFSET: burstOffset      += stepSizes[stepIndex];  break;
            }
        }
        if (dDownNow && !dpadDownPressed) {
            switch (tuningMode) {
                case VELOCITY:     targetVelocity   = Math.max(0, targetVelocity   - VELOCITY_INCREMENT);   break;
                case DEADBAND:     bangBangDeadband = Math.max(0, bangBangDeadband - stepSizes[stepIndex]); break;
                case BURST_OFFSET: burstOffset      = Math.max(0, burstOffset      - stepSizes[stepIndex]); break;
            }
        }
        dpadUpPressed = dUpNow; dpadDownPressed = dDownNow;

        // DPAD L/R: hood
        boolean dLeftNow = gamepad1.dpad_left, dRightNow = gamepad1.dpad_right;
        if (dLeftNow && !dpadLeftPressed) {
            hoodPosition = Math.max(HOOD_MIN, hoodPosition - HOOD_INCREMENT);
            hoodServo.setPosition(hoodPosition);
        }
        if (dRightNow && !dpadRightPressed) {
            hoodPosition = Math.min(HOOD_MAX, hoodPosition + HOOD_INCREMENT);
            hoodServo.setPosition(hoodPosition);
        }
        dpadLeftPressed = dLeftNow; dpadRightPressed = dRightNow;

        // Triggers: fine velocity
        if (gamepad1.left_trigger > 0.5 && !l1Pressed) {
            targetVelocity = Math.max(0, targetVelocity - VELOCITY_FINE_INCREMENT);
            l1Pressed = true;
        } else if (gamepad1.left_trigger < 0.3) { l1Pressed = false; }
        if (gamepad1.right_trigger > 0.5 && !r1Pressed) {
            targetVelocity += VELOCITY_FINE_INCREMENT;
            r1Pressed = true;
        } else if (gamepad1.right_trigger < 0.3) { r1Pressed = false; }

        // Telemetry
        telemetry.addLine("=== MODE ===");
        telemetry.addData("Control",    "BANG-BANG (AGGRESSIVE)");
        telemetry.addData("DPAD tunes", tuningMode.toString());
        telemetry.addData("Step size",  "%.1f", stepSizes[stepIndex]);
        telemetry.addLine();
        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("Status",           flywheelRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Base target",      "%.0f ticks/s", targetVelocity);
        telemetry.addData("Effective target", "%.0f ticks/s%s", effectiveTarget,
                burstModeActive ? "  (BURST +" + (int) burstOffset + ")" : "");
        telemetry.addData("Top actual",       "%.0f ticks/s", topVel);
        telemetry.addData("Bottom actual",    "%.0f ticks/s", bottomVel);
        telemetry.addData("Error",            "%.0f ticks/s", error);
        telemetry.addLine();
        telemetry.addLine("=== BANG-BANG ===");
        telemetry.addData("Deadband",          "%.1f ticks/s", bangBangDeadband);
        telemetry.addData("Burst offset",      "%.0f ticks/s", burstOffset);
        telemetry.addData("Motor power",       topMotor.getPower() > 0.5 ? "FULL ON" : "COAST");
        telemetry.addData("At speed?",         Math.abs(error) < Math.max(bangBangDeadband, 10)
                ? "YES ✓" : "NO  (" + (int) error + ")");
        telemetry.addLine("-- Oscillation (resets 3s) --");
        telemetry.addData("Peak above target", "%.1f ticks/s", peakAbove);
        telemetry.addData("Peak below target", "%.1f ticks/s", peakBelow);
        telemetry.addLine("-- Recovery --");
        telemetry.addData("Last recovery",     "%.0f ms", lastRecoveryMs);
        telemetry.addLine();
        telemetry.addLine("=== HOOD ===");
        telemetry.addData("Position", "%.3f", hoodPosition);
        telemetry.addLine();
        telemetry.addLine("=== LATCH ===");
        telemetry.addData("State", latchOpen ? "OPEN" : "CLOSED");
        telemetry.addLine();
        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Pivot", intakeDeployed ? "DEPLOYED" : "RETRACTED");
        telemetry.addData("Motor", "%.2f", intakeMotor.getPower());
        telemetry.addLine();
        telemetry.addLine("Circle=flywheel  Square=pivot  Triangle=intake");
        telemetry.addLine("RB=latch  LB=close  Cross(hold)=burst pre-spin");
        telemetry.addLine("Share=cycle tune mode  Options=step size");
        telemetry.addLine("DPAD U/D=tune active  DPAD L/R=hood  Triggers=fine vel");
        telemetry.update();
    }

    private void setIntakeDeployed() {
        intakePivot.setPosition(PIVOT_DEPLOYED);
        intakePivot2.setPosition(1.0 - PIVOT_DEPLOYED);
    }

    private void setIntakeRetracted() {
        intakePivot.setPosition(PIVOT_RETRACTED);
        intakePivot2.setPosition(1.0 - PIVOT_RETRACTED);
    }

    @Override
    public void stop() {
        topMotor.setPower(0);
        bottomMotor.setPower(0);
        intakeMotor.setPower(0);
        latchServo.setPosition(LATCH_CLOSED);
        setIntakeRetracted();
    }
}
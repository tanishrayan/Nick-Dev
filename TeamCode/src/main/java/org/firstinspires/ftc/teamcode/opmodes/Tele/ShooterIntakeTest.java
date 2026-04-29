package org.firstinspires.ftc.teamcode.opmodes.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter Intake Test", group = "Testing")
public class ShooterIntakeTest extends OpMode {

    // ── Shooter ────────────────────────────────────────────────
    private DcMotorEx topMotor, bottomMotor;
    private Servo     hoodServo;
    private Servo     latchServo;

    // ── Intake ─────────────────────────────────────────────────
    private DcMotorEx intakeMotor;
    private Servo     intakePivot;
    private Servo     intakePivot2;

    // ── Shooter values ─────────────────────────────────────────
    private double targetVelocity = 1000.0;
    private double hoodPosition   = 0.269;

    // ── Hood limits ───────────────────────────────────────────
    private static final double HOOD_MIN = 0.269;
    private static final double HOOD_MAX = 0.89;

    // ── PIDF ──────────────────────────────────────────────────
    private double   P         = 240.0;
    private double   F         = 15.8;
    private double[] stepSizes = {100.0, 10.0, 1.0, 0.1, 0.01};
    private int      stepIndex = 2;

    // ── Servo positions ────────────────────────────────────────
    private static final double LATCH_CLOSED    = 0.843;
    private static final double LATCH_OPEN      = 0.6;
    private static final double PIVOT_RETRACTED = 0.44;
    private static final double PIVOT_DEPLOYED  = 0.125;
    private static final double PIVOT_FLOOR     = 0.05;

    // ── Increments ────────────────────────────────────────────
    private static final double VELOCITY_INCREMENT      = 100.0;
    private static final double VELOCITY_FINE_INCREMENT = 25.0;
    private static final double HOOD_INCREMENT          = 0.05;

    // ── States ────────────────────────────────────────────────
    private boolean flywheelRunning = false;
    private boolean intakeDeployed  = false;
    private boolean pidfTuningMode  = false;
    private boolean latchOpen       = false;

    // ── Edge detection ─────────────────────────────────────────
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

        topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setPIDFCoefficients(P, F);

        hoodServo.setPosition(hoodPosition);
        latchServo.setPosition(LATCH_CLOSED);
        latchOpen = false;
        setIntakeRetracted();

        telemetry.addLine("Shooter Intake Test ready.");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ── Avg velocity ───────────────────────────────────────
        double avg   = (Math.abs(topMotor.getVelocity()) + Math.abs(bottomMotor.getVelocity())) / 2.0;
        double error = targetVelocity - avg;

        // ── Share = PIDF mode toggle ───────────────────────────
        boolean shareNow = gamepad1.share;
        if (shareNow && !sharePressed) pidfTuningMode = !pidfTuningMode;
        sharePressed = shareNow;

        // ── Options = step size ────────────────────────────────
        boolean optNow = gamepad1.options;
        if (optNow && !optionsPressed) stepIndex = (stepIndex + 1) % stepSizes.length;
        optionsPressed = optNow;

        // ── Circle = flywheel toggle ───────────────────────────
        boolean circleNow = gamepad1.circle;
        if (circleNow && !circlePressed) {
            flywheelRunning = !flywheelRunning;
            if (flywheelRunning) {
                topMotor.setVelocity(targetVelocity);
                bottomMotor.setVelocity(-targetVelocity);
            } else {
                topMotor.setPower(0);
                bottomMotor.setPower(0);
            }
        }
        circlePressed = circleNow;

        // ── Square = intake pivot toggle ───────────────────────
        boolean squareNow = gamepad1.square;
        if (squareNow && !squarePressed) {
            intakeDeployed = !intakeDeployed;
            if (intakeDeployed) setIntakeDeployed();
            else setIntakeRetracted();
        }
        squarePressed = squareNow;

        // ── Triangle hold = intake motor ───────────────────────
        intakeMotor.setPower(gamepad1.triangle ? 1.0 : 0.0);

        // ── RB = toggle latch (always works) ──────────────────
        boolean rbNow = gamepad1.right_bumper;
        if (rbNow && !rbPressed) {
            latchOpen = !latchOpen;
            latchServo.setPosition(latchOpen ? LATCH_OPEN : LATCH_CLOSED);
        }
        rbPressed = rbNow;

        // ── LB = force close latch ────────────────────────────
        boolean lbNow = gamepad1.left_bumper;
        if (lbNow && !lbPressed) {
            latchOpen = false;
            latchServo.setPosition(LATCH_CLOSED);
        }
        lbPressed = lbNow;

        // ── PIDF tuning or normal mode ─────────────────────────
        if (pidfTuningMode) {
            boolean dUpNow   = gamepad1.dpad_up;
            boolean dDownNow = gamepad1.dpad_down;
            if (dUpNow   && !dpadUpPressed)  { P += stepSizes[stepIndex]; setPIDFCoefficients(P, F); }
            if (dDownNow && !dpadDownPressed) { P = Math.max(0, P - stepSizes[stepIndex]); setPIDFCoefficients(P, F); }
            dpadUpPressed   = dUpNow;
            dpadDownPressed = dDownNow;

            boolean dLeftNow  = gamepad1.dpad_left;
            boolean dRightNow = gamepad1.dpad_right;
            if (dLeftNow  && !dpadLeftPressed)  { F = Math.max(0, F - stepSizes[stepIndex]); setPIDFCoefficients(P, F); }
            if (dRightNow && !dpadRightPressed) { F += stepSizes[stepIndex]; setPIDFCoefficients(P, F); }
            dpadLeftPressed  = dLeftNow;
            dpadRightPressed = dRightNow;

        } else {
            boolean dUpNow   = gamepad1.dpad_up;
            boolean dDownNow = gamepad1.dpad_down;
            if (dUpNow && !dpadUpPressed) {
                targetVelocity += VELOCITY_INCREMENT;
                if (flywheelRunning) { topMotor.setVelocity(targetVelocity); bottomMotor.setVelocity(-targetVelocity); }
            }
            if (dDownNow && !dpadDownPressed) {
                targetVelocity = Math.max(0, targetVelocity - VELOCITY_INCREMENT);
                if (flywheelRunning) { topMotor.setVelocity(targetVelocity); bottomMotor.setVelocity(-targetVelocity); }
            }
            dpadUpPressed   = dUpNow;
            dpadDownPressed = dDownNow;

            boolean dLeftNow  = gamepad1.dpad_left;
            boolean dRightNow = gamepad1.dpad_right;
            if (dLeftNow && !dpadLeftPressed) {
                hoodPosition = Math.max(HOOD_MIN, hoodPosition - HOOD_INCREMENT);
                hoodServo.setPosition(hoodPosition);
            }
            if (dRightNow && !dpadRightPressed) {
                hoodPosition = Math.min(HOOD_MAX, hoodPosition + HOOD_INCREMENT);
                hoodServo.setPosition(hoodPosition);
            }
            dpadLeftPressed  = dLeftNow;
            dpadRightPressed = dRightNow;
        }

        // ── Velocity fine (triggers) ───────────────────────────
        if (gamepad1.left_trigger > 0.5 && !l1Pressed) {
            targetVelocity = Math.max(0, targetVelocity - VELOCITY_FINE_INCREMENT);
            if (flywheelRunning) { topMotor.setVelocity(targetVelocity); bottomMotor.setVelocity(-targetVelocity); }
            l1Pressed = true;
        } else if (gamepad1.left_trigger < 0.3) { l1Pressed = false; }

        if (gamepad1.right_trigger > 0.5 && !r1Pressed) {
            targetVelocity += VELOCITY_FINE_INCREMENT;
            if (flywheelRunning) { topMotor.setVelocity(targetVelocity); bottomMotor.setVelocity(-targetVelocity); }
            r1Pressed = true;
        } else if (gamepad1.right_trigger < 0.3) { r1Pressed = false; }

        // ── Telemetry ──────────────────────────────────────────
        telemetry.addLine("=== MODE ===");
        telemetry.addData("Control Mode",    pidfTuningMode ? "PIDF TUNING" : "NORMAL");
        telemetry.addData("Flywheel Ready?", Math.abs(error) < 20 ? "YES ✓" : "NO  (" + (int)error + ")");
        telemetry.addLine();
        telemetry.addLine("=== FLYWHEEL ===");
        telemetry.addData("Status",        flywheelRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Target",        "%.0f ticks/s", targetVelocity);
        telemetry.addData("Top actual",    "%.0f ticks/s", topMotor.getVelocity());
        telemetry.addData("Bottom actual", "%.0f ticks/s", bottomMotor.getVelocity());
        telemetry.addData("Avg velocity",  "%.0f ticks/s", avg);
        telemetry.addData("Error",         "%.0f ticks/s", error);
        telemetry.addLine();
        telemetry.addLine("=== PIDF ===");
        telemetry.addData("P",         "%.2f", P);
        telemetry.addData("F",         "%.2f", F);
        telemetry.addData("Step size", "%.2f", stepSizes[stepIndex]);
        telemetry.addLine();
        telemetry.addLine("=== HOOD ===");
        telemetry.addData("Position",  "%.3f", hoodPosition);
        telemetry.addLine();
        telemetry.addLine("=== LATCH ===");
        telemetry.addData("State",     latchOpen ? "OPEN" : "CLOSED");
        telemetry.addLine();
        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("Pivot",     intakeDeployed ? "DEPLOYED" : "RETRACTED");
        telemetry.addData("Motor",     "%.2f", intakeMotor.getPower());
        telemetry.addLine();
        telemetry.addLine("Circle=flywheel  Square=pivot  Triangle=intake");
        telemetry.addLine("RB=latch  LB=close  Share=PIDF  Options=step");
        telemetry.addLine("DPAD U/D=vel  DPAD L/R=hood  Triggers=fine vel");
        telemetry.update();
    }

    private void setIntakeDeployed() {
        intakePivot.setPosition(Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
        intakePivot2.setPosition(1.0 - Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
    }

    private void setIntakeRetracted() {
        intakePivot.setPosition(PIVOT_RETRACTED);
        intakePivot2.setPosition(1.0 - PIVOT_RETRACTED);
    }

    private void setPIDFCoefficients(double p, double f) {
        PIDFCoefficients pidf = new PIDFCoefficients(p, 0, 0, f);
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
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
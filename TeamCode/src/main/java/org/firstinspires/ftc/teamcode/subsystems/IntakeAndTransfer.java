package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeAndTransfer {

    // ── Hardware ───────────────────────────────────────────────
    private DcMotorEx intakeMotor;
    private Servo     intakePivot;
    private Servo     intakePivot2;

    // ── Break beams — one per ball slot ───────────────────────
    private BeambreakSensor beamTop;   // port 0 — first ball to enter, closest to shooter
    private BeambreakSensor beamMid;   // port 2 — middle ball
    private BeambreakSensor beamFront; // port 4 — last ball to enter, closest to ground

    // ── Pivot positions ────────────────────────────────────────
    private static final double PIVOT_RETRACTED = 0.44;
    private static final double PIVOT_DEPLOYED  = 0.125;
    private static final double PIVOT_FLOOR     = 0.05;

    // ── Motor power ───────────────────────────────────────────
    private static final double INTAKE_POWER  =  1.0;
    private static final double OUTTAKE_POWER = -0.5;

    // ── State machine ─────────────────────────────────────────
    public enum IntakeState { IDLE, INTAKING, OUTTAKING }
    private IntakeState currentState = IntakeState.IDLE;

    private Telemetry telemetry;

    // ── Constructor ────────────────────────────────────────────
    public IntakeAndTransfer(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakePivot  = hardwareMap.get(Servo.class, "intakePivot");
        intakePivot2 = hardwareMap.get(Servo.class, "intakePivot2");

        beamTop   = new BeambreakSensor(hardwareMap, "beamTop");
        beamMid   = new BeambreakSensor(hardwareMap, "beamMid");
        beamFront = new BeambreakSensor(hardwareMap, "beamFront");

        setIdle();
    }

    // ── State control ─────────────────────────────────────────

    public void setState(IntakeState state) {
        currentState = state;
        switch (state) {
            case IDLE:
                intakePivot.setPosition(PIVOT_RETRACTED);
                intakePivot2.setPosition(1.0 - PIVOT_RETRACTED);
                intakeMotor.setPower(0);
                break;
            case INTAKING:
                intakePivot.setPosition(Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
                intakePivot2.setPosition(1.0 - Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
                intakeMotor.setPower(INTAKE_POWER);
                break;
            case OUTTAKING:
                intakePivot.setPosition(Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
                intakePivot2.setPosition(1.0 - Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
                intakeMotor.setPower(OUTTAKE_POWER);
                break;
        }
    }

    public void setIdle()      { setState(IntakeState.IDLE); }
    public void setIntaking()  { setState(IntakeState.INTAKING); }
    public void setOuttaking() { setState(IntakeState.OUTTAKING); }

    public void toggleIntake() {
        if (currentState == IntakeState.INTAKING) setState(IntakeState.IDLE);
        else setState(IntakeState.INTAKING);
    }

    // ── Break beam reads ───────────────────────────────────────

    public boolean isTopBallPresent()   { return beamTop.isBlocked(); }
    public boolean isMidBallPresent()   { return beamMid.isBlocked(); }
    public boolean isFrontBallPresent() { return beamFront.isBlocked(); }

    public boolean isFullyLoaded() {
        return isTopBallPresent() && isMidBallPresent() && isFrontBallPresent();
    }

    public boolean isEmpty() {
        return !isTopBallPresent() && !isMidBallPresent() && !isFrontBallPresent();
    }

    public int getBallCount() {
        return (isTopBallPresent()   ? 1 : 0)
                + (isMidBallPresent()   ? 1 : 0)
                + (isFrontBallPresent() ? 1 : 0);
    }

    /** True when front beam is clear — ball has left toward shooter */
    public boolean isFrontBeamClear() { return !isFrontBallPresent(); }

    // ── Getters ───────────────────────────────────────────────

    public IntakeState getState()   { return currentState; }
    public boolean     isIntaking() { return currentState == IntakeState.INTAKING; }

    // ── Telemetry ─────────────────────────────────────────────

    public void setTelemetry(Telemetry telem) { this.telemetry = telem; }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Intake State",    currentState);
        telemetry.addData("Ball Count",      getBallCount() + " / 3");
        telemetry.addData("Top  (shooter)",  isTopBallPresent()   ? "BALL ●" : "empty ○");
        telemetry.addData("Mid",             isMidBallPresent()   ? "BALL ●" : "empty ○");
        telemetry.addData("Front (ground)",  isFrontBallPresent() ? "BALL ●" : "empty ○");
        telemetry.addData("Pivot Position",  "%.2f", intakePivot.getPosition());
        telemetry.addData("Motor Power",     "%.2f", intakeMotor.getPower());
        beamTop.updateTelemetry();
        beamMid.updateTelemetry();
        beamFront.updateTelemetry();
    }
}
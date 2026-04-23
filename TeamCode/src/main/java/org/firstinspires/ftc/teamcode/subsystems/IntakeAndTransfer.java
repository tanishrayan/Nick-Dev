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
    private Servo intakePivot2; // second pivot servo

    // ── Break beams (one BeambreakSensor per digital channel pin) ─
    // Each physical sensor spans two pins — both are checked per slot
    private BeambreakSensor beam1, break1; // slot 1 — top (first ball to enter)
    private BeambreakSensor beam2, break2; // slot 2 — middle
    private BeambreakSensor beam3, break3; // slot 3 — front (last ball, closest to shooter)

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
        intakePivot2 = hardwareMap.get(Servo.class, "intakePivot2"); // match config name

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");

        // Config names must match robot config file exactly
        beam1  = new BeambreakSensor(hardwareMap, "beam1");
        break1 = new BeambreakSensor(hardwareMap, "break1");
        beam2  = new BeambreakSensor(hardwareMap, "beam2");
        break2 = new BeambreakSensor(hardwareMap, "break2");
        beam3  = new BeambreakSensor(hardwareMap, "beam3");
        break3 = new BeambreakSensor(hardwareMap, "break3");

        setIdle();
    }

    // ── State control ─────────────────────────────────────────

    public void setState(IntakeState state) {
        currentState = state;
        switch (state) {
            case IDLE:
                intakePivot.setPosition(PIVOT_RETRACTED);
                intakePivot2.setPosition(1-PIVOT_RETRACTED); // or (1.0 - x) if it's reversed
                intakeMotor.setPower(0);
                break;
            case INTAKING:
                intakePivot.setPosition(Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
                intakePivot2.setPosition(1-Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
                intakeMotor.setPower(INTAKE_POWER);
                break;
            case OUTTAKING:
                intakePivot.setPosition(Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
                intakePivot2.setPosition(1-Math.max(PIVOT_FLOOR, PIVOT_DEPLOYED));
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

    // ── Ball detection ────────────────────────────────────────
    // A slot is occupied if either pin of its sensor pair is blocked

    public boolean isSlot1Present() { return beam1.isBlocked() || break1.isBlocked(); } // top
    public boolean isSlot2Present() { return beam2.isBlocked() || break2.isBlocked(); } // middle
    public boolean isSlot3Present() { return beam3.isBlocked() || break3.isBlocked(); } // front

    public boolean isFullyLoaded() {
        return isSlot1Present() && isSlot2Present() && isSlot3Present();
    }

    public boolean isEmpty() {
        return !isSlot1Present() && !isSlot2Present() && !isSlot3Present();
    }

    public int getBallCount() {
        return (isSlot1Present() ? 1 : 0)
                + (isSlot2Present() ? 1 : 0)
                + (isSlot3Present() ? 1 : 0);
    }

    /**
     * True when front slot is clear — ball has left toward shooter.
     * Used by shooting sequence to confirm a ball has fired.
     */
    public boolean isFrontBeamClear() { return !isSlot3Present(); }

    // ── Getters ───────────────────────────────────────────────

    public IntakeState getState()   { return currentState; }
    public boolean     isIntaking() { return currentState == IntakeState.INTAKING; }

    // ── Telemetry ─────────────────────────────────────────────

    public void setTelemetry(Telemetry telem) { this.telemetry = telem; }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Intake State",   currentState);
        telemetry.addData("Ball Count",     getBallCount() + " / 3");
        telemetry.addData("Slot 1 (top)",   isSlot1Present() ? "BALL ●" : "empty ○");
        telemetry.addData("Slot 2 (mid)",   isSlot2Present() ? "BALL ●" : "empty ○");
        telemetry.addData("Slot 3 (front)", isSlot3Present() ? "BALL ●" : "empty ○");
        telemetry.addData("Pivot Position", "%.2f", intakePivot.getPosition());
        telemetry.addData("Motor Power",    "%.2f", intakeMotor.getPower());
    }
}
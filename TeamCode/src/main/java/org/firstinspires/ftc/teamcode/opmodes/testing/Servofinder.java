package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * SERVO POSITION FINDER
 *
 * Controls servos independently. intakePivot2 always mirrors intakePivot (1 - x).
 *
 * GAMEPAD 1:
 *   LB / RB       — cycle active servo
 *   Dpad UP/DOWN  — nudge +/-0.01
 *   Dpad L/R      — nudge +/-0.001 (fine)
 *   Y = 0.0   X = 0.5   A = 1.0
 */
@TeleOp(name = "TEST: Servo Finder", group = "testing")
public class Servofinder extends OpMode {

    private Servo servoPort0;   // hood
    private Servo servoPort1;   // latch
    private Servo intakePivot;
    private Servo intakePivot2;

    private static final String NAME_PORT0  = "hood";
    private static final String NAME_PORT1  = "latch";
    private static final String NAME_PIVOT  = "intakePivot";
    private static final String NAME_PIVOT2 = "intakePivot2"; // TODO: match config name

    private int activeServoIndex = 0;
    private final String[] servoNames = {"Hood", "Latch", "Intake Pivot"};

    private static final double NUDGE_COARSE = 0.01;
    private static final double NUDGE_FINE   = 0.001;

    private boolean lbWasPressed     = false;
    private boolean rbWasPressed     = false;
    private boolean dUpWasPressed    = false;
    private boolean dDownWasPressed  = false;
    private boolean dLeftWasPressed  = false;
    private boolean dRightWasPressed = false;
    private boolean yWasPressed      = false;
    private boolean aWasPressed      = false;
    private boolean xWasPressed      = false;

    @Override
    public void init() {
        servoPort0   = hardwareMap.get(Servo.class, NAME_PORT0);
        servoPort1   = hardwareMap.get(Servo.class, NAME_PORT1);
        intakePivot  = hardwareMap.get(Servo.class, NAME_PIVOT);
        intakePivot2 = hardwareMap.get(Servo.class, NAME_PIVOT2);

        servoPort0.setPosition(0.5);
        servoPort1.setPosition(0.5);
        intakePivot.setPosition(0.5);
        intakePivot2.setPosition(0.5); // mirrored: 1 - 0.5 = 0.5

        telemetry.addLine("Servo Finder ready.");
        telemetry.addLine("intakePivot2 always mirrors intakePivot (1 - x)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ── Servo selection ────────────────────────────────────
        boolean lbNow = gamepad1.left_bumper;
        boolean rbNow = gamepad1.right_bumper;
        if (lbNow && !lbWasPressed) activeServoIndex = (activeServoIndex + 2) % 3;
        if (rbNow && !rbWasPressed) activeServoIndex = (activeServoIndex + 1) % 3;
        lbWasPressed = lbNow;
        rbWasPressed = rbNow;

        Servo  active = getActiveServo();
        double pos    = active.getPosition();

        // ── Coarse nudge ───────────────────────────────────────
        boolean dUpNow   = gamepad1.dpad_up;
        boolean dDownNow = gamepad1.dpad_down;
        if (dUpNow   && !dUpWasPressed)   pos += NUDGE_COARSE;
        if (dDownNow && !dDownWasPressed) pos -= NUDGE_COARSE;
        dUpWasPressed   = dUpNow;
        dDownWasPressed = dDownNow;

        // ── Fine nudge ─────────────────────────────────────────
        boolean dLeftNow  = gamepad1.dpad_left;
        boolean dRightNow = gamepad1.dpad_right;
        if (dRightNow && !dRightWasPressed) pos += NUDGE_FINE;
        if (dLeftNow  && !dLeftWasPressed)  pos -= NUDGE_FINE;
        dLeftWasPressed  = dLeftNow;
        dRightWasPressed = dRightNow;

        // ── Presets ────────────────────────────────────────────
        boolean yNow = gamepad1.y;
        boolean aNow = gamepad1.a;
        boolean xNow = gamepad1.x;
        if (yNow && !yWasPressed) pos = 0.0;
        if (aNow && !aWasPressed) pos = 1.0;
        if (xNow && !xWasPressed) pos = 0.5;
        yWasPressed = yNow;
        aWasPressed = aNow;
        xWasPressed = xNow;

        pos = Math.max(0.0, Math.min(1.0, pos));
        active.setPosition(pos);

        // ── Mirror intakePivot2 whenever pivot is active ───────
        if (activeServoIndex == 2) {
            intakePivot2.setPosition(1.0 - pos);
        }

        // ── Telemetry ──────────────────────────────────────────
        telemetry.addLine("=== ACTIVE SERVO ===");
        telemetry.addData("Selected", servoNames[activeServoIndex]);
        telemetry.addData("Position", "%.3f", pos);
        telemetry.addLine();
        telemetry.addLine("=== ALL SERVOS ===");
        telemetry.addData("Hood",          "%.3f  %s", servoPort0.getPosition(),
                activeServoIndex == 0 ? "<< ACTIVE" : "");
        telemetry.addData("Latch",         "%.3f  %s", servoPort1.getPosition(),
                activeServoIndex == 1 ? "<< ACTIVE" : "");
        telemetry.addData("Intake Pivot",  "%.3f  %s", intakePivot.getPosition(),
                activeServoIndex == 2 ? "<< ACTIVE" : "");
        telemetry.addData("Intake Pivot2", "%.3f  (mirrored)", intakePivot2.getPosition());
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("LB/RB = change servo");
        telemetry.addLine("DPAD U/D = +/-0.01  L/R = +/-0.001");
        telemetry.addLine("Y=0.0  X=0.5  A=1.0");
        telemetry.update();
    }

    private Servo getActiveServo() {
        switch (activeServoIndex) {
            case 0:  return servoPort0;
            case 1:  return servoPort1;
            default: return intakePivot;
        }
    }
}
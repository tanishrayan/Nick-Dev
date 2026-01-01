package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Zero", group="Testing")
public class ServoZero extends LinearOpMode {

    // Declare all servos
    private Servo intakePivotLeft;
    private Servo intakePivotRight;
    private Servo transferRamp;
    private Servo turretLeft;
    private Servo turretRight;
    private Servo hood;

    @Override
    public void runOpMode() {
        // Initialize all servos from hardware map
        intakePivotLeft = hardwareMap.get(Servo.class, "intakePivotLeft");
        intakePivotRight = hardwareMap.get(Servo.class, "intakePivotRight");
        transferRamp = hardwareMap.get(Servo.class, "transferRamp");
        turretLeft = hardwareMap.get(Servo.class, "turretLeft");
        turretRight = hardwareMap.get(Servo.class, "turretRight");
        hood = hardwareMap.get(Servo.class, "hood");

        telemetry.addLine("Press PLAY to zero all servos");
        telemetry.addLine("WARNING: Make sure robot is in a safe position!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Set all servos to 0.0 position
            intakePivotLeft.setPosition(0.0);
            intakePivotRight.setPosition(0.0);
            transferRamp.setPosition(0.0);
            turretLeft.setPosition(0.0);
            turretRight.setPosition(0.0);
            hood.setPosition(0.0);

            telemetry.addLine("✓ All servos zeroed to 0.0");
            telemetry.addData("Intake Pivot Left", intakePivotLeft.getPosition());
            telemetry.addData("Intake Pivot Right", intakePivotRight.getPosition());
            telemetry.addData("Transfer Ramp", transferRamp.getPosition());
            telemetry.addData("Turret Left", turretLeft.getPosition());
            telemetry.addData("Turret Right", turretRight.getPosition());
            telemetry.addData("Hood", hood.getPosition());
            telemetry.update();

            // Keep the program running so servos hold position
            while (opModeIsActive()) {
                idle();
            }
        }
    }
}
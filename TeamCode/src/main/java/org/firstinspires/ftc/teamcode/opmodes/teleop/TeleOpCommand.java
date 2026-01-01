/*package org.firstinspires.ftc.teamcode.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Slot Test (Cmd-Based)", group = "Test")
public class TeleOpCommand extends CommandOpMode {

    private GamepadEx g1;
    private DcMotor motor1, frontRight;

    @Override
    public void initialize() {
        motor1     = hardwareMap.get(DcMotor.class, "slideRight"); // make sure names match config
        frontRight = hardwareMap.get(DcMotor.class, "backRight");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        g1 = new GamepadEx(gamepad1);

        // While RB held: run motors at 0.5; when released: stop
        new GamepadButton(g1, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(() -> {
                    motor1.setPower(1);
                    frontRight.setPower(1);
                }))
                .whenReleased(new InstantCommand(() -> {
                    motor1.setPower(0);
                    frontRight.setPower(0);
                }));

        // LB acts as an immediate STOP button too
        new GamepadButton(g1, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    motor1.setPower(0);
                    frontRight.setPower(0);
                }));

        // Keep telemetry updating every loop
        schedule(new RunCommand(() -> {
            telemetry.addData("RB held", gamepad1.right_bumper);
            telemetry.addData("LB", gamepad1.left_bumper);
            telemetry.addData("Pwr motor1/backRight", "%.2f / %.2f",
                    motor1.getPower(), frontRight.getPower());
            telemetry.update();
        }));

        telemetry.addLine("Ready: RB = run (0.5), LB = stop");
        telemetry.update();
    }
}
*/
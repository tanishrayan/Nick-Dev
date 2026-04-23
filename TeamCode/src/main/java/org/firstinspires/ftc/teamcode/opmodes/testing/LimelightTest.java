package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Limelight Test", group = "Testing")
public class LimelightTest extends OpMode {

    private Limelight3A limelight;
    private int currentPipeline = 0;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void init() {
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        try {
            // Initialize Limelight - make sure this name matches your config!
            limelight = hardwareMap.get(Limelight3A.class, "limelight");

            // Set telemetry update speed
            telemetry.setMsTransmissionInterval(11);

            // Start on pipeline 0
            limelight.pipelineSwitch(0);

            // Start polling for data
            limelight.start();

            telemetry.addLine("✓ Limelight initialized successfully!");
            telemetry.addLine("");
            telemetry.addLine("Controls:");
            telemetry.addLine("  DPAD UP/DOWN = Change pipeline");
            telemetry.addLine("  A = Restart Limelight");

        } catch (Exception e) {
            telemetry.addLine("✗ ERROR: Could not find Limelight!");
            telemetry.addLine("");
            telemetry.addLine("Check that:");
            telemetry.addLine("  1. Limelight is plugged into USB 3.0 (blue port)");
            telemetry.addLine("  2. Config has 'limelight' device");
            telemetry.addLine("  3. Green light is blinking on Limelight");
            telemetry.addLine("");
            telemetry.addLine("Error: " + e.getMessage());
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        if (limelight == null) {
            telemetry.addLine("✗ Limelight not connected!");
            telemetry.addLine("Please check configuration and restart.");
            telemetry.update();
            return;
        }

        // ===== CONTROLS =====

        // Change pipeline with DPAD
        if (gamepad1.dpad_up && !dpadUpPressed) {
            currentPipeline = Math.min(9, currentPipeline + 1);
            limelight.pipelineSwitch(currentPipeline);
            dpadUpPressed = true;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        if (gamepad1.dpad_down && !dpadDownPressed) {
            currentPipeline = Math.max(0, currentPipeline - 1);
            limelight.pipelineSwitch(currentPipeline);
            dpadDownPressed = true;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // Restart Limelight with A button
        if (gamepad1.a) {
            limelight.stop();
            limelight.start();
            telemetry.addLine(">>> Limelight restarted <<<");
        }

        // ===== GET LIMELIGHT DATA =====
        LLResult result = limelight.getLatestResult();

        // ===== TELEMETRY =====
        telemetry.addLine("========== LIMELIGHT TEST ==========");
        telemetry.addLine("");

        telemetry.addData("Current Pipeline", currentPipeline);
        telemetry.addLine("");

        if (result != null) {
            telemetry.addData("Result Valid", result.isValid() ? "YES ✓" : "NO ✗");
            telemetry.addLine("");

            if (result.isValid()) {
                telemetry.addLine("--- TARGET DATA ---");
                telemetry.addData("TX (horizontal offset)", "%.2f°", result.getTx());
                telemetry.addData("TY (vertical offset)", "%.2f°", result.getTy());
                telemetry.addData("TA (target area)", "%.2f%%", result.getTa());

                telemetry.addLine("");
                telemetry.addLine("--- INTERPRETATION ---");

                double tx = result.getTx();
                if (Math.abs(tx) < 2.0) {
                    telemetry.addLine("Horizontal: CENTERED ✓");
                } else if (tx > 0) {
                    telemetry.addLine("Horizontal: Target is RIGHT →");
                } else {
                    telemetry.addLine("Horizontal: Target is LEFT ←");
                }

                // If using AprilTags, show botpose
                if (result.getBotpose() != null) {
                    telemetry.addLine("");
                    telemetry.addLine("--- BOTPOSE (if AprilTags) ---");
                    telemetry.addData("Botpose", result.getBotpose().toString());
                }

            } else {
                telemetry.addLine("No valid target detected.");
                telemetry.addLine("Point camera at target or check pipeline.");
            }

            telemetry.addLine("");
            telemetry.addData("Latency", "%.0f ms", result.getStaleness());

        } else {
            telemetry.addLine("✗ No result from Limelight");
            telemetry.addLine("Camera may still be starting up...");
        }

        telemetry.addLine("");
        telemetry.addLine("========== CONTROLS ==========");
        telemetry.addLine("DPAD UP/DOWN = Change pipeline");
        telemetry.addLine("A = Restart Limelight");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}
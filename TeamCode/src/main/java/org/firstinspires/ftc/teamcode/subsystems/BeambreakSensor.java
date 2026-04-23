package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Wrapper around a single REV digital channel acting as a break beam sensor.
 * REV break beam: false = beam broken = ball present
 *                 true  = beam clear  = no ball
 */
public class BeambreakSensor {

    private DigitalChannel channel;
    private final String   configName;
    private boolean        connected = false;
    private Telemetry      telemetry;

    public BeambreakSensor(HardwareMap hardwareMap, String configName) {
        this.configName = configName;
        try {
            channel = hardwareMap.get(DigitalChannel.class, configName);
            channel.setMode(DigitalChannel.Mode.INPUT);
            connected = true;
        } catch (Exception e) {
            connected = false;
        }
    }

    /**
     * Returns true when the beam is broken (ball is present).
     * Returns false if sensor is not connected.
     */
    public boolean isBlocked() {
        if (!connected || channel == null) return false;
        return !channel.getState(); // REV: false = blocked
    }

    public boolean isConnected() { return connected; }

    public String getConfigName() { return configName; }

    public void setTelemetry(Telemetry telem) { this.telemetry = telem; }

    public void updateTelemetry() {
        if (telemetry == null) return;
        telemetry.addData(configName, connected ? (isBlocked() ? "BALL ●" : "clear ○") : "NOT FOUND");
    }
}
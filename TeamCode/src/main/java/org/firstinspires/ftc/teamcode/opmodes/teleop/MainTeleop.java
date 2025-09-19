package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;


public class MainTeleop extends OpMode {

    @Override
    public void init() {
        TelemetryUtils.initialize(telemetry); // pass the OpMode telemetry
    }

    @Override
    public void loop() {
    TelemetryUtils.update();
    }
}

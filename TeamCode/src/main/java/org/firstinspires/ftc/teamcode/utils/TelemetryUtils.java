package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class TelemetryUtils {
    public static double LOW_BATTERY_THRESHOLD = 12.0;

    private static Telemetry dsTelemetry = null;
    private static Telemetry dashboardTelemetry = null;

    public static void initialize(Telemetry opModeTelemetry) {
        dsTelemetry = opModeTelemetry;
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
    }

    public static void addData(String key, Object value) {
        if (dsTelemetry != null) dsTelemetry.addData(key, value);
        if (dashboardTelemetry != null) dashboardTelemetry.addData(key, value);
    }

    public static void logVoltage(HardwareMap hwMap){
        double voltage = hwMap.voltageSensor.iterator().next().getVoltage();
        if (dsTelemetry != null) dsTelemetry.addData("Battery Voltage", voltage);
        if (dashboardTelemetry != null) dashboardTelemetry.addData("Battery Voltage", voltage);
        if (voltage < LOW_BATTERY_THRESHOLD) {
            if (dsTelemetry != null) dsTelemetry.addLine("⚠️ LOW BATTERY ⚠️");
            if (dashboardTelemetry != null) dashboardTelemetry.addLine("⚠️ LOW BATTERY ⚠️");
        }
    }


    public static void update() {
        if (dsTelemetry != null) dsTelemetry.update();
        if (dashboardTelemetry != null) dashboardTelemetry.update();
    }
}

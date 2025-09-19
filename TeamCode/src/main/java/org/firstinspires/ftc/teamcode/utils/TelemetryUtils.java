package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUtils {

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

    public static void update() {
        if (dsTelemetry != null) dsTelemetry.update();
        if (dashboardTelemetry != null) dashboardTelemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.NewDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;


@TeleOp(name = "drive subsystem")
public class MainTeleop extends OpMode {

    NewDriveSubsystem drive_base = new NewDriveSubsystem();

    @Override
    public void init() {
        TelemetryUtils.initialize(telemetry); // pass the OpMode telemetry
        drive_base.initialize(hardwareMap);
    }

    @Override
    public void loop() {
    TelemetryUtils.update();
    TelemetryUtils.logVoltage(hardwareMap);
        drive_base.drive(gamepad1);
    }
}

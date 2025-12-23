package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;


@TeleOp(name = "Main Teleop")
public class MainTeleop extends OpMode {

    // Using the Robot container instead of individual subsystems
    Robot robot = new Robot();

    @Override
    public void init() {
        // Initializes everything: Subsystems, Hubs, and Telemetry
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        TelemetryUtils.update();
        //  Always clear bulk cache at the start of each loop
        robot.clearCache();

        //Handle Subsystem Logic
        robot.drive.updateOdo();
        robot.drive.gamepadDrive(gamepad1);

        // Logic for auto-alignment
        if (gamepad1.a) {
            robot.drive.alignHeadingToAprilTag(1.0);
        }

        // Reset heading logic
        if (gamepad1.options) {
            robot.drive.resetHeading();
        }

        //Update Telemetry
        TelemetryUtils.update();
    }
}

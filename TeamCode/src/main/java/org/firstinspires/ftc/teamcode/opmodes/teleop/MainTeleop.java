package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;


@TeleOp(name = "Main Teleop")
public class MainTeleop extends OpMode {
    private ElapsedTime loopTimer = new ElapsedTime();
    // Using the Robot container instead of individual subsystems
    Robot robot = new Robot();

    @Override
    public void init() {
        // Initializes everything: Subsystems, Hubs, and Telemetry
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        loopTimer.reset();
    }


    @Override
    public void loop() {

        // Refresh Data
        robot.clearCache();
        robot.drive.updateOdo();
        robot.drive.logMotorCurrent();



        // Control Logic

        // Logic for auto-alignment
        if (gamepad1.a) {
            robot.drive.alignHeadingToAprilTag(1.0);
        } else {
            // Normal driving
            robot.drive.gamepadDrive(gamepad1);
        }

        if (gamepad1.x) {
            robot.drive.switchOrientation();
        }
        if (gamepad2.right_bumper){
            robot.intake.MoveIntakeMotor();
        }
        // Reset heading logic
        if (gamepad1.start) {
            robot.drive.resetHeading();
        }

        // Telemetry
        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        TelemetryUtils.addData("Loop Hz", 1000.0 / loopTime);
        TelemetryUtils.update(); // Move to bottom
    }
}

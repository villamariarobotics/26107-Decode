package org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        // 1. Refresh Data & Drive (Runs every loop)
        robot.clearCache();
        robot.drive.updateOdo();

        if (gamepad1.a) {
            robot.drive.alignHeadingToAprilTag(1.0);
        } else {
            robot.drive.gamepadDrive(gamepad1);
        }

        if (gamepad1.xWasPressed()) {
            robot.drive.switchOrientation();
        }

        // 2. Transfer Logic (Independent)
        if (gamepad2.b) {
            robot.intake.runTransfer();
        } else if (gamepad2.x) {
            robot.intake.retractTransfer();
        } else {
            robot.intake.stopTransfer(); // Stop if B is released
        }

        // 3. Intake Logic (Independent)
        if (gamepad2.a) {
            robot.intake.runIntake();
        } else if (gamepad2.left_bumper) {
            robot.intake.retractIntake();
        } else {
            robot.intake.stopIntake(); // Stop if A is released
        }

        // 4. Shooter Logic (Independent)
        if (gamepad2.y) {
            robot.shooter.runShooter(2000, true);
        } else if (gamepad2.right_bumper) {
            robot.shooter.runShooter(1000, false);
        } else {
            robot.shooter.runShooter(0, true); // Stop if Y is released
        }
        // Modulate shooter speed with direction pad
        if (gamepad2.dpad_down) {
            robot.shooter.speed -= 0.0025;
            if (robot.shooter.speed <= 0) {
                robot.shooter.speed = 0;
            }
        } else if (gamepad2.dpad_up) {
            robot.shooter.speed += 0.0025;
            if (robot.shooter.speed >= 1) {
                robot.shooter.speed = 1;
            }
        }

        // 5. Utility Logic
        if (gamepad1.start) {
            robot.drive.resetHeading();
        }
        robot.drive.acceleration = gamepad1.right_bumper;

        // 6. Telemetry (Moves the "Finish Line" to the end)
        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        TelemetryUtils.addData("Loop Hz", 1000.0 / loopTime);
        TelemetryUtils.addData("speed", robot.shooter.speed);
        TelemetryUtils.update();
    }

}

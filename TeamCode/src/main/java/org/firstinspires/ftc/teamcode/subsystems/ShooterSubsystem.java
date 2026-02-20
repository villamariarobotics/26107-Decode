package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

public class ShooterSubsystem {
    private DcMotorEx shooterMotor;
    private PIDController velocityPID;
    private final double TICKS_PER_ROTATION = 28;

    public static double kP = 0.04, kI = 0.0, kD = 0.0005;

    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        shooterMotor = hwMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        velocityPID = new PIDController(kP, kI, kD);
        velocityPID.setOutputMax(1);
    }

    public void runShooter(double RPM) {
        velocityPID.setGains(kP, kI, kD);  // Update PID gains live from Dashboard
        double targetVelocity = RPM * TICKS_PER_ROTATION;
        double currentVelocity = shooterMotor.getVelocity() / TICKS_PER_ROTATION;

        double power = velocityPID.output(targetVelocity, currentVelocity);
        shooterMotor.setPower(power);
        TelemetryUtils.addData("TurretTargetVel", targetVelocity);
        TelemetryUtils.addData("TurretVelocity", currentVelocity);
        TelemetryUtils.addData("TurretPower", power);
    }


}

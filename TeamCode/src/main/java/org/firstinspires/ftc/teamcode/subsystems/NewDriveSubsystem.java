package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDController;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Config
public class NewDriveSubsystem {
    public static boolean FieldOriented = true;

    private DcMotor fl, fr, bl,br;
    IMU imu;
    private Limelight3A limelight;
    private PIDController headingPID;
    // PID coefficients (for heading hold)
    public static double kP = 0.05;
    public static double kI = 0.0001;
    public static double kD = 0.005;
    private static final double ROT_TOLERANCE_DEG = 1.0;
    private static final double MIN_ROT_POWER = 0.05;

//    private double targetHeading = 0.0; // radians

    public void initialize(HardwareMap hwMap) {
        fl = hwMap.get(DcMotorEx.class, "left_front_motor");
        fr = hwMap.get(DcMotorEx.class, "right_front_motor");
        bl = hwMap.get(DcMotorEx.class, "left_back_motor");
        br = hwMap.get(DcMotorEx.class, "right_back_motor");

        // Initialize the heading PID controller
        headingPID = new PIDController(kP, kI, kD, true);
        headingPID.setOutputLimits(-1, 1);
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Disable built-in velocity control
        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hwMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
    }


    public boolean alignHeadingToAprilTag (double maxMotorPower){
        // Ensure PID gains are updated for FTC Dashboard
        headingPID.setGains(kP, kI, kD);
        LLResult result = limelight.getLatestResult();


        // --- 0. Check for Target Visibility ---
        if (!result.isValid()) {

            headingPID.reset();
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
            TelemetryUtils.addData("Alignment Status", "TARGET NOT FOUND");
            return false;
        }
        double rotationErrorDegrees = result.getTx();

        //  Rotation (Yaw) Control: PID Calculation
        double rotPower = headingPID.calculate(rotationErrorDegrees);
        // Translation Powers (0 because only aligning rotationally for now)
        double forwardPower = 0.0;
        double strafePower = 0.0;
        boolean alignedRotation = Math.abs(rotationErrorDegrees) < ROT_TOLERANCE_DEG;

        if (alignedRotation) {
            // Alignment is complete: STOP
            rotPower = 0.0;
            headingPID.reset();
            TelemetryUtils.addData("Alignment Status", "ALIGNED");
        } else {
            // Still aligning: APPLY PID POWER
            // Clamp rotation power to the maximum inputted to alignment function
            rotPower = Math.min(Math.max(rotPower, -maxMotorPower), maxMotorPower);
            // Apply Deadband (minimum speed to overcome static friction)
            if (Math.abs(rotPower) < MIN_ROT_POWER) {
                rotPower = Math.signum(rotPower) * MIN_ROT_POWER;
            }
            TelemetryUtils.addData("Alignment Status", "ALIGNING");
        }
        double y = forwardPower;
        double x = strafePower;
        double rx = rotPower;
        // denominator not really needed cause its only rotation but like ¯\_(ツ)_/¯
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double left_front_power  = (y + x + rx) / denominator;
        double right_front_power = (y - x - rx) / denominator;
        double left_back_power   = (y - x + rx) / denominator;
        double right_back_power  = (y + x - rx) / denominator;

        fl.setPower(left_front_power);
        fr.setPower(right_front_power);
        bl.setPower(left_back_power);
        br.setPower(right_back_power);

        TelemetryUtils.addData("Limelight tx Error", rotationErrorDegrees);
        TelemetryUtils.addData("LL Rotational Power (rx)", rotPower);

        return alignedRotation;
    }

    public void drive (Gamepad controller){

        if (FieldOriented) {
            double y = -controller.left_stick_y;
            double x = controller.left_stick_x;
            double rx = controller.right_stick_x;


            double botHeading = getHeading();

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotY) + Math.abs(rx), 1);
            double left_front_power = (rotY + rotX + rx) / denominator;
            double right_front_power = (rotY - rotX - rx) / denominator;
            double left_back_power = (rotY - rotX + rx) / denominator;
            double right_back_power = (rotY + rotX - rx) / denominator;


            TelemetryUtils.addData("Front Left Power", left_front_power);
            TelemetryUtils.addData("Front Right Power", right_front_power);
            TelemetryUtils.addData("Back Left Power", left_back_power);
            TelemetryUtils.addData("Back Right Power", right_back_power);

            fl.setPower(left_front_power);
            fr.setPower(right_front_power);
            bl.setPower(left_back_power);
            br.setPower(right_back_power);
        } else {
            double drive  = controller.left_stick_y;
            double strafe = -controller.left_stick_x;
            double twist  = -controller.right_stick_x;
            double left_front_power =  (drive + strafe + twist);
            double right_front_power = (drive - strafe - twist);
            double left_back_power = (drive - strafe + twist);
            double right_back_power = (drive + strafe - twist);
            fl.setPower(left_front_power);
            fr.setPower(right_front_power);
            bl.setPower(left_back_power);
            br.setPower(right_back_power);
        }

    }
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading(){
        imu.resetYaw();
    }

}
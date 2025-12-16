package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDController;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
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

    private double targetHeading = 0.0; // radians

    public void initialize(HardwareMap hwMap) {
        fl = hwMap.get(DcMotorEx.class, "left_front_motor");
        fr = hwMap.get(DcMotorEx.class, "right_front_motor");
        bl = hwMap.get(DcMotorEx.class, "left_back_motor");
        br = hwMap.get(DcMotorEx.class, "right_back_motor");

        // Initialize the heading PID controller
        headingPID = new PIDController(kP, kI, kD, true);
        headingPID.setOutputLimits(-1, 1);
        targetHeading = 0.0;

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


    public double LimelightFunc (){

        LLStatus status = limelight.getStatus();
        TelemetryUtils.addData("LL name:", status.getName());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            double txResult = result.getTx();
            return Math.toRadians(-txResult);

//            TelemetryUtils.addData("LL Latency", captureLatency + targetingLatency);
//            TelemetryUtils.addData("Parse Latency", parseLatency);
//            TelemetryUtils.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
        }
        return 0;



    }
        public void drive (Gamepad controller){

            // Update PID gains from dashboard live
//            headingPID.setGains(kP, kI, kD);

        if (FieldOriented) {

            double targetHeading = LimelightFunc();
            double y = 0;
            double x = 0;
            double rx = headingPID.output(targetHeading, 0);
            TelemetryUtils.addData("target heading", targetHeading);
            TelemetryUtils.addData("tx", rx);


            if (controller.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotY) + Math.abs(rx), 1);


            double left_front_power = (rotY + rotX + rx) / denominator;
            double right_front_power = (rotY - rotX - rx) / denominator;
            double left_back_power = (rotY - rotX + rx) / denominator;
            double right_back_power = (rotY + rotX - rx) / denominator;

            if (rx == 0){

                 left_front_power = 0;
                 right_front_power = 0;
                 left_back_power = 0;
                 right_back_power = 0;
            }


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

}
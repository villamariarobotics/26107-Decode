package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;import org.firstinspires.ftc.teamcode.utils.PIDController;

@Config

public class NewDriveSubsystem {
    private DcMotor fl, fr, bl,br;

    public void initialize(HardwareMap hwMap) {
        fl = hwMap.get(DcMotorEx.class, "left_front_motor");
        fr = hwMap.get(DcMotorEx.class, "right_front_motor");
        bl = hwMap.get(DcMotorEx.class, "left_back_motor");
        br = hwMap.get(DcMotorEx.class, "right_back_motor");

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(Gamepad controller){

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
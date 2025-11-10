package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    }

    public void drive(Gamepad controller){

        double leftstick = controller.left_stick_y;
        double rightstick = controller.right_stick_y;

        fl.setPower(leftstick);
        fr.setPower(rightstick);
        bl.setPower(leftstick);
        br.setPower(rightstick);
    }


}
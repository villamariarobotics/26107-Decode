package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {
    private DcMotor intakeMotor ;
    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void MoveIntakeMotor(){
        intakeMotor.setPower(1);
    }
}

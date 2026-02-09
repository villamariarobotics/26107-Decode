package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeSubsystem {
    private DcMotor intakeMotor ;
    private DcMotor outtakeMotor ;

    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor = hwMap.get(DcMotor.class, "outtakeMotor");
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void MoveIntakeMotor(){
        intakeMotor.setPower(1);
    }
    public void MoveOuttakeMotor(double y){
        outtakeMotor.setPower(y);
    }

}


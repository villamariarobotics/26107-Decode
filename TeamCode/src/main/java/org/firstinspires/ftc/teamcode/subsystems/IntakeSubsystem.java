package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    private DcMotor intakeMotor ;
    private DcMotor outtakeMotor ;

    private CRServo beltServo;

    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor = hwMap.get(DcMotor.class, "outtakeMotor");
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        beltServo = hwMap.get(CRServo.class, "beltServo");
    }
    public void MoveIntakeMotor(){
        intakeMotor.setPower(1);
    }
    public void MoveOuttakeMotor(double y){
        outtakeMotor.setPower(y);
    }

    public void MoveBeltServo(double power){
        beltServo.setPower(power);
    }

}


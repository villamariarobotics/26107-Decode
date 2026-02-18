package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    private DcMotor intakeMotor ;
    private DcMotor outtakeMotor ;

    private CRServo beltServo1;
    private CRServo beltServo2;
    private Boolean servoRightWay = true;

    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor = hwMap.get(DcMotor.class, "outtakeMotor");
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        beltServo1 = hwMap.get(CRServo.class, "beltServo1");
        beltServo2 = hwMap.get(CRServo.class, "beltServo2");
        beltServo2.setDirection(CRServo.Direction.REVERSE);
    }
    public void MoveIntakeMotor(){
        intakeMotor.setPower(1);
    }
    public void MoveOuttakeMotor(double y){
        outtakeMotor.setPower(y);
    }

    public void MoveBeltServo(double power){
        if (servoRightWay) {
            beltServo1.setPower(power);
            beltServo2.setPower(power);
        } else {
            beltServo1.setPower(-power);
            beltServo2.setPower(-power);
        }
    }

}


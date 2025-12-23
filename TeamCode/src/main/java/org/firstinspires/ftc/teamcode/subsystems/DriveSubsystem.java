package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.LimelightUtils;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Configurable
public class DriveSubsystem {

    // Hardware Variables
    private GoBildaPinpointDriver odo;
    private DcMotor fl, fr, bl, br;
    private Limelight3A limelight;
    private PIDController headingPID;

    // Configuration Constants (Static for @Configurable)
    @Sorter(sort = 0)
    public static boolean FieldOriented = true;
    @Sorter(sort = 1)
    public static double kP = 0.04, kI = 0.0, kD = 0.0005;
    @Sorter(sort = 4)
    public static double ROT_TOLERANCE_DEG = 2;
    @Sorter(sort = 5)
    public static double MIN_ROT_POWER = 0.01;

    public void initialize(HardwareMap hwMap) {
        // Motor Setup
        fl = hwMap.get(DcMotorEx.class, "left_front_motor");
        fr = hwMap.get(DcMotorEx.class, "right_front_motor");
        bl = hwMap.get(DcMotorEx.class, "left_back_motor");
        br = hwMap.get(DcMotorEx.class, "right_back_motor");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        // Pinpoint Setup
        odo = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(44, 0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        //Limelight Setup
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // PID Utils (currently only for heading alignment)
        headingPID = new PIDController(kP, kI, kD, false);
        headingPID.setOutputLimits(-1, 1);
    }

    public void updateOdo() {
        odo.update();
        TelemetryUtils.addData("Odo Status", odo.getDeviceStatus());

        Pose2D pos = odo.getPosition();
        TelemetryUtils.addData("Odo x pos", pos.getX(DistanceUnit.MM));
        TelemetryUtils.addData("Odo y pos", pos.getY(DistanceUnit.MM));
        TelemetryUtils.addData("Odo heading", pos.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Aligns the robot's heading to an AprilTag using Limelight feedback.
     *
     * @param maxMotorPower The maximum motor power to apply for rotation.
     * @return true if aligned within tolerance, false otherwise.
     */

    public boolean alignHeadingToAprilTag(double maxMotorPower) {
        headingPID.setGains(kP, kI, kD);
        LLResult result = limelight.getLatestResult();

        double rotPower = LimelightUtils.calculateRotationPower(
                result, headingPID, maxMotorPower, MIN_ROT_POWER, ROT_TOLERANCE_DEG
        );

        boolean aligned = LimelightUtils.isAligned(result, ROT_TOLERANCE_DEG);
        TelemetryUtils.addData("Alignment Status", !result.isValid() ? "NO TARGET" : (aligned ? "ALIGNED" : "ALIGNING"));

        // Translation Powers = 0 because only aligning rotationally (for now)
        applyMotorPower(0, 0, rotPower);
        return aligned;
    }


    /**
     * Helper method to consolidate the motor power math
     */
    private void applyMotorPower(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        fl.setPower((y + x + rx) / denominator);
        fr.setPower((y - x - rx) / denominator);
        bl.setPower((y - x + rx) / denominator);
        br.setPower((y + x - rx) / denominator);
    }

    public void gamepadDrive(Gamepad controller) {

        // Added a tiny deadband to prevent robot "humming" when sticks aren't touched
        double x = (Math.abs(controller.left_stick_x) > 0.05) ? controller.left_stick_x : 0;
        double y = (Math.abs(controller.left_stick_y) > 0.05) ? -controller.left_stick_y : 0;
        double rx = (Math.abs(controller.right_stick_x) > 0.05) ? controller.right_stick_x : 0;

        if (FieldOriented) {
            double botHeading = getHeading();
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            // Slightly increase rotational power to mitigate imperfections in strafing
            applyMotorPower(rotY, rotX * 1.1, rx);
        } else {
            applyMotorPower(y, x, rx);
        }

    }

    public double getHeading() {
        // Get heading from pinpoint odometry in radians
        return odo.getPosition().getHeading(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        odo.resetPosAndIMU(); // Resets pinpoint heading to 0

    }
}
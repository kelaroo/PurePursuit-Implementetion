package org.firstinspires.ftc.teamcode.DriveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Motion.Point;

import java.util.Arrays;
import java.util.List;

public class Mecanum_Roti_Negre {
    public static final double WHEEL_RADIUS = 5;
    public static final double GEAR_RATIO = 0.66;
    public static final double TICKS_PER_REV = 1120;

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public List<DcMotor> driveMotors;

    public Mecanum_Roti_Negre(HardwareMap hw) {
        leftFront = hw.get(DcMotor.class, "left_front");
        leftBack = hw.get(DcMotor.class, "left_back");
        rightFront = hw.get(DcMotor.class, "right_front");
        rightBack = hw.get(DcMotor.class, "right_back");

        driveMotors = Arrays.asList(leftFront, leftBack, rightFront, rightBack);
        for(DcMotor motor: driveMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set motors to reverse
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
    }

    public static double encoderTicksToCM(double ticks) {
        return 2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS *ticks / TICKS_PER_REV;
    }

    public void setMotorPower(double LF, double LB, double RF, double RB) {
        leftFront.setPower(LF);
        leftBack.setPower(LB);
        rightFront.setPower(RF);
        rightBack.setPower(RB);
    }

    public void stopAllPower() {
        for(DcMotor motor: driveMotors) {
            motor.setPower(0);
        }
    }

    public void goToPoint(Point targetVector) {
        double angleToTarget = Math.atan2(targetVector.x, targetVector.y);
        double drive = Math.sin(angleToTarget);
        double strafe = Math.cos(angleToTarget);

        double LF = drive - strafe;
        double LB = drive + strafe;
        double RF = drive + strafe;
        double RB = drive - strafe;
    }
}

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriveTrains.Mecanum_Roti_Negre;
import org.firstinspires.ftc.teamcode.Motion.MotionProfiling;
import org.firstinspires.ftc.teamcode.Motion.Point;

@TeleOp(name = "LocalizationTest")
public class LocalizationTest extends OpMode {

    private Mecanum_Roti_Negre hw;
    private MotionProfiling motion;

    @Override
    public void init() {
        hw = new Mecanum_Roti_Negre(hardwareMap);
        motion = new MotionProfiling(new Point(0, 0), Math.toRadians(90));
        telemetry.addData("Status", "Initialized");
        resetDriveEncoders();
    }

    @Override
    public void loop() {
        motion.updatePosition(hw.leftFront.getCurrentPosition(),
                hw.leftBack.getCurrentPosition(),
                hw.rightFront.getCurrentPosition(),
                hw.rightBack.getCurrentPosition());
        displayCurrentPosition();

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double LF = drive - strafe + rotate;
        double LB = drive + strafe + rotate;
        double RF = drive + strafe - rotate;
        double RB = drive - strafe - rotate;

        hw.setMotorPower(LF, LB, RF, RB);
    }

    private void resetDriveEncoders() {
        for(DcMotor motor: hw.driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void displayCurrentPosition() {
        telemetry.addData("x", String.valueOf(motion.absolutePos.x));
        telemetry.addData("y", String.valueOf(motion.absolutePos.y));
        telemetry.addData("absAngle", String.valueOf(motion.absoluteAngle));
    }
}

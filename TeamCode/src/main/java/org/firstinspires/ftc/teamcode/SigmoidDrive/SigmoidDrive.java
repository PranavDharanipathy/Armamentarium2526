package org.firstinspires.ftc.teamcode.SigmoidDrive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GeneralUtil.BetterGamepad;
import org.firstinspires.ftc.teamcode.GeneralUtil.FastSigmoidCurve;

public class SigmoidDrive {

    public static float JOYSTICK_MINIMUM = 0.02f;

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private BetterGamepad controller1;

    private final FastSigmoidCurve curve = new FastSigmoidCurve(); // when coding the robot, you would use the FastSigmoidCurve from a constants class

    public void provideComponents(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack, BetterGamepad controller1) {

        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;

        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.controller1 = controller1;
    }

    public void update() {

        double forward = safeJoystick(-controller1.left_stick_y());
        double strafe = safeJoystick(controller1.left_stick_x());
        double rotation = safeJoystick(controller1.right_stick_x());

        double forwardSigmoid = Math.signum(forward) * curve.getOutput(Math.abs(forward));
        double strafeSigmoid = Math.signum(strafe) * curve.getOutput(Math.abs(strafe));
        double rotationSigmoid = Math.signum(rotation) * curve.getOutput(Math.abs(rotation));

        double lfPower = forwardSigmoid + strafeSigmoid + rotationSigmoid;
        double lbPower = forwardSigmoid - strafeSigmoid + rotationSigmoid;
        double rfPower = forwardSigmoid - strafeSigmoid - rotationSigmoid;
        double rbPower = forwardSigmoid + strafeSigmoid - rotationSigmoid;

        leftFront.setPower(lfPower);
        rightFront.setPower(rfPower);
        leftBack.setPower(lbPower);
        rightBack.setPower(rbPower);
    }

    private double safeJoystick(double value) { //does the deadbanding
        return Math.abs(value) > JOYSTICK_MINIMUM ? value : 0;
    }
}

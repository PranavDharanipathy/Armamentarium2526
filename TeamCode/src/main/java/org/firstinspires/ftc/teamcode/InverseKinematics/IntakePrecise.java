package org.firstinspires.ftc.teamcode.InverseKinematics;
import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.teamcode.pedroPathing.DriveConstants;


/// To be used for intaking small objects
public class IntakePrecise {
    Follower follower;

    /// The precise motor that accounts for driver error(It will correct by moving to the position right above the sample/object)
    DcMotorEx xMotor;
    DcMotorEx extendoLeft;
    DcMotorEx extendoRight;
    Gamepad gamepad;

    ServoImplEx armServo;

    ServoImplEx wristServo;

    /// The extendo will extend until it reaches this position
    public double extendoTargetPosition = 0;

    double ticksPerInch;


    /// The pose at which the IVK(Inverse Kinematics) system is aiming at to intake the sample/object
    Pose targetPose;

    /**
     * Creates a follower, and the extendo motors(the main motors, and the xy motors on the extendo wrist that pick up the sample).
     * @Creation: ONLY TO BE CREATED AT THE START OF THE OpMode(During init()). DO NOT CREATE IN MIDDLE OF LOOP().
     * @Functions: update(), intake(), extend(double targetPosition),  movePrecisely(double preciseMovementAngle).
     *
     * @Controls: right_bumper(Intake), left_bumper(Outtake).
     * 
     * @Authored: 3/18/2026
     */
    public IntakePrecise(@NonNull HardwareMap hardwareMap, Gamepad gamepad1, Pose startPose, Pose samplePose, double ticksPerInch) {
        /// Creates the follower used to gain the robot's pose data.
        follower = DriveConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
        this.ticksPerInch = ticksPerInch;

        /// Initializes all the motors and devices needed for intake
        xMotor = hardwareMap.get(DcMotorEx.class, "extendo_xMotor");
        extendoLeft = hardwareMap.get(DcMotorEx.class, "left_extendo");
        extendoRight = hardwareMap.get(DcMotorEx.class, "right_extendo");
        wristServo = hardwareMap.get(ServoImplEx.class, "wrist_servo");
        this.gamepad = gamepad1;
        this.targetPose = samplePose;
        armServo = hardwareMap.get(ServoImplEx.class, "arm_servo");


        /// Resetting motors for start of OpMode.

       /* xMotor.resetDeviceConfigurationForOpMode();
        extendoLeft.resetDeviceConfigurationForOpMode();
        extendoRight.resetDeviceConfigurationForOpMode();
        wristServo.resetDeviceConfigurationForOpMode(); */
        
        /// Setting the motors to go to the target position(set in Motor.setTargetPosition(target)).

       /* xMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendoLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendoRight.setMode(DcMotor.RunMode.RUN_TO_POSITION); */
    }
    /// Sets the extendo movement distance; the distance which both left_extendo and right_extendo will move to.
    public void extend(double targetPosition) {
            this.extendoTargetPosition = targetPosition * ticksPerInch;
    }

    /// If the requested Angle is beyond the limits of the servo's movement range, it will be true, and intake() will be stopped.
    boolean nullPositionRequested = false;

    /// Sets the precise movement angle; the angle at which the precise motor(xMotor) will move to in order to pick up the sample.
    public void movePrecisely(double preciseMovementAngle) {
        if (!nullPositionRequested) {
            
        this.preciseMovementAngle = preciseMovementAngle;
        }
        
    }



    public void moveArm() {
        ///  Un-Comment this code to use

        //armServo.setPosition(IVKConstants.armServoPivot);
    }
    double deltaX =0;
    double deltaY = 0;
    Pose currentPose;
    boolean isIntaking = false;
    double robotHeadingInDegrees;
    double robotHeadingInRadians;
    public void update() {

        follower.update();
        currentPose = follower.getPose();

        deltaX = (targetPose.getY()) - (currentPose.getY());
        deltaY = (targetPose.getX()) - (currentPose.getX());
        
        if (gamepad.right_bumper) {
            robotHeadingInRadians = currentPose.getHeading();

            
            isIntaking = true;
            robotHeadingInDegrees = FastMath.toDegrees(robotHeadingInRadians);

            robotHeadingInDegrees *= -1;
            robotHeadingInDegrees -= 90;
            intake();

        }
        if (isIntaking) {
            follower.holdPoint(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()));
        }

        if (gamepad.left_bumper) {
            extend(0.8);
            //TODO: Add CRServo activation to outtake element;
        }
    }
    /// The combined angle of the robot heading and the angle between the robot and the sample/object.
    double otherArea = 0;

    /// The angle of the object relative to the robot's x-axis.
    double objectAngle = 0;

    /// The Angle the precise motors(xMotor) needs to move in order to be right on top of the object(in degrees).
    double preciseMovementAngle = 0;


    
    public void intake() {

        objectAngle = FastMath.toDegrees(FastMath.atan2(deltaY, deltaX));
        otherArea = 90 - objectAngle;

        preciseMovementAngle = otherArea - robotHeadingInDegrees;

        if (preciseMovementAngle < IVKConstants.minPrecisePosition || preciseMovementAngle > IVKConstants.maxPrecisePosition) {
            nullPositionRequested = true;
            gamepad.rumble(1200);
            isIntaking = false;
        }
        else {
            extend(Math.hypot(deltaX, deltaY));
            movePrecisely(preciseMovementAngle);

            isIntaking = false;
        }
    }
}

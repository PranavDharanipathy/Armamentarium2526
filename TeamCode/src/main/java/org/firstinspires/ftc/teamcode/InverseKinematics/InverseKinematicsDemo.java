package org.firstinspires.ftc.teamcode.InverseKinematics;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/// Only made a demo class of IntakePrecise.
public class InverseKinematicsDemo extends OpMode {

    IntakePrecise intake;

    Pose targetPose;

    Pose startPose;

    @Override
    public void init() {
        startPose = new Pose(72, 72);
        targetPose = new Pose(24, 24, 24);
        intake = new IntakePrecise(hardwareMap, super.gamepad1, startPose, targetPose);

    }

    @Override
    public void loop() {

        intake.update();

    }
}

package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(14, -62.69, 90*Math.PI/180);
        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
//                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .splineTo(new Vector2d(0, -40), 90*Math.PI/180)
                        .waitSeconds(2)
                        .turnTo(Math.toRadians(-180))
                        .splineTo(new Vector2d(-48, -38), 90*Math.PI/180)
                        .waitSeconds(2)
                        .splineTo(new Vector2d(-54, -54), -135*Math.PI/180)
                        .waitSeconds(2)
                        .splineTo(new Vector2d(-57.5, -38), 90*Math.PI/180)
                        .waitSeconds(2)
                        .splineTo(new Vector2d(-54, -54), -135*Math.PI/180)
                        .waitSeconds(2)
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}

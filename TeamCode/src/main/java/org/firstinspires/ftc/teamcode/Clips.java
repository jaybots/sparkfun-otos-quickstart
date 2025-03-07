package org.firstinspires.ftc.teamcode;
import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;
import java.util.Vector;

@Autonomous(name = "Clips", group = "Right Side (Clip Red/Blue blocks)")
public class Clips extends LinearOpMode {
    HardwareBot robot = new HardwareBot();

    int cutTime = 26;
    double startTime=0;

    public void runOpMode() {
        robot.init(hardwareMap);
        Pose2d initialPose = new Pose2d(7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(0,-29) ,Math.PI/2);

        TrajectoryActionBuilder pushBlock = drive.actionBuilder(new Pose2d(0, -28, Math.PI / 2))
                .setTangent(-Math.PI / 2)
                .splineToSplineHeading(new Pose2d(34, -34, -Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(34, -21), Math.PI / 2, new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(43, -8), -Math.PI / 2, new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(43, -46), -Math.PI / 2)
                .strafeTo(new Vector2d(34,-46));

        TrajectoryActionBuilder wall2Bar1 = drive.actionBuilder(new Pose2d(34, -56, -Math.PI / 2))
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-2, -27, Math.PI / 2), Math.PI / 2);

        TrajectoryActionBuilder wall2Bar2 = drive.actionBuilder(new Pose2d(34, -56, -Math.PI / 2))
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(0, -27, Math.PI / 2), Math.PI / 2);

        TrajectoryActionBuilder bar2Wall = drive.actionBuilder(new Pose2d(-2, -27, Math.PI / 2))
                .setTangent(-Math.PI / 2)
                .splineToSplineHeading(new Pose2d(38, -52, -Math.PI / 2), -Math.PI / 2);

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(2, -27, Math.PI / 2))
                .setTangent(-Math.PI / 2)
                .splineToSplineHeading(new Pose2d(38, -52, Math.PI / 2), 0);

        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to", "start");
            telemetry.update();
        }
        startTime = getRuntime();
        boolean done = false;
        robot.lift.setPower(1);
        robot.tilt.setPower(1);
        robot.lift.setTargetPosition(2050);
        sleep(200);
        runBlocking(new SequentialAction(start.build()));
        robot.forwardTouch();
        sleep(500);
        robot.releaseSpecimen();
        robot.tilt.setTargetPosition(150);
        runBlocking(new SequentialAction(pushBlock.build()));
        robot.forwardTouch();
        boolean goodGrab = robot.grabSpecimen();
        while (!goodGrab) {
            robot.backTime(.2,1000);
            sleep(500);
            if (getRuntime()-startTime>cutTime){
                return;
            }
            robot.forwardTouch();
            goodGrab = robot.grabSpecimen();
        }
        robot.lift.setTargetPosition(2050);
        sleep(100);
        robot.tilt.setTargetPosition(0);
        SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(34, -56, -Math.PI / 2));
        runBlocking(new SequentialAction(wall2Bar1.build()));
        robot.touch.getState();
        robot.forwardTouch();
        robot.releaseSpecimen();
        robot.tilt.setTargetPosition(150);
        SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(-2, -27, Math.PI / 2));
        runBlocking(new SequentialAction(bar2Wall.build()));
        robot.touch.getState();
        robot.forwardTouch();
        sleep(300);
        goodGrab = robot.grabSpecimen();
        while (!goodGrab) {
            robot.backTime(.2, 500);
            if (getRuntime() - startTime > cutTime) {
                return;
            }
            sleep(500);
            robot.forwardTouch();
            goodGrab = robot.grabSpecimen();
        }
        robot.lift.setTargetPosition(2050);
        sleep(100);
        robot.tilt.setTargetPosition(0);
        SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(34, -56, -Math.PI / 2));
        runBlocking(new SequentialAction(wall2Bar2.build()));
        robot.touch.getState();
        robot.forwardTouch();
        sleep(300);
        robot.releaseSpecimen();
        robot.tilt.setTargetPosition(0);
        SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(2, -27, Math.PI / 2));
        runBlocking(new SequentialAction(park.build()));
    }  //ends runOpMode
}  //ends Clips class


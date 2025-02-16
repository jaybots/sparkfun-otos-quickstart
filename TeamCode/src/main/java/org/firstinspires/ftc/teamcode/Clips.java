package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;


@Autonomous(name = "Clips", group = "Right Side (Clip Red/Blue blocks)")
public class Clips extends LinearOpMode {
    HardwareBot robot = new HardwareBot();

    public void runOpMode() {
        robot.init(hardwareMap);

        Pose2d initialPose = new Pose2d(7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-30);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(7, -31,Math.PI/2 ))
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(33,-34,-Math.PI/2),Math.PI/2)
                .splineToConstantHeading(new Vector2d(33,-21),Math.PI/2,new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(47,-8),-Math.PI/2,new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(47, -42), -Math.PI / 2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(38,-52),-Math.PI/2);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(35, -56,-Math.PI/2 ))
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(2,-27, Math.PI/2),Math.PI/2);

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(2, -27,Math.PI/2 ))
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(38,-52, -Math.PI/2),-Math.PI/2);

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(2, -27,Math.PI/2 ))
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(38,-52, Math.PI/2),0);


        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to","start");
        }
        boolean done = false;
        robot.lift.setPower(1);
        robot.tilt.setPower(1);
        while (opModeIsActive() && !done) {
            if (isStopRequested()) return;
            robot.twist.setPosition(1);
            robot.lift.setTargetPosition(2180);
            robot.tilt.setTargetPosition(380);
            sleep(200);
            Actions.runBlocking(new SequentialAction(tab1.build()));
            sleep(300);
            robot.releaseSpecimen();
            robot.tilt.setTargetPosition(150);
            Actions.runBlocking(new SequentialAction(tab2.build()));
            sleep(100);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(2,0 ),0));
            sleep(1000);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0. ),0));
            robot.grabSpecimen();
            robot.lift.setTargetPosition(2050);
            sleep(100);
            Actions.runBlocking(new SequentialAction(tab3.build()));
            sleep(500);
            robot.releaseSpecimen();
            sleep(200);
            Actions.runBlocking(new SequentialAction(tab4.build()));
            sleep(100);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(2,0 ),0));
            sleep(1000);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0. ),0));
            robot.grabSpecimen();
            robot.lift.setTargetPosition(2050);
            sleep(100);
            Actions.runBlocking(new SequentialAction(tab3.build()));
            sleep(500);
            robot.releaseSpecimen();
            sleep(200);
            Actions.runBlocking(new SequentialAction(tab5.build()));
            done = true;
        }
    }


}
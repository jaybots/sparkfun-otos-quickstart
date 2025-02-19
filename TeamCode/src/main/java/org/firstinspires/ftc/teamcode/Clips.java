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
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;


@Autonomous(name = "Clips", group = "Right Side (Clip Red/Blue blocks)")
public class Clips extends LinearOpMode {
    HardwareBot robot = new HardwareBot();
    Pose2d initialPose = new Pose2d(7, -61, Math.toRadians(90));
    SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
    ElapsedTime match = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);

        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)
                .lineToY(-31);

        TrajectoryActionBuilder pushBlock = drive.actionBuilder(new Pose2d(7, -31, Math.PI / 2))
                .setTangent(-Math.PI / 2)
                .splineToSplineHeading(new Pose2d(33, -34, -Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(33, -21), Math.PI / 2, new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(46, -8), -Math.PI / 2, new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(46, -42), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(38, -52), -Math.PI / 2);

        TrajectoryActionBuilder wall2Bar = drive.actionBuilder(new Pose2d(35, -56, -Math.PI / 2))
                .setTangent(Math.PI / 2)
                .splineToSplineHeading(new Pose2d(2, -27, Math.PI / 2), Math.PI / 2);

        TrajectoryActionBuilder bar2Wall = drive.actionBuilder(new Pose2d(2, -27, Math.PI / 2))
                .setTangent(-Math.PI / 2)
                .splineToSplineHeading(new Pose2d(38, -52, -Math.PI / 2), -Math.PI / 2);

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(2, -27, Math.PI / 2))
                .setTangent(-Math.PI / 2)
                .splineToSplineHeading(new Pose2d(38, -52, Math.PI / 2), 0);


        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to", "start");
        }
        boolean done = false;
        robot.lift.setPower(1);
        robot.tilt.setPower(1);
        while (opModeIsActive() && !done) {
            if (isStopRequested()) return;
            robot.lift.setTargetPosition(2150);
            robot.tilt.setTargetPosition(380);
            sleep(200);
            Actions.runBlocking(new SequentialAction(start.build()));
            sleep(300);
            robot.releaseSpecimen();
            robot.tilt.setTargetPosition(150);
            Actions.runBlocking(new SequentialAction(pushBlock.build()));
            sleep(100);
            roll(1000);
            //allow two tries to get specimen
            while (!robot.grabSpecimen()) {
                rollBack(500);
                sleep(1500);
                roll(650);
                if (match.milliseconds()>25000){
                    rollBack(500);
                    sleep(5000);
                }
            }
            robot.lift.setTargetPosition(2050);
            sleep(100);
            Actions.runBlocking(new SequentialAction(wall2Bar.build()));
            sleep(500);
            robot.releaseSpecimen();
            sleep(200);
            Actions.runBlocking(new SequentialAction(bar2Wall.build()));
            sleep(100);
            roll(1000);
            while (!robot.grabSpecimen()) {
                rollBack(500);
                sleep(1500);
                roll(650);
                if (match.milliseconds()>25000){
                    rollBack(500);
                    sleep(5000);
                }
            }
            robot.lift.setTargetPosition(2050);
            sleep(100);
            Actions.runBlocking(new SequentialAction(wall2Bar.build()));
            sleep(500);
            robot.releaseSpecimen();
            sleep(200);
            Actions.runBlocking(new SequentialAction(park.build()));
            done = true;
        } //while opModeIsActive
    }  //ends runOpMode

    /**
     /rolls forward
     /@param t as int time in milliseconds
     */
    public void roll ( int t){
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(2, 0), 0));
        sleep(t);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0.), 0));
    }

    public void rollBack ( int t){
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-2, 0), 0));
        sleep(t);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0.), 0));
    }

}  //ends Clips3 class


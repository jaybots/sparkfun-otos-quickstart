package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
//

@Autonomous(name = "Clips", group = "Right Side (Clip Red/Blue blocks)")
public class Clips extends LinearOpMode {
    HardwareBot robot = new HardwareBot();

    ElapsedTime match = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        Pose2d initialPose = new Pose2d(7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder start = drive.actionBuilder(initialPose)
                .lineToY(-29);

        TrajectoryActionBuilder pushBlock = drive.actionBuilder(new Pose2d(7, -29, Math.PI / 2))
                .setTangent(-Math.PI / 2)
                .splineToSplineHeading(new Pose2d(34, -34, -Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(34, -21), Math.PI / 2, new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(43, -8), -Math.PI / 2, new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(43, -46), -Math.PI / 2)
                .strafeTo(new Vector2d(34,-46));

        TrajectoryActionBuilder wall2Bar = drive.actionBuilder(new Pose2d(34, -56, -Math.PI / 2))
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
            telemetry.update();
        }
        boolean done = false;
        robot.lift.setPower(1);
        robot.tilt.setPower(1);
        while (opModeIsActive() && !done) {
            boolean abort = false;
            if (isStopRequested()) return;
            robot.lift.setTargetPosition(2140);
            robot.tilt.setTargetPosition(300);
            sleep(200);
            runBlocking(new SequentialAction(start.build()));
            sleep(300);
            robot.releaseSpecimen();
            robot.tilt.setTargetPosition(150);
            runBlocking(new SequentialAction(pushBlock.build()));
            robot.forwardIr();
            boolean goodGrab = robot.grabSpecimen();
            while (!goodGrab) {
                robot.backTime(.2,1000);
                sleep(1000);
                robot.forwardIr();
                if (match.milliseconds()>25000){
                    abort = true;
                    break;
                }
                if (abort) break;
                goodGrab = robot.grabSpecimen();
            }
            if (!abort) {
                robot.lift.setTargetPosition(2050);
                sleep(100);
                robot.tilt.setTargetPosition(350);
                runBlocking(new SequentialAction(wall2Bar.build()));
                sleep(500);
                robot.releaseSpecimen();
                sleep(200);
                robot.tilt.setTargetPosition(150);
                runBlocking(new SequentialAction(bar2Wall.build()));
                sleep(100);
                robot.forwardIr();
                goodGrab = robot.grabSpecimen();
                while (!goodGrab) {
                    robot.backTime(.2, 500);
                    sleep(1500);
                    robot.forwardIr();
                    if (match.milliseconds() > 25000) {
                        abort = true;
                        break;
                    }
                    if (abort) break;
                    goodGrab = robot.grabSpecimen();
                }
            }
            if (!abort) {
                robot.lift.setTargetPosition(2050);
                robot.tilt.setTargetPosition(350);
                sleep(100);
                runBlocking(new SequentialAction(wall2Bar.build()));
                sleep(500);
                robot.releaseSpecimen();
                robot.tilt.setTargetPosition(0);
                sleep(200);
                runBlocking(new SequentialAction(park.build()));
            }
            done = true;
        } //while opModeIsActive
    }  //ends runOpMode
}  //ends Clips class


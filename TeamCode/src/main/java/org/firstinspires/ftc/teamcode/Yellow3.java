package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Yellow 3 samples in basket")
public class Yellow3 extends LinearOpMode {
    HardwareBot robot = new HardwareBot();

    public void runOpMode() {
        robot.init(hardwareMap);

        //start so right wheels are on edge of tile even with angled support (facing submersible)
        Pose2d initialPose = new Pose2d(-30, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder basket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-52,-52),Math.toRadians(225));

        TrajectoryActionBuilder grab1 = drive.actionBuilder(new Pose2d(-52, -52,Math.toRadians(225)))
                .turnTo(Math.toRadians(86));

        TrajectoryActionBuilder rotateToBasket = drive.actionBuilder(new Pose2d(-52, -52,Math.toRadians(86)))
                .turnTo(Math.toRadians(225));


        TrajectoryActionBuilder rotateToBasket2 = drive.actionBuilder(new Pose2d(-52, -52,Math.toRadians(105)))
                .turnTo(Math.toRadians(225));


        TrajectoryActionBuilder grab2 = drive.actionBuilder(new Pose2d(-52, -52,Math.toRadians(225)))
                .turnTo(Math.toRadians(105));


        TrajectoryActionBuilder basket2 = drive.actionBuilder(new Pose2d(-52, -52,Math.toRadians(83)))
                .splineTo(new Vector2d(-57,-55),Math.toRadians(215));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-52, -52,Math.toRadians(225) ))
                .setReversed(true)
                .splineTo(new Vector2d(-30,-6), Math.toRadians(5));

        TrajectoryActionBuilder back = drive.actionBuilder(new Pose2d(-57, -57,Math.toRadians(225) ))
                .splineToConstantHeading(new Vector2d(-49,-49),225)
                .turnTo(Math.toRadians(75));


        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to","start");
        }
        boolean done = false;
        robot.lift.setPower(1);
        robot.tilt.setPower(1);

        if (isStopRequested()) return;
        robot.twist.setPosition(robot.twistZero);
        robot.claw.setPosition(robot.clawOpen);
        Actions.runBlocking(new SequentialAction(basket.build()));
        robot.lift.setTargetPosition(robot.maxHeight+100);
        while (robot.lift.getCurrentPosition()<robot.maxHeight - 100) {}
        robot.tilt.setTargetPosition(400);
        sleep(1000);
        robot.spinOut();
        sleep(1000);
        robot.tilt.setTargetPosition(150);
        robot.spinStop();
        sleep(500);
        robot.lift.setTargetPosition(1700);
        Actions.runBlocking(new SequentialAction(grab1.build()));
        robot.spinIn();
        robot.tilt.setTargetPosition(2350);
        sleep(1500);
        robot.spinStop();
        robot.tilt.setTargetPosition(0);
        sleep(1000);
        Actions.runBlocking(new SequentialAction(rotateToBasket.build()));
        robot.lift.setTargetPosition(robot.maxHeight+200);
        sleep(1000);
        robot.tilt.setTargetPosition(400);
        sleep(1500);
        robot.spinOut();
        sleep(1000);
        robot.spinStop();
        robot.tilt.setPower(.4);
        robot.tilt.setTargetPosition(200);
        sleep(1000);
        robot.tilt.setPower(1);
        robot.lift.setTargetPosition(1700);
        robot.twist.setPosition(.45);
        Actions.runBlocking(new SequentialAction(grab2.build()));
        robot.tilt.setTargetPosition(2350);
        sleep(1000);
        robot.spinIn();
        sleep(500);
        robot.spinStop();
        robot.tilt.setTargetPosition(200);
        sleep(500);
        Actions.runBlocking(new SequentialAction(rotateToBasket2.build()));
        robot.lift.setTargetPosition(robot.maxHeight+100);
        sleep(1000);
        robot.tilt.setTargetPosition(400);
        robot.spinOut();
        sleep(1000);
        robot.spinStop();
        robot.tilt.setTargetPosition(100);
        robot.lift.setTargetPosition(0);
        Actions.runBlocking(new SequentialAction(park.build()));
        robot.hang.setPower(.5);
        robot.backTime(.3,1000);
        robot.hang.setPower(.5);
        sleep(1500);
        robot.hang.setPower(0);
    }
}
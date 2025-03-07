package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Yellow 2 samples in basket")
public class Yellow2 extends LinearOpMode {
    HardwareBot robot = new HardwareBot();

    public void runOpMode() {
        robot.init(hardwareMap);

        //start so right wheels are on edge of tile even with angled support (facing submersible)
        Pose2d initialPose = new Pose2d(-30, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder basket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-54,-54),Math.toRadians(225));

        TrajectoryActionBuilder basket2 = drive.actionBuilder(new Pose2d(-46, -46,Math.toRadians(85)))
                .splineTo(new Vector2d(-57,-55),Math.toRadians(215));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-55, -55,Math.toRadians(235) ))
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
        robot.lift.setTargetPosition(robot.maxHeight);
        while (robot.lift.getCurrentPosition()<robot.maxHeight - 100) {}
        robot.tilt.setTargetPosition(300);
        robot.spin1.setPower(-1);
        robot.spin2.setPower(1);
        sleep(1000);
        robot.tilt.setTargetPosition(150);
        robot.spin1.setPower(0);
        robot.spin2.setPower(0);
        sleep(500);
        robot.lift.setTargetPosition(1600);
        SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(-57, -57,Math.toRadians(225)));
        Actions.runBlocking(new SequentialAction(back.build()));
        robot.lift.setTargetPosition(1600);
        robot.tilt.setTargetPosition(2200);
        sleep(1000);
        robot.leftPixy();
        robot.lift.setTargetPosition(1100);
        sleep(500);
        robot.spinIn();
        robot.tilt.setTargetPosition(2350);
        sleep(600);
        robot.spinStop();
        robot.tilt.setTargetPosition(0);
        sleep(1000);
        robot.lift.setTargetPosition(robot.maxHeight);
        SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(-49, -49,Math.toRadians(85)));
        Actions.runBlocking(new SequentialAction(basket2.build()));
        robot.tilt.setTargetPosition(300);
        sleep(1000);
        robot.spinOut();
        sleep(2000);
        robot.spinStop();
        robot.tilt.setTargetPosition(0);
        sleep(700);
        robot.lift.setTargetPosition(0);
        sleep(300);
        SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(-55, -55,Math.toRadians(238)));
        Actions.runBlocking(new SequentialAction(park.build()));
        robot.backTime(.2,1000);
        robot.hang.setPower(.5);
        sleep(2500);
        robot.hang.setPower(0);
    }
}
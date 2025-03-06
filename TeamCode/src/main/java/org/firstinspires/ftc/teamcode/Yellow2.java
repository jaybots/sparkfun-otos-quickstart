package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
        Pose2d initialPose = new Pose2d(-29, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder basket = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-54,-54),Math.toRadians(225));

        TrajectoryActionBuilder basket2 = drive.actionBuilder(new Pose2d(-45, -45,Math.toRadians(85)))
                .splineTo(new Vector2d(-57,-54),Math.toRadians(220));

        TrajectoryActionBuilder park = drive.actionBuilder(new Pose2d(-55, -55,Math.toRadians(235) ))
                .setReversed(true)
                .splineTo(new Vector2d(-30,-9), Math.toRadians(0));

        TrajectoryActionBuilder back = drive.actionBuilder(new Pose2d(-57, -57,Math.toRadians(225) ))
                .lineToXConstantHeading(-48)
                .turnTo(Math.toRadians(70));


        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to","start");
        }
        boolean done = false;
        robot.lift.setPower(1);
        robot.tilt.setPower(1);
        while (opModeIsActive() && !done) {
            if (isStopRequested()) return;
            robot.twist.setPosition(robot.twistZero);
            robot.claw.setPosition(robot.clawOpen);
            Actions.runBlocking(new SequentialAction(basket.build()));
            robot.lift.setTargetPosition(robot.maxHeight);
            while (robot.lift.getCurrentPosition()<robot.maxHeight - 100) {}
            robot.tilt.setTargetPosition(400);
            //robot.forwardTime(.2,800);
            robot.spin1.setPower(-1);
            robot.spin2.setPower(1);
            sleep(1000);
            robot.tilt.setTargetPosition(0);
            robot.spin1.setPower(0);
            robot.spin2.setPower(0);
            robot.lift.setTargetPosition(1600);
            sleep(500);
            SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(-57, -57,Math.toRadians(225)));
            Actions.runBlocking(new SequentialAction(back.build()));
            robot.lift.setTargetPosition(1600);
            sleep(1000);
            robot.tilt.setTargetPosition(2150);
            sleep(1000);
            robot.leftPixy();
            robot.lift.setTargetPosition(1250);
            sleep(500);
            robot.spinIn();
            robot.tilt.setTargetPosition(2300);
            sleep(600);
            robot.spinStop();
            robot.tilt.setTargetPosition(0);
            sleep(1000);
            robot.lift.setTargetPosition(robot.maxHeight);
            SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(-45, -45,Math.toRadians(85)));
            Actions.runBlocking(new SequentialAction(basket2.build()));
            robot.tilt.setTargetPosition(150);
            robot.forwardTime(.2,1000);
            robot.spinOut();
            sleep(700);
            robot.spinStop();
            robot.backTime(.2,500);
            robot.lift.setTargetPosition(0);
            SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(-55, -55,Math.toRadians(235)));
            Actions.runBlocking(new SequentialAction(park.build()));
            sleep(10000);
            robot.hang.setPower(.5);
            sleep(2500);
            robot.hang.setPower(0);
            done = true;
        }
    }


}
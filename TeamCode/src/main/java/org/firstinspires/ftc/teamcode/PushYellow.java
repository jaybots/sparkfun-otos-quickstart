package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Left Side (clip color then push yellow)")
public class PushYellow extends LinearOpMode {
    RRHardware rr = new RRHardware();


    public void runOpMode() {
        rr.init(hardwareMap);

        Pose2d initialPose = new Pose2d(-7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-30);


        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-7, -30,Math.toRadians(90) ))
                .setTangent(-Math.PI/2)
                .splineTo(new Vector2d(-19,-35),Math.PI)
                .splineTo(new Vector2d(-37,-8),Math.toRadians(85))
                .setTangent(Math.toRadians(265))
                .splineTo(new Vector2d(-42,-25),Math.toRadians(265))
                .splineTo(new Vector2d(-44,-55),Math.toRadians(265))
                .setTangent(Math.toRadians(85))
                .splineTo(new Vector2d(-46,-8),Math.toRadians(85))
                .setTangent(Math.toRadians(265))
                .splineTo(new Vector2d(-58,-45),Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-50,0),Math.toRadians(90))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-60,-10),Math.toRadians(270))
                .splineTo(new Vector2d(-60,-42),Math.toRadians(270))
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-30,-9),Math.toRadians(0));

        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to","start");
        }
        boolean done = false;
        rr.lift.setPower(1);
        rr.tilt.setPower(1);
        while (opModeIsActive() && !done) {
            if (isStopRequested()) return;
            rr.twist.setPosition(1);
            //rr.lift.setTargetPosition(2180);
            //rr.tilt.setTargetPosition(380);
            sleep(500);
            Actions.runBlocking(new SequentialAction(tab1.build()));
            sleep(300);
            //rr.releaseSpecimen();
            //rr.tilt.setTargetPosition(0);
            //push three yellow blocks to corner
            Actions.runBlocking(new SequentialAction(tab2.build()));
            sleep(100);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-1,0 ),0));
            sleep(2000);
            rr.hang.setPower(-.5);
            sleep(2500);
            rr.hang.setPower(0);
            done = true;
        }
    }


}
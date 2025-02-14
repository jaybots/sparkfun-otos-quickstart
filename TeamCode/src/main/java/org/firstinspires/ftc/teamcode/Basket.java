package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Left (clip color then push yellow)")
public class Basket extends LinearOpMode {
    RRHardware rr = new RRHardware();

    public void runOpMode() {
        rr.init(hardwareMap);

        Pose2d initialPose = new Pose2d(-7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-30)
                .splineToConstantHeading(new Vector2d(-14,-40),Math.PI/2);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-14, -30,Math.PI/2 ))
                .splineToConstantHeading(new Vector2d(-33,-21),Math.PI/2)
                .splineToConstantHeading(new Vector2d(-47,-8),-Math.PI/2)
                .splineToConstantHeading(new Vector2d(-47, -56), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-47, -6), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-55, -18), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-55, -52), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-55, -6), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-60, -18), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-60, -44), -Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-60, -24), -Math.PI / 2)
                .splineToSplineHeading(new Pose2d(-20,-8,Math.PI),0);

        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to","start");
        }
        boolean done = false;
        rr.lift.setPower(1);
        rr.tilt.setPower(1);
        while (opModeIsActive() && !done) {
            if (isStopRequested()) return;
            rr.claw.setPosition(1);
            rr.lift.setTargetPosition(2200);
            rr.tilt.setTargetPosition(380);
            sleep(200);
            Actions.runBlocking(new SequentialAction(tab1.build()));
            sleep(300);
            rr.releaseSpecimen();
            rr.tilt.setTargetPosition(0);
            //push three yellow blocks to corner
            Actions.runBlocking(new SequentialAction(tab2.build()));
            sleep(100);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-1,0 ),0));
            sleep(2000);
            rr.hang.setPower(.1);
            sleep(2000);
            rr.hang.setPower(0);
            done = true;
        }
    }


}
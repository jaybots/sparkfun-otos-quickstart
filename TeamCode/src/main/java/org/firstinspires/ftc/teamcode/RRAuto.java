package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.lang.Math;


@Autonomous(name = "RRAuto", group = "RoadRunner")
public class RRAuto extends LinearOpMode {
    RRHardware rr = new RRHardware();

    public void runOpMode() {
        rr.init(hardwareMap);

        Pose2d initialPose = new Pose2d(7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-33);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(7, -33,Math.PI/2 ))
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(32,-31,-Math.PI/2),Math.PI/2)
                .splineToConstantHeading(new Vector2d(32,-21),Math.PI/2,new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(49,-8),-Math.PI/2,new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(49, -46), -Math.PI / 2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(49,-8),Math.PI/2)
                .splineToConstantHeading(new Vector2d(58,-20),-Math.PI/2)
                .splineToConstantHeading(new Vector2d(58,-46),-Math.PI/2)
                .splineToConstantHeading(new Vector2d(38,-65),-Math.PI/2);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(37, -65,-Math.PI/2 ))
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(2,-27, Math.PI/2),Math.PI/2);

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(2, -27,-Math.PI/2 ))
                .setTangent(-Math.PI/2)
                //.splineToSplineHeading(new Pose2d(38,-65, -Math.PI/2),Math.PI/2);
                //Using splinetolinearheading in a attempt to prevent the odd run into the wall during T4
                //If does not work use .lineToX(38) This will solve for y auto. A turn would need added
                .splineToLinearHeading(Pose2d(38,-65,-Math.PI/2),Math.PI/2);


        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to","start");
        }
        boolean done = false;
        rr.lift.setPower(1);
        rr.tilt.setPower(1);
        while (opModeIsActive() && !done) {
            if (isStopRequested()) return;
            rr.lift.setTargetPosition(2250);
            rr.tilt.setTargetPosition(400);
            sleep(500);
            Actions.runBlocking(new SequentialAction(tab1.build()));
            rr.tilt.setTargetPosition(500);
            sleep(500);
            rr.releaseSpecimen();
            rr.tilt.setTargetPosition(160);
            Actions.runBlocking(new SequentialAction(tab2.build()));
            rr.grabSpecimen();
            rr.lift.setTargetPosition(2150);
            Actions.runBlocking(new SequentialAction(tab3.build()));
            rr.tilt.setTargetPosition(150);
            sleep(500);
            rr.releaseSpecimen();
            sleep(200);
            rr.claw.setPosition(0.56);
            //drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0.2 ),0));
            Actions.runBlocking(new SequentialAction(tab4.build()));
            sleep(1000);
            done = true;
        }
    }


}

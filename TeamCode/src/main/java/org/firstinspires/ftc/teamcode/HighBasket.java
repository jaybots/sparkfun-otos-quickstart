package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Left Side (yellow to high basket)")
public class HighBasket extends LinearOpMode {
    RRHardware rr = new RRHardware();
    double fast = 30;
    double slow = 10;

    public void runOpMode() {
        rr.init(hardwareMap);

        Pose2d initialPose = new Pose2d(-7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-55,-55),Math.toRadians(225));


        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-55, -55,Math.toRadians(225) ))
                .setTangent(Math.toRadians(45))
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
            sleep(1500);
            Actions.runBlocking(new SequentialAction(tab1.build()));
            //rr.lift.setTargetPosition(3500);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(1,0 ),0));
            sleep(2000);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0 ),0));
            rr.spin1.setPower(-1);
            rr.spin2.setPower(1);
            sleep(1500);
            rr.spin1.setPower(0);
            rr.spin2.setPower(0);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-1,0 ),0));
            sleep(1000);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0 ),0));
            //rr.lift.setTargetPosition(0)
            sleep(2000);
            Actions.runBlocking(new SequentialAction(tab2.build()));
            sleep(100);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-1,0 ),0));
            sleep(3000);
            rr.hang.setPower(-.5);
            sleep(2500);
            rr.hang.setPower(0);
            done = true;
        }
    }


}
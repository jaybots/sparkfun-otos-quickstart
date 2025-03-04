package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-55,-55),Math.toRadians(225));


        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-55, -55,Math.toRadians(225) ))
                .setTangent(Math.toRadians(45))
                .splineTo(new Vector2d(-30,-9),Math.toRadians(0));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-51, -50,Math.toRadians(225) ))
                .turnTo(Math.toRadians(75));



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
            Actions.runBlocking(new SequentialAction(tab1.build()));
            robot.lift.setTargetPosition(robot.maxHeight);
            while (robot.lift.getCurrentPosition()<robot.maxHeight - 100) {}
            robot.tilt.setTargetPosition(150);
            robot.forwardTime(.2,1000);
            robot.spin1.setPower(-1);
            robot.spin2.setPower(1);
            sleep(1500);
            robot.tilt.setTargetPosition(0);
            robot.spin1.setPower(0);
            robot.spin2.setPower(0);
            robot.backTime(.2,1000);
            robot.lift.setTargetPosition(1500);
            sleep(1000);
            Actions.runBlocking(new SequentialAction(tab3.build()));
            robot.tilt.setTargetPosition(2150);
            sleep(1000);
            robot.lift.setTargetPosition(1500);
            robot.liftTarget=1500;
            robot.leftPixy();
            robot.retractPixy();
            robot.spinIn();
            robot.tilt.setTargetPosition(2300);
            sleep(1000);
            robot.spinStop();
            robot.tilt.setTargetPosition(0);
            sleep(10000);
            Actions.runBlocking(new SequentialAction(tab2.build()));
            sleep(100);
            robot.backTime(.2,3000);
            robot.hang.setPower(.5);
            sleep(2500);
            robot.hang.setPower(0);
            done = true;
        }
    }


}
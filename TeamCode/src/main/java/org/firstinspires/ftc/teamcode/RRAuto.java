package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous(name = "RRAuto", group = "RoadRunner")
public class RRAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SparkFunOTOS odo =  hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        odo.calibrateImu();
        sleep(500);
        odo.setLinearUnit(DistanceUnit.INCH);
        odo.setAngularUnit(AngleUnit.DEGREES);
        odo.resetTracking();
        sleep(100);
        odo.setLinearScalar(1.0);
        sleep(100);
        odo.setAngularScalar(1.0);
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift");
        DcMotor tilt = hardwareMap.get(DcMotor.class, "tilt");
        DcMotor hang = hardwareMap.get(DcMotor.class, "hang");
        CRServo spin1 = hardwareMap.get(CRServo.class, "spin1");
        CRServo spin2 = hardwareMap.get(CRServo.class, "spin2");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        lift.setDirection(DcMotor.Direction.REVERSE);
        tilt.setDirection(DcMotor.Direction.REVERSE);
        hang.setDirection(DcMotor.Direction.FORWARD);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hang.setPower(0);
        spin1.setPower(0);
        spin2.setPower(0);
        claw.setPosition(0.56);  //closed

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(12)
                .turn(Math.PI);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(20, 0,Math.PI ))
                .lineToX(0);


        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to","start");
        }
        boolean done = false;
        lift.setPower(0);
        tilt.setPower(0);
        while (opModeIsActive() && !done) {

            if (isStopRequested()) return;

            lift.setTargetPosition(2050);
            tilt.setTargetPosition(320);
            Actions.runBlocking(new SequentialAction(tab1.build()));
            tilt.setTargetPosition(490);
            sleep(500);
            lift.setTargetPosition(1500);
            sleep(600);
            claw.setPosition(0.3);
            lift.setTargetPosition(0);
            tilt.setTargetPosition(0);
            Actions.runBlocking(new SequentialAction(tab2.build()));
            lift.setPower(0);
            tilt.setPower(0);
            done = true;
        }
    }
}
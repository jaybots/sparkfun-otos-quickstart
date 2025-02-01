package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
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

        Pose2d initialPose = new Pose2d(7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(-33);


        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(7, -33,Math.PI/2 ))
                .setTangent(-Math.PI/2)
                .splineToSplineHeading(new Pose2d(35,-31,-Math.PI/2),Math.PI/2)
                .splineToConstantHeading(new Vector2d(35,-21),Math.PI/2,new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(45,-8),-Math.PI/2,new TranslationalVelConstraint(20))
                .splineToConstantHeading(new Vector2d(45, -46), -Math.PI / 2)
                .setTangent(Math.PI/2)
                .splineToConstantHeading(new Vector2d(45,-8),Math.PI/2)
                .splineToConstantHeading(new Vector2d(58,-20),-Math.PI/2)
                .splineToConstantHeading(new Vector2d(58,-46),-Math.PI/2);
                //.lineToLinearHeading(new Pose2d(9,-35,Math.PI/2))



        while (!isStopRequested() && !opModeIsActive()) {
            sleep(100);
            telemetry.addData("ready to","start");
        }
        boolean done = false;
        lift.setPower(1);
        tilt.setPower(1);
        while (opModeIsActive() && !done) {

            if (isStopRequested()) return;
            lift.setTargetPosition(2100);
            tilt.setTargetPosition(400);
            Actions.runBlocking(new SequentialAction(tab1.build()));
            tilt.setTargetPosition(500);
            sleep(500);
            lift.setTargetPosition(1500);
            sleep(300);
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
package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RRHardware {
    public ElapsedTime timer = new ElapsedTime();
    public DcMotor lift = null;
    public DcMotor tilt = null;
    public Servo claw = null;
    public CRServo spin1 = null;
    public CRServo spin2 = null;
    public DistanceSensor laser = null;
    HardwareMap rrMap = null;
    public void init(HardwareMap arrMap){
        rrMap = arrMap;

        lift = rrMap.get(DcMotor.class,"lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setPower(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        tilt = rrMap.get(DcMotor.class, "tilt");
        tilt.setDirection(DcMotor.Direction.FORWARD);
        tilt.setPower(0);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        claw = rrMap.get(Servo.class, "claw");
        claw.setPosition(0.56); //closed  (0.30 is open);

        spin1 = rrMap.get(CRServo.class, "spin1");
        spin2 = rrMap.get(CRServo.class, "spin2");

        laser = rrMap.get(DistanceSensor.class,"laser");

    }

    public void sleep(int ms){
        timer.reset();
        while (timer.milliseconds()<ms){
        }
    }

    public void grabSpecimen(){
        claw.setPosition(0.56);
        sleep(400);
        lift.setTargetPosition(500);
        sleep(200);
    }

    public void releaseSpecimen(){
        lift.setTargetPosition(0);
        while (laser.getDistance(DistanceUnit.INCH)>10){
        }
        sleep(170);
        claw.setPosition(0.3);
    }

}
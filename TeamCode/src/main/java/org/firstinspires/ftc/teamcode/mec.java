package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import java.lang.Math;


@TeleOp(name="Mec")
public class Mec extends LinearOpMode
{
    HardwareBot robot = new HardwareBot();


    double liftControl = 0;
    double twistControl = 0;
    boolean grab = false;
    boolean release = false;
    ElapsedTime timer  = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        SparkFunOTOS.Pose2D pos;
        Pose2d initialPose = new Pose2d(7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);


        double speedFactor;
        int maxExt = 1697;


        // The while loop below runs until Play is pressed
        // Current info is displayed
        while (!isStarted() && !isStopRequested()){
            //robot.claw.setPosition(robot.clawOpen);
            pos = robot.odo.getPosition();
            telemetry.addData("laser", robot.laser.getDistance(DistanceUnit.INCH));
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);
            telemetry.addData("tilt pos", robot.tilt.getCurrentPosition());
            telemetry.addData("lift pos", robot.lift.getCurrentPosition());
            telemetry.addData("tilt target", robot.tiltTarget);
            telemetry.addData("lift target", robot.liftTarget);
            telemetry.addData("lift amps", robot.lift.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
            sleep(100);
        }

        robot.tilt.setPower(robot.tiltPower);
        robot.lift.setPower(robot.liftPower);

        while (opModeIsActive()) {

            telemetry.addData("laser", robot.laser.getDistance(DistanceUnit.INCH));
            telemetry.update();

            //reset lift encoder if amps are high and it's in down position
            if (robot.liftPosition < 100 && liftControl > -0.3 && robot.lift.getCurrent(CurrentUnit.AMPS) > 7) {
                robot.lift.setPower(0);
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftPosition = 0;
                robot.liftTarget = 10;
                robot.lift.setPower(robot.liftPower);
            }

            if (robot.tiltPosition > robot.floor/2){
                robot.liftTarget = Math.min(robot.liftTarget,maxExt);
                robot.claw.setPosition(robot.clawOpen);
            }

            robot.tilt.setTargetPosition(robot.tiltTarget);
            robot.lift.setTargetPosition(robot.liftTarget);


            robot.tiltPosition = robot.tilt.getCurrentPosition();
            robot.liftPosition = robot.lift.getCurrentPosition();

            liftControl = gamepad2.left_stick_y;
            twistControl = gamepad2.right_stick_x;

            //grab is set to true if right trigger is squeezed
            //it activates all intake mechanisms (claw and spinner)
            grab = gamepad2.right_trigger > 0.3;

            //release is set to true if left trigger is squeezed
            //it activates all outtake mechanisms (claw and spinner)
            release = gamepad2.left_trigger > 0.3;

            if (twistControl < -0.3 && robot.twistPosition < 1){
                robot.twistPosition += 0.01;
            }

            else if (twistControl > 0.3 && robot.twistPosition > 0){
                robot.twistPosition -= 0.01;
            }

            else if (gamepad2.right_stick_y < -0.3) robot.twistPosition = robot.twistZero;

            robot.twist.setPosition(robot.twistPosition);

            if (gamepad2.left_bumper){
                robot.releaseSpecimen();
                robot.liftTarget = 0;
            }

            if (gamepad1.x) {
                robot.hang.setPower(1);
            }
            else if (gamepad1.y){
                robot.hang.setPower(-1);
                robot.tiltTarget = 400;
            }
            else robot.hang.setPower(0);

            //going up by joystick
            if (liftControl < -0.3 && (robot.liftTarget < robot.maxHeight || gamepad2.back)){
                robot.liftTarget += 20;
            }
            //going down by joystick
            else if (liftControl  > 0.3 ){
                robot.liftTarget -=20;
                //allow lift to go down below zero if 'back' is held
                if (!gamepad2.back) robot.liftTarget = Math.max(0,robot.liftTarget);
            }




            //rotate tilt inward (up toward vertical)
            if (gamepad2.dpad_down) {
                robot.tiltTarget = Math.max(0,robot.tiltTarget - 15);
            }
            //rotate tilt away (down toward floor)
            else if (gamepad2.dpad_up && (robot.tiltPosition < 200 || robot.liftPosition < maxExt *.9 )){
                robot.tiltTarget = Math.min(robot.tiltTarget + 15, robot.floor);
            }

            if (grab && robot.tiltPosition < 300){
                //grab with claw
                robot.claw.setPosition(robot.clawClosed);
                robot.spinIn();

            }

            if (grab && robot.tiltPosition >300){
                //grab with wheels
                robot.spinIn();
            }

            if (gamepad2.a){
                robot.liftTarget = 0;
            }


            //lift various amounts
            if (gamepad2.x){

                if  (robot.liftPosition < 1900 && robot.tiltPosition < 100){
                    robot.liftTarget = 2000;
                    robot.twistPosition = 1;
                }

                else if (robot.tiltPosition < 100 && robot.liftPosition > 1900) {
                    robot.twistPosition = robot.twistZero;
                    robot.liftTarget = robot.maxHeight;
                    robot.claw.setPosition(robot.clawOpen);
                }
            }
            //tilt various amounts, timer enforces 0.5 second between changes
            if (gamepad2.y && timer.milliseconds()>500){
                timer.reset();
                if  (robot.tiltPosition < 100 || robot.tiltPosition > robot.floor*.9){
                    robot.tiltTarget = (int)(robot.floor*.75);
                    release = true;
                }
                else {
                    robot.tiltTarget = robot.floor - 20;
                }
            }

            //tilt back to vertical
            if (gamepad2.b ){
                robot.tiltTarget = 0;
            }

            if (release){
                //release
                robot.spinOut();
                robot.claw.setPosition(robot.clawOpen);
            }

            if (!release && !grab){
                robot.spinStop();
            }

            //wheel control
            double drv  = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist  = gamepad1.right_stick_x;
            if (gamepad1.dpad_up) drv = 0.5;
            if (gamepad1.dpad_down) drv = -0.5;
            if (gamepad1.dpad_right) strafe = 0.5;
            if (gamepad1.dpad_left) strafe = -0.5;
            //Fine controls for movement on the ACCESSORIES side
            if(gamepad2.dpad_right) strafe = 0.4;
            if(gamepad2.dpad_left) strafe = -0.4;

            //lower wheel power due to lift tilted down to ground
            if (robot.tiltPosition > 400 || robot.liftPosition > robot.maxHeight*.9){
                speedFactor = 0.5;
            }
            else
                speedFactor = 0.8;

            //speedFactor overrides
            if (gamepad1.right_trigger > 0.3) speedFactor = 0.8;
            if (gamepad1.left_trigger > 0.3) speedFactor = 0.5;

            double[] speeds = {
                    (drv + strafe + twist) * speedFactor,
                    (drv - strafe - twist) * speedFactor,
                    (drv - strafe + twist) * speedFactor,
                    (drv + strafe - twist) * speedFactor
            };

            //check if any motor speeds are >1, and normalize if needed
            double max = Math.abs(speeds[0]);
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }


            //Set the drv motors to the correct powers.

            robot.leftFront.setPower(speeds[0]);
            robot.rightFront.setPower(speeds[1]);
            robot.leftBack.setPower(speeds[2]);
            robot.rightBack.setPower(speeds[3]);

            if (gamepad2.right_bumper && robot.tiltPosition<200 && robot.liftPosition < 200 ){
                robot.grabSpecimen();

                //robot.twistPosition = 1;
                //robot.flip();
            }

        } //ends "while opMode is active" loop
    }  //brace ends runOpMode method
} //brace ends Mec.java class

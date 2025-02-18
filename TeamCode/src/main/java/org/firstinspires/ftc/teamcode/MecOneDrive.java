package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import java.lang.Math;


@TeleOp(name="MecOneDrive")
public class MecOneDrive extends LinearOpMode
{
    HardwareBot robot = new HardwareBot();

    int barHeight = 2150;
    double liftControl = 0;
    double twistControl = 0;
    boolean grab = false;
    boolean release = false;
    ElapsedTime timer  = new ElapsedTime();
    enum Mode {ADRIVE, BSUB, XBASKET, YBAR}
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        SparkFunOTOS.Pose2D pos;
        Pose2d initialPose = new Pose2d(7, -61, Math.toRadians(90));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        TrajectoryActionBuilder right1 = drive.actionBuilder(new Pose2d(0, 0,Math.PI/2))
                .strafeTo(new Vector2d(1,0), new TranslationalVelConstraint(1));
        TrajectoryActionBuilder left1 = drive.actionBuilder(new Pose2d(0, 0,Math.PI/2 ))
                .strafeTo(new Vector2d(-1,0), new TranslationalVelConstraint(1));
        TrajectoryActionBuilder forward1 = drive.actionBuilder(new Pose2d(0, 0,Math.PI/2 ))
                .lineToY(1, new TranslationalVelConstraint(1));
        TrajectoryActionBuilder back1 = drive.actionBuilder(new Pose2d(0, 0,Math.PI/2 ))
                .lineToY(-1, new TranslationalVelConstraint(1));

        double speedFactor;
        int maxExt = 1697;
        Mode driveMode = Mode.ADRIVE;

        // The while loop below runs until Play is pressed
        // Current info is displayed
        while (!isStarted() && !isStopRequested()){
            //robot.claw.setPosition(robot.clawOpen);
            pos = robot.odo.getPosition();
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

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad2.a) driveMode = Mode.ADRIVE;
            if (currentGamepad2.b) driveMode = Mode.BSUB;
            if (currentGamepad2.x) driveMode = Mode.XBASKET;
            if (currentGamepad2.y) driveMode = Mode.YBAR;

            //reset lift encoder if amps are high and it's in down position
            if (robot.liftPosition < 100 && gamepad1.left_stick_y < -0.3 && robot.lift.getCurrent(CurrentUnit.AMPS) > 7) {
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

            //CONTROLS THAT ARE ACTIVE IN ALL MODES

            //release is set to true if left trigger is squeezed
            //it activates all outtake mechanisms (claw and spinner)
            release = currentGamepad1.left_trigger > 0.3;

            //grab is set to true if right trigger is squeezed
            //it activates all intake mechanisms (claw and spinner)
            grab = currentGamepad1.right_trigger > 0.3;

            if (release){
                //release
                robot.spinOut();
                robot.claw.setPosition(robot.clawOpen);
            }

            if (!release && !grab){
                robot.spinStop();
            }

            if (currentGamepad1.a) robot.liftPosition = 0;
            if (currentGamepad1.b) robot.tiltPosition = 0;

            telemetry.addData(driveMode.name(), "x");
            telemetry.update();

            switch  (driveMode){
                //CONTROLS FOR BASIC DRIVE MODE
                case ADRIVE:
                    //lift control
                    if (currentGamepad1.x){
                        robot.liftTarget = barHeight;
                        if (robot.liftPosition > barHeight -50) robot.liftTarget = robot.maxHeight;
                    }

                    //tilt control
                    if (currentGamepad1.y && !previousGamepad1.y){
                        if  (robot.tiltPosition < 100 || robot.tiltPosition > robot.floor*.9){
                            robot.tiltTarget = (int)(robot.floor*.75);
                            release = true;
                        }
                        else {
                            robot.tiltTarget = robot.floor - 50;
                        }
                    }

                    //hang control
                    if (currentGamepad1.back) robot.hang.setPower(-1);
                    else if(currentGamepad1.start) {
                        robot.hang.setPower(1);
                        robot.tiltTarget = 400;
                    }
                    else robot.hang.setPower(0);

                    //wheel control
                    double drv  = -currentGamepad1.left_stick_y;
                    double strafe = currentGamepad1.left_stick_x;
                    double twist  = currentGamepad1.right_stick_x;
                    if (currentGamepad1.dpad_up) drv = 0.5;
                    if (currentGamepad1.dpad_down) drv = -0.5;
                    if (currentGamepad1.dpad_right) strafe = 0.5;
                    if (currentGamepad1.dpad_left) strafe = -0.5;

                    //lower wheel power due to lift tilted down to ground or lift being very high
                    if (robot.tiltPosition > 400 || robot.liftPosition > robot.maxHeight*.9){
                        speedFactor = 0.5;
                    }
                    else
                        speedFactor = 0.8;

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

                    //Set the drive motors to the correct powers.
                    robot.leftFront.setPower(speeds[0]);
                    robot.rightFront.setPower(speeds[1]);
                    robot.leftBack.setPower(speeds[2]);
                    robot.rightBack.setPower(speeds[3]);
                    break;

                //CONTROLS FOR INTAKE FROM SUBMERSIBLE
                case BSUB:

                    //rotate tilt using dpad
                    if (currentGamepad1.dpad_down) {
                        robot.tiltTarget = Math.max(0, robot.tiltTarget - 15);
                    }
                    else if (currentGamepad1.dpad_up && (robot.tiltPosition < 200 || robot.liftPosition < maxExt *.9 )){
                        robot.tiltTarget = Math.min(robot.tiltTarget + 15, robot.floor);
                    }

                    //toggle tilt positions quickly
                    if (currentGamepad1.y && !previousGamepad1.y){
                        if (robot.tiltPosition < robot.floor -100) robot.tiltTarget = (int)(robot.floor * .8);
                        else robot.tiltTarget = robot.floor - 100;
                    }

                    //tilt to higher position for exiting submersible
                    if (currentGamepad1.b) robot.tiltTarget = (int)(robot.floor * .6);

                    if (currentGamepad1.dpad_right &&  robot.twistPosition < 1){
                        robot.twistPosition += 0.01;
                    }

                    else if (currentGamepad1.dpad_left && robot.twistPosition > 0){
                        robot.twistPosition -= 0.01;
                    }

                    if (currentGamepad1.left_stick_x < -0.3 && previousGamepad1.left_stick_x > -0.3)
                        Actions.runBlocking(new SequentialAction(left1.build()));
                    if (currentGamepad1.left_stick_x > 0.3 && previousGamepad1.left_stick_x < 0.3)
                        Actions.runBlocking(new SequentialAction(right1.build()));
                    if (currentGamepad1.left_stick_y > 0.3 && previousGamepad1.left_stick_y < 0.3)
                        Actions.runBlocking(new SequentialAction(back1.build()));
                    if (currentGamepad1.left_stick_y < -0.3 && previousGamepad1.left_stick_y > -0.3)
                        Actions.runBlocking(new SequentialAction(forward1.build()));

                    break;
                //CONTROLS FOR OUTTAKE AT BASKET
                case XBASKET:

                    //going up by joystick
                    if (currentGamepad1.left_stick_y < -0.3 && (robot.liftTarget < robot.maxHeight || gamepad2.back)){
                        robot.liftTarget += 20;
                    }
                    //going down by joystick
                    else if (currentGamepad1.left_stick_y  > 0.3 ){
                        robot.liftTarget -=20;
                        //allow lift to go down below zero if 'back' is held
                        if (!gamepad1.back) robot.liftTarget = Math.max(0,robot.liftTarget);
                    }

                    break;

                case YBAR:
                    //CONTROLS FOR OUTTAKE AT BAR

                    break;
            }

            robot.twist.setPosition(robot.twistPosition);

            if (gamepad2.left_bumper){
                robot.releaseSpecimen();
                robot.liftTarget = 0;
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


            //tilt back to vertical
            if (gamepad2.b ){
                robot.tiltTarget = 0;
            }


            if (gamepad2.right_bumper && robot.tiltPosition<200 && robot.liftPosition < 200 ){
                robot.grabSpecimen();
                robot.twistPosition = 1;
                robot.flip();
            }

        } //ends "while opMode is active" loop
    }  //brace ends runOpMode method
} //brace ends Mec.java class

package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//
@TeleOp(name="Test")
public class Test extends LinearOpMode
{
    HardwareBot robot = new HardwareBot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0,0,Math.PI/2));


        TrajectoryActionBuilder forward3 = drive.actionBuilder(new Pose2d (0,0,Math.PI/2))
                .lineToY(3);

        TrajectoryActionBuilder back3 = drive.actionBuilder(new Pose2d (0,3,Math.PI/2))
                .lineToY(0);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // The while loop below runs until Play is pressed
        // Current info is displayed
        while (!isStarted() && !isStopRequested()){
            robot.claw.setPosition(robot.clawOpen);
            telemetry.addData("tilt pos", robot.tilt.getCurrentPosition());
            telemetry.addData("lift pos", robot.lift.getCurrentPosition());
            telemetry.addData("tilt target", robot.tiltTarget);
            telemetry.addData("lift target", robot.liftTarget);
            telemetry.addData("lift amps", robot.lift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("pixy", robot.pixy.getVoltage());
            telemetry.addData("touch", robot.touch.getState());
            telemetry.addData("time", getRuntime());
            telemetry.update();

            sleep(100);
        }

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(0,0,Math.PI/2));
                runBlocking(new SequentialAction(forward3.build()));
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(0,3,Math.PI/2));
                runBlocking(new SequentialAction(back3.build()));
            }


        } //brace ends "while opMode is active" loop
    }  //brace ends runOpMode method
} //brace ends Mec.java class

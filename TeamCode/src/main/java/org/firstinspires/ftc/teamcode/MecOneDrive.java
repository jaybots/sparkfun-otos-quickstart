package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="MecOneDrive")
public class MecOneDrive extends LinearOpMode
{
    double hangPower = 0;
    double setHangTime = 0;
    double startTime = 0;
    ElapsedTime ampTimer = new ElapsedTime();
    boolean ryanMode = false;
    double amps = 0;
    HardwareBot robot = new HardwareBot();
    double speedFactor = .7;
    double drv = 0;
    double strafe = 0;
    double twist = 0;
    int barHeight = 1860;
    int tiltVertical = 150;
    boolean grab = false;
    boolean release = false;
    enum Mode {ADRIVE, BSUB, XBASKET, YBAR}
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(40,-56,-Math.PI/2));

        TrajectoryActionBuilder wall2Bar = drive.actionBuilder(new Pose2d(40,-56,-Math.PI/2))
                    .setTangent(Math.PI / 2)
                    .splineToSplineHeading(new Pose2d(2, -28, Math.PI / 2), Math.PI / 2);

        TrajectoryActionBuilder bar2Wall = drive.actionBuilder(new Pose2d(0,-26,Math.PI/2))
                    .setTangent(-Math.PI / 2)
                    .splineToSplineHeading(new Pose2d(40, -50, -Math.PI / 2), -Math.PI / 2);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        int maxExt = 1690;
        Mode driveMode = Mode.ADRIVE;

        // The while loop below runs until Play is pressed
        // Current info is displayed
        while (!isStarted() && !isStopRequested()){
            if (robot.pixy.getVoltage() > 0.1) robot.claw.setPosition(robot.clawClosed);
            else robot.claw.setPosition(robot.clawOpen);
            telemetry.addData("tilt pos", robot.tilt.getCurrentPosition());
            telemetry.addData("lift pos", robot.lift.getCurrentPosition());
            telemetry.addData("tilt target", robot.tiltTarget);
            telemetry.addData("lift target", robot.liftTarget);
            telemetry.addData("lift amps", robot.lift.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("pixy", robot.pixy.getVoltage());
            telemetry.addData("touch", robot.touch.getState());
            telemetry.addData("time", getRuntime());
            telemetry.update();
            if (gamepad2.dpad_left) robot.led.setPower(0.5);
            if (gamepad2.dpad_right) robot.led.setPower(0);
            sleep(100);
        }
        startTime = getRuntime();
        robot.tilt.setPower(robot.tiltPower);
        robot.lift.setPower(robot.liftPower);

        double[] speeds = {0, 0, 0, 0};
        robot.twist.setPosition(robot.twistZero);
        robot.led.setPower(0);

        while (opModeIsActive()) {
            if (isStopRequested()) return;
//            telemetry.addData("tilt position", robot.tiltPosition);
//            telemetry.addData("tiltTarget", robot.tiltTarget);
//            telemetry.addData("time",getRuntime()-startTime);
//            telemetry.addData("hook power", hangPower);
//            telemetry.update();

            robot.hang.setPower(hangPower);

            amps = robot.lift.getCurrent(CurrentUnit.AMPS);
            robot.tilt.setTargetPosition(robot.tiltTarget);
            robot.lift.setTargetPosition(robot.liftTarget);

            robot.tiltPosition = robot.tilt.getCurrentPosition();
            robot.liftPosition = robot.lift.getCurrentPosition();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (ryanMode){
                boolean temp;
                temp = currentGamepad1.x;
                currentGamepad1.x = currentGamepad1.a;
                currentGamepad1.x = temp;
                temp = currentGamepad1.y;
                currentGamepad1.y = currentGamepad1.x;
                currentGamepad1.x = temp;
                temp = currentGamepad1.b;
                currentGamepad1.b = currentGamepad1.y;
                currentGamepad1.y = temp;
                temp = currentGamepad1.a;
                currentGamepad1.a = currentGamepad1.b;
                currentGamepad1.b = temp;
            }

            //SECONDARY DRIVER CONTROLS
            if (currentGamepad2.a) driveMode = Mode.ADRIVE;
            if (currentGamepad2.b ) driveMode = Mode.BSUB;
            if (currentGamepad2.x) driveMode = Mode.XBASKET;
            if (currentGamepad2.y) driveMode = Mode.YBAR;
            if (currentGamepad2.right_bumper) ryanMode = true;
            if (currentGamepad2.left_bumper) ryanMode = false;

            if ( currentGamepad1.right_stick_y < -0.3){
                robot.liftTarget += 20;
            }

            else if (currentGamepad1.right_stick_y  > 0.3 ) {
                robot.liftTarget -= 20;
            }

            //SAFETY AND RULE REQUIRED OVERRIDES

            //reset lift encoder if amps are high
            if (amps > 7) {
                robot.lift.setPower(0);
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftPosition = 0;
                robot.liftTarget = 10;
                robot.lift.setPower(robot.liftPower);
                ampTimer.reset();
            }

            if (amps > 4 && ampTimer.milliseconds() > 3000) {
                robot.lift.setPower(0);
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftPosition = 0;
                robot.liftTarget = 10;
                robot.lift.setPower(robot.liftPower);
                ampTimer.reset();
            }
            else if (amps <= 4){
                ampTimer.reset();
            }

            if (robot.tiltPosition > robot.floor/2){
                robot.liftTarget = Math.min(robot.liftTarget,maxExt);
                robot.claw.setPosition(robot.clawOpen);
            }

            //PRIMARY CONTROLS THAT ARE ACTIVE IN ALL MODES



            //wheel control
            drv  = -currentGamepad1.left_stick_y;
            if (!robot.touch.getState() && drv > 0.2 ) drv = 0.2;
            strafe = currentGamepad1.left_stick_x*.75;
            twist  = currentGamepad1.right_stick_x*.75;



            //prevent rotation of robot when lift control up/down in use
            if (Math.abs(currentGamepad1.right_stick_y)>.4) twist = 0;
            if (ryanMode) speedFactor *= 1.4;

            speeds[0] = (drv + strafe + twist) * speedFactor;
            speeds[1] = (drv - strafe - twist) * speedFactor;
            speeds[2] = (drv - strafe + twist) * speedFactor;
            speeds[3] = (drv + strafe - twist) * speedFactor;

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

            if ((currentGamepad1.right_bumper)
                    && robot.liftPosition < 100 && robot.tiltPosition < 300 ){
                if(robot.grabSpecimen()) {
                    robot.liftTarget = barHeight;
                    robot.lift.setTargetPosition(barHeight);
                    SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(40,-56,-Math.PI/2));
                    runBlocking(new SequentialAction(wall2Bar.build()));
                    robot.touch.getState();
                    robot.forwardTouch();
                }
                else {
                    robot.claw.setPosition(robot.clawOpen);
                }
            }

            if ((currentGamepad1.left_bumper)
                    && Math.abs(robot.liftPosition - barHeight) < 500){
                robot.releaseSpecimen();
                SparkFunOTOSDrive.otos.setPosition(new SparkFunOTOS.Pose2D(0,-26,Math.PI/2));
                runBlocking(new SequentialAction(bar2Wall.build()));
                robot.liftTarget = 0;
                robot.lift.setTargetPosition(robot.liftTarget);
                robot.touch.getState();
                robot.forwardTouch();
            }

            //Swivel the intake
            if (currentGamepad1.dpad_left &&  robot.twistPosition < 1){
                robot.twistPosition += 0.02;
            }

            else if (currentGamepad1.dpad_right && robot.twistPosition > 0){
                robot.twistPosition -= 0.02;
            }

            //fast lift controls
            if (currentGamepad1.x){
                if (robot.liftPosition < barHeight) {
                    robot.liftTarget = barHeight;
                    robot.tiltTarget  = 0;
                    robot.twist.setPosition(1);
                }
                else if (robot.liftPosition > barHeight - 50){
                    robot.liftTarget = robot.maxHeight;
                    robot.twist.setPosition(robot.twistZero);
                }
            }

            //hang control
            if (setHangTime == 0 && getRuntime() - startTime > 60) setHangTime = getRuntime();
            if (currentGamepad1.guide){
                hangPower = 1;
                robot.tilt.setPower(1);
                robot.tiltTarget = 400;
                robot.tilt.setTargetPosition(400);
            }
            else if (currentGamepad1.back){
                hangPower = -1;
            }
            else if (setHangTime != 0 && getRuntime() - setHangTime < 1 )
                hangPower = 0.5;
            else hangPower = 0;

            //rotate tilt using dpad - doesn't prevent tilt from going higher than it should
            double sf = 1;
            if (ryanMode) sf = 1.5;
            if (currentGamepad1.dpad_down) {
                robot.tiltTarget = Math.max(0, robot.tiltTarget - (int)(30*sf));
            }
            else if (currentGamepad1.dpad_up){
                robot.tiltTarget = Math.min(robot.tiltTarget + (int)(30*sf), robot.floor);
            }

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

            if (currentGamepad1.a) {
                robot.liftTarget = 0;
                robot.tiltTarget = tiltVertical;
            }
            if (currentGamepad1.b) robot.tiltTarget = tiltVertical;

            switch  (driveMode){
                //CONTROLS FOR BASIC DRIVE MODE
                case ADRIVE:

                    if (currentGamepad1.b) robot.twist.setPosition(robot.twistZero);

                    //fastest driving speeds
                    if (currentGamepad1.x) {
                        robot.liftTarget = barHeight;
                        robot.tiltTarget = 0;
                        robot.twist.setPosition(robot.twistZero);
                    }

                    speedFactor = .7;
                    break;

                //CONTROLS FOR INTAKE FROM SUBMERSIBLE
                case BSUB:

                    speedFactor = 0.4;
                    //toggle tilt positions quickly
                    if (currentGamepad1.y && !previousGamepad1.y){
                        if (robot.tiltPosition < robot.floor -100) robot.tiltTarget = (int)(robot.floor * .8);
                        else robot.tiltTarget = robot.floor - 100;
                    }
                    //tilt to higher position for exiting submersible
                    if (currentGamepad1.b) robot.tiltTarget = (int)(robot.floor * .6);
                    break;

                //CONTROLS FOR OUTTAKE AT BASKET
                case XBASKET:

                    robot.twist.setPosition(robot.twistZero);
                    speedFactor = 0.4;
                    //tilt back to vertical
                    if (currentGamepad1.b ){
                        robot.tiltTarget = tiltVertical;
                    }

                    break;

                //CONTROLS FOR BAR / SPECIMENS
                case YBAR:

                    speedFactor = 0.4;

                    if (currentGamepad1.left_bumper){
                        robot.releaseSpecimen();
                        robot.liftTarget = 0;
                        robot.tiltTarget = tiltVertical;
                    }

                    //tilt back to vertical
                    if (currentGamepad1.b ){
                        robot.tiltTarget = tiltVertical;
                    }

                    break;
            }

            robot.twist.setPosition(robot.twistPosition);

            if (grab && robot.tiltPosition < 300){
                //grab with claw
                robot.claw.setPosition(robot.clawClosed);
                robot.spinIn();
            }

            if (grab && robot.tiltPosition >300){
                //grab with wheels
                robot.spinIn();
            }
        } //brace ends "while opMode is active" loop
    }  //brace ends runOpMode method
} //brace ends MecOneDrive.java class

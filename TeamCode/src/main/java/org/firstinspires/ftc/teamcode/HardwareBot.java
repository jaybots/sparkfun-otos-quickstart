package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareBot
{
    AnalogInput pixy = null;
    AnalogInput sonar = null;
    double voltage = 0;
    double pixyCenter = 1.85;
    double pixyRange = 0.1;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime sleeper = new ElapsedTime();
    public double flspeed = 0;
    public double frspeed = 0;
    public double blspeed = 0;
    public double brspeed = 0;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor led = null;
    public DcMotorEx  lift = null;
    public DcMotor  tilt = null;
    public DcMotor  hang = null;
    public CRServo spin1 = null;
    public CRServo spin2 = null;
    public Servo claw = null;
    public ServoImplEx twist = null;

    public SparkFunOTOS odo;
    public double clawOpen = 0.2;
    public double clawClosed = 0.58;
    public double liftPower = 1;
    public double tiltPower = 0.7;
    public int liftPosition = 0;
    public int tiltPosition = 0;
    public int liftTarget = 0;
    public int tiltTarget = 0;
    public int floor = 2400;
    public int maxHeight = 3400;
    public double twistZero = 0.52;
    public double twistPosition = 0.52;
    HardwareMap hwMap  =  null;
    SparkFunOTOS.Pose2D pos;


    /* Constructor */
    public HardwareBot(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        sleep(100);

        // Define and Initialize Hardware
        pixy = hwMap.get(AnalogInput.class, "pixy");
        sonar = hwMap.get(AnalogInput.class, "sonar");
        leftFront = hwMap.get(DcMotor.class, "left_front");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        leftBack = hwMap.get(DcMotor.class, "left_back");
        rightBack = hwMap.get(DcMotor.class, "right_back");
        lift = hwMap.get(DcMotorEx.class, "lift");
        led = hwMap.get(DcMotor.class, "led");
        tilt = hwMap.get(DcMotor.class, "tilt");
        hang = hwMap.get(DcMotor.class, "hang");
        spin1 = hwMap.get(CRServo.class, "spin1");
        spin2 = hwMap.get(CRServo.class, "spin2");
        claw = hwMap.get(Servo.class, "claw");
        twist = hwMap.get(ServoImplEx.class, "twist");


        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        tilt.setDirection(DcMotor.Direction.REVERSE);
        hang.setDirection(DcMotor.Direction.REVERSE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        lift.setPower(0);
        tilt.setPower(0);
        hang.setPower(0);
        spin1.setPower(0);
        spin2.setPower(0);
        claw.setPosition(clawClosed);
        led.setPower(0);


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        tilt.setTargetPosition(0);
        tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);




    }
    public void blink(int n) {
        for (int i = 0; i < n; i++){
            led.setPower(0);
            sleep(500);
            led.setPower(.5);
            sleep(500);
        }
    }

    public boolean grabSpecimen() {
        led.setPower(0.5);
        tilt.setTargetPosition(150);
        tiltTarget = 150;
        sleep(200);
        timer.reset();
        pixy.getVoltage();
        double s = 0.25;
        //specimen not visible, return false, do nothing else
        if (pixy.getVoltage() < 0.1) {
            led.setPower(0);
            return false;
        }
        if (pixy.getVoltage() > 0.1 && pixy.getVoltage() < pixyCenter - pixyRange){
            while (pixy.getVoltage() < pixyCenter - pixyRange - 0.4 && timer.milliseconds()<2000){
                leftFront.setPower(s);
                rightFront.setPower(-s);
                leftBack.setPower(-s);
                rightBack.setPower(s);
            }
        }
        else {
            while (pixy.getVoltage() > pixyCenter + pixyRange + 0.3  && timer.milliseconds()<2000){
                leftFront.setPower(-s);
                rightFront.setPower(s);
                leftBack.setPower(s);
                rightBack.setPower(-s);
            }

        }
        forwardTime(.2,300);
        claw.setPosition(clawClosed);
        sleep(400);
        liftTarget = 500;
        lift.setTargetPosition(liftTarget);
        sleep(500);
        tiltTarget = 0;
        tilt.setTargetPosition(tiltTarget);
        if (pixy.getVoltage() < .1){
            claw.setPosition(clawOpen);
            liftTarget = 0;
            lift.setTargetPosition(0);
            led.setPower(0);
            return false;
        }
        led.setPower(0);
        return true;

    }

        /**
         * Lowers lift to clip specimen
         */
        public void releaseSpecimen () {
            lift.setPower(1);
            liftTarget = 0;
            lift.setTargetPosition(0);
            sleep(300);
            claw.setPosition(clawOpen);
            tiltTarget = 150;
            tilt.setTargetPosition(150);
        }

        /**
         * Stops the intake wheels
         */
        public void spinStop () {
            spin1.setPower(0);
            spin2.setPower(0);
        }
        /**
         * Spins the intake wheels to pull block in
         */
        public void spinIn () {
            spin1.setPower(1);
            spin2.setPower(-1);
        }
        /**
         * Spins the intake wheels to push block out
         */
        public void spinOut () {
            spin1.setPower(-1);
            spin2.setPower(1);
        }


        /**
         * Sets power to all drive wheels to zero
         */
        public void stopWheels () {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }

        /**
         * Sets power to all drive wheels to the same value
         * @param power as double
         */
        public void setWheels ( double power){
            leftFront.setPower(power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);
        }

        /**
         * Drives forward using time
         * @param speed as double
         * @param time in milliseconds
         */
        public void forwardTime ( double speed, int time){

            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftBack.setPower(speed);
            rightBack.setPower(speed);
            sleep(time);
            stopWheels();
        }

        /**
         * Drives backward using time
         * @param speed (positive value)
         * @param time in milliseconds
         */
        public void backTime ( double speed, int time){

            leftFront.setPower(-speed);
            rightFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightBack.setPower(-speed);
            sleep(time);
            stopWheels();
        }
        /**
         * Drives right a set amount of time, then stops wheels
         * @param time in milliseconds
         * @param speed as double
         */
        public void rightTime ( double speed, int time){
            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftBack.setPower(-speed);
            rightBack.setPower(speed);
            sleep(time);
            stopWheels();
        }
        /**
         * Drives left a set amount of time, then stops wheels
         * @param time in milliseconds
         * @param speed as double
         */
        public void leftTime (double speed, int time){
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
            leftBack.setPower(speed);
            rightBack.setPower(-speed);
            sleep(time);
            stopWheels();
        }
       /**
         * Just waits a set amount of time
         * All running motors will still be running
         * @param x time in milliseconds
         */
        public void sleep ( int x){
            sleeper.reset();
            while (sleeper.milliseconds() < x) {
            }
        }

        /**
         * Sets the optical odometer to read 0 position (x and y) and zero heading
         */
        public void resetOdo () {
            odo.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
            sleep(50);
        }


}



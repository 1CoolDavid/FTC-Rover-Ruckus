package org.firstinspires.ftc.robotcontroller.internal;

import android.webkit.DownloadListener;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

/**
 * Created by david on 5/14/2018.
 */


public class RoverOpMode extends OpMode {
    final int ROTATION_TICKS = 0; //Ticks per rotation (number that appears)
    final int ROTATION_DISTANCE = 0; //Distance travelled from one rotation (cm)

    //Four Wheel Drive
    DcMotor brightWheel;
    DcMotor bleftWheel;

    //Linear Slide
    DcMotor leftSlide;
    DcMotor rightSlide;

    //Color Sensor
    ColorSensor goldFinder;

    //Fly Wheels
    DcMotor leftFly;
    DcMotor rightFly;

    //Climbing Servo
    Servo latch;
    Servo basket;

    //Fly Wheel Hinge Servo
    Servo hinge;

    @Override
    public void init(){

        brightWheel = hardwareMap.dcMotor.get("Back-Right Wheel");
        bleftWheel = hardwareMap.dcMotor.get("Back-Left Wheel");

        leftSlide = hardwareMap.dcMotor.get("Left Slide");
        rightSlide = hardwareMap.dcMotor.get("Right slide");

        goldFinder = hardwareMap.colorSensor.get("Gold Finder");

        leftFly = hardwareMap.dcMotor.get("Left Fly Wheel");
        rightFly = hardwareMap.dcMotor.get("Right Fly Wheel");

        latch = hardwareMap.servo.get("Climb Latch");
        basket = hardwareMap.servo.get("Climb Basket");

        hinge = hardwareMap.servo.get("Fly Wheel Hinge");
    }

    public void init_loop(){

        //Reset Encoders
        brightWheel.setMode(STOP_AND_RESET_ENCODER);
        bleftWheel.setMode(STOP_AND_RESET_ENCODER);

        //Park Rules (Precaution, may not be necessary)
        bleftWheel.setZeroPowerBehavior(BRAKE);
        brightWheel.setZeroPowerBehavior(BRAKE);

        //Fly Wheel Reset
        leftFly.setMode(STOP_AND_RESET_ENCODER);
        rightFly.setMode(STOP_AND_RESET_ENCODER);

        //Lift Mechanism Initialization
        leftSlide.setMode(RUN_WITHOUT_ENCODER);
        rightSlide.setMode(RUN_WITHOUT_ENCODER);

        //Fly Wheels Initialization
        leftFly.setMode(RUN_USING_ENCODER);  //Encoders may not be needed
        rightFly.setMode(RUN_USING_ENCODER); //Encoders may not be needed

        //Encoder Initialization
        brightWheel.setMode(RUN_USING_ENCODER);
        bleftWheel.setMode(RUN_USING_ENCODER);

    }
    public void loop(){

        //Four Wheel Drive
        brightWheel.setPower(gamepad1.right_stick_y * 1); //may be flipped
        bleftWheel.setPower(gamepad1.left_stick_y * -1); //may be flipped

        if(gamepad1.right_bumper){
            rightSlide.setPower(.8);
            leftSlide.setPower(.8);
        }

        if(gamepad1.left_bumper){
            rightSlide.setPower(-.8);
            leftSlide.setPower(-.8);
        }

        if(gamepad1.dpad_up){
            rightFly.setPower(-1);
            leftFly.setPower(-1);
        }

        if(gamepad1.dpad_down){
            rightFly.setPower(1);
            leftFly.setPower(1);
        }

        //Debugging & encoder measuring purposes
        telemetry.addData("Back-Left Wheel", bleftWheel.getCurrentPosition());
        telemetry.addData("Back-Right Wheel", brightWheel.getCurrentPosition());
    }

    /**
     * Sets target position for motors to current postion
     */
    public void stopMotors(){

        bleftWheel.setPower(0);
        brightWheel.setPower(0);

    }

    /**
     * Drives forward a specific amount (in centimeters)
     * @param cm is the distance requested in centimeters
     * @param speed is how fast the motors will spin until distance is reached
     */
    public void moveForward(int cm, double speed){

        int bleftTarget = getBackLeftTarget(cm);
        int brightTarget = getBackRightTarget(cm);

        startDrive(bleftTarget, brightTarget, speed);
    }


    /**
     * Sets encoders to run to target position located in @param.
     * //@param fleftTarget front left wheel target
     * //@param frightTarget front right wheel target
     * @param bleftTarget back left wheel target
     * @param brightTarget back right wheel target
     */
    public void startDrive(int bleftTarget, int brightTarget, double speed){

        bleftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bleftWheel.setTargetPosition(bleftTarget);
        bleftWheel.setPower(speed);

        brightWheel.setTargetPosition(brightTarget);
        brightWheel.setPower(speed);

        while(isDriving()){
            //Filler for motors to run
        }

        stopMotors();
    }

    /**
     * Calculates target position for back-right motor by dividing the wanted distance by the distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getBackRightTarget(int cm){

        return ((cm/ROTATION_DISTANCE)*ROTATION_TICKS)+brightWheel.getCurrentPosition();

    }
    /**
     * Calculates target position for back-left motor by dividing the wanted distance by the distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getBackLeftTarget(int cm){

        return ((cm/ROTATION_DISTANCE)*ROTATION_TICKS)+bleftWheel.getCurrentPosition();

    }

    /**
     * Checks is motors are currently trying to reach a target
     * @return true if motors are in active target run
     */
    public boolean isDriving(){

        return bleftWheel.isBusy() && brightWheel.isBusy();

    }



}

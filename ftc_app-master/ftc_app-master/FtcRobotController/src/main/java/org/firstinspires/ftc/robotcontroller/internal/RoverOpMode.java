package org.firstinspires.ftc.robotcontroller.internal;

import android.webkit.DownloadListener;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    /*public enum State{
        Up,
        Down,
        Left,
        Right,
    }*/
    //Reference for later


    //Four Wheel Drive
    DcMotor fleftWheel;
    DcMotor frightWheel;
    DcMotor brightWheel;
    DcMotor bleftWheel;

    //Linear Slide
    DcMotor leftSlide;
    DcMotor rightSlide;

    //Fly Wheels
    DcMotor leftFly;
    DcMotor rightFly;

    //Climbing Servo
    Servo latch;

    //Fly Wheel Hinge Servo
    Servo hinge;

    @Override
    public void init(){
        //Four Wheel Drive
        fleftWheel = hardwareMap.dcMotor.get("Front-Left Wheel");
        frightWheel = hardwareMap.dcMotor.get("Front-Right Wheel");
        brightWheel = hardwareMap.dcMotor.get("Back-Right Wheel");
        bleftWheel = hardwareMap.dcMotor.get("Back-Left Wheel");

        leftSlide = hardwareMap.dcMotor.get("Left Slide");
        rightSlide = hardwareMap.dcMotor.get("Right slide");

        leftFly = hardwareMap.dcMotor.get("Left Fly Wheel");
        rightFly = hardwareMap.dcMotor.get("Right Fly Wheel");

        latch = hardwareMap.servo.get("Climb Latch");

        hinge = hardwareMap.servo.get("Fly Wheel Hinge");
    }

    public void init_loop(){
        //Four Wheel Drive Reset
        fleftWheel.setMode(STOP_AND_RESET_ENCODER);
        frightWheel.setMode(STOP_AND_RESET_ENCODER);
        brightWheel.setMode(STOP_AND_RESET_ENCODER);
        bleftWheel.setMode(STOP_AND_RESET_ENCODER);

        //Four Wheel Drive Park Rules (Precaution, may not be necessary)
        fleftWheel.setZeroPowerBehavior(BRAKE);
        frightWheel.setZeroPowerBehavior(BRAKE);
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

        //Four Wheel Drive Encoder Initialization
        fleftWheel.setMode(RUN_USING_ENCODER);
        frightWheel.setMode(RUN_USING_ENCODER);
        brightWheel.setMode(RUN_USING_ENCODER);
        bleftWheel.setMode(RUN_USING_ENCODER);

    }
    public void loop(){

        //Four Wheel Drive
        brightWheel.setPower(gamepad1.right_stick_y * 1); //may be flipped
        frightWheel.setPower(gamepad1.right_stick_y * 1); //may be flipped
        bleftWheel.setPower(gamepad1.left_stick_y * -1); //may be flipped
        fleftWheel.setPower(gamepad1.left_stick_y * -1); //may be flipped

        if(gamepad1.right_bumper){
            rightSlide.setPower(.8);
            leftSlide.setPower(.8);
        }

        if(gamepad1.left_bumper){
            rightSlide.setPower(-.8);
            leftSlide.setPower(-.8);
        }

        if(gamepad1.dpad_up){
            rightFly.setPower(1);
            leftFly.setPower(1);
        }

        if(gamepad1.dpad_down){
            rightFly.setPower(-1);
            leftFly.setPower(-1);
        }

        //Debugging & encoder measuring purposes
        telemetry.addData("Front-Left Wheel", fleftWheel.getCurrentPosition());
        telemetry.addData("Front-Right Wheel", frightWheel.getCurrentPosition());
        telemetry.addData("Back-Left Wheel", bleftWheel.getCurrentPosition());
        telemetry.addData("Back-Right Wheel", brightWheel.getCurrentPosition());
    }

    /**
     * Sets target position for motors to current postion
     */
    public void stopMotors(){

        fleftWheel.setPower(0);
        frightWheel.setPower(0);
        bleftWheel.setPower(0);
        brightWheel.setPower(0);

    }

    /**
     * Drives forward a specific amount (in centimeters)
     * @param cm is the distance requested in centimeters
     * @param speed is how fast the motors will spin until distance is reached
     */
    public void moveForward(int cm, double speed){

        int fleftTarget = getFrontLeftTarget(cm);
        int frightTarget = getFrontRightTarget(cm);
        int bleftTarget = getBackLeftTarget(cm);
        int brightTarget = getBackRightTarget(cm);

        startDrive(fleftTarget, frightTarget, bleftTarget, brightTarget, speed);
    }


    /**
     * Sets encoders to run to target position located in @param.
     * @param fleftTarget front left wheel target
     * @param frightTarget front right wheel target
     * @param bleftTarget back left wheel target
     * @param brightTarget back right wheel target
     */
    public void startDrive(int fleftTarget, int frightTarget, int bleftTarget, int brightTarget, double speed){

        fleftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bleftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        fleftWheel.setTargetPosition(fleftTarget);
        fleftWheel.setPower(speed);

        frightWheel.setTargetPosition(frightTarget);
        frightWheel.setPower(speed);

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
     * Calculates target position for front-left motor by dividing the wanted distance by the distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getFrontLeftTarget(int cm){

        return ((cm/ROTATION_DISTANCE)*ROTATION_TICKS)+fleftWheel.getCurrentPosition();

    }
    /**
     * Calculates target position for front-right motor by dividing the wanted distance by the distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getFrontRightTarget(int cm){

        return ((cm/ROTATION_DISTANCE)*ROTATION_TICKS)+frightWheel.getCurrentPosition();

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

        return fleftWheel.isBusy() && frightWheel.isBusy() && bleftWheel.isBusy() && brightWheel.isBusy();

    }



}

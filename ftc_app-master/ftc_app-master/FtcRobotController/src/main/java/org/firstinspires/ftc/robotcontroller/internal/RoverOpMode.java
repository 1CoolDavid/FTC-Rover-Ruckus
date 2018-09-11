package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by david on 5/14/2018.
 */

public class RoverOpMode extends OpMode {
    final int ROTATION_TICKS = 0; //Ticks per rotation (number that appears)
    final int ROTATION_DISTANCE = 0; //Distance travelled from one rotation (cm)


    int currentPos;
    //Four Wheel Drive
    DcMotor fleftWheel;
    DcMotor frightWheel;
    DcMotor brightWheel;
    DcMotor bleftWheel;

    //Linear Slide
    DcMotor leftSlide;
    DcMotor rightSlide;


    @Override
    public void init(){
        //Four Wheel Drive
        fleftWheel = hardwareMap.dcMotor.get("Front-Left Wheel");
        frightWheel = hardwareMap.dcMotor.get("Front-Right Wheel");
        brightWheel = hardwareMap.dcMotor.get("Back-Right Wheel");
        bleftWheel = hardwareMap.dcMotor.get("Back-Left Wheel");

        leftSlide = hardwareMap.dcMotor.get("Left Slide");
        rightSlide = hardwareMap.dcMotor.get("Right slide");

    }

    public void init_loop(){
        //Four Wheel Drive
        fleftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bleftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        if(gamepad1.left_bumper) {
            rightSlide.setPower(-.8);
            leftSlide.setPower(-.8);
        }
        telemetry.addData("Front-Left Wheel", fleftWheel.getCurrentPosition());
        telemetry.addData("Front-Right Wheel", frightWheel.getCurrentPosition());
    }

    /**
     * Sets target position for motors to current postion
     */
    public void stopMotors(){
        int leftTarget = getLeftTarget(0);
        int rightTarget = getRightTarget(0);
        fleftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fleftWheel.setTargetPosition(leftTarget);
        frightWheel.setTargetPosition(rightTarget);
    }

    /**
     * Sets target position relative to distance wanted to travel
     * @param cm is the distance requested in centimeters
     */
    public void moveForward(int cm){
        int leftTarget = getLeftTarget(cm);
        int rightTarget = getRightTarget(cm);
        fleftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fleftWheel.setTargetPosition(leftTarget);
        frightWheel.setTargetPosition(rightTarget);
    }

    /**
     * Calculates target position for left motor by dividing the wanted distance by the distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getLeftTarget(int cm){
        return ((cm/ROTATION_DISTANCE)*ROTATION_TICKS)+fleftWheel.getCurrentPosition();
    }
    /**
     * Calculates target position for right motor by dividing the wanted distance by the distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getRightTarget(int cm){
        return ((cm/ROTATION_DISTANCE)*ROTATION_TICKS)+frightWheel.getCurrentPosition();
    }



}

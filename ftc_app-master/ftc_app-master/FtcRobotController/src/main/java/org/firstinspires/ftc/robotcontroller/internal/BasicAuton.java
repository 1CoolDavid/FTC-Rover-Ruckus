package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by david on 5/14/2018.
 */

public class BasicAuton extends OpMode {
    DcMotor fleftWheel;
    DcMotor frightWheel;
    DcMotor brightWheel;
    DcMotor bleftWheel;

    //Two Wheel Drive
    DcMotor rightWheel;
    DcMotor leftWheel;

    //Servos
    Servo collector;
    Servo arm;

    @Override
    public void init() {
        //Four Wheel Drive
        fleftWheel = hardwareMap.dcMotor.get("Front-Left Wheel");
        frightWheel = hardwareMap.dcMotor.get("Front-Right Wheel");
        brightWheel = hardwareMap.dcMotor.get("Back-Right Wheel");
        bleftWheel = hardwareMap.dcMotor.get("Back-Left Wheel");
        //Two Wheel Drive
        rightWheel = hardwareMap.dcMotor.get("Right Wheel");
        leftWheel = hardwareMap.dcMotor.get("Left Wheel");
        //Servos
        collector = hardwareMap.servo.get("collector");
        arm = hardwareMap.servo.get("arm");

        //Setting servo initial positions
        arm.setPosition(0);
        collector.setPosition(.5);
    }
    public void init_loop(){
        super.init_loop();
        //Four Wheel Drive
        fleftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bleftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Two Wheel Drive
        rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void loop(){

    }
    public void driveStriaghtFour(double meters){
        final double CONSTANT = 3.5; //theory
        fleftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int leftPos = fleftWheel.getCurrentPosition();
        int rightPos = rightWheel.getCurrentPosition();
        int newLeftPos = leftPos + (int)(CONSTANT*meters);
        int newRightPos = rightPos + (int)(CONSTANT*meters);
        telemetry.addData("Init Moving Forward", newLeftPos + newRightPos);
        fleftWheel.setTargetPosition(newLeftPos);
        frightWheel.setTargetPosition(newRightPos);
        telemetry.addData("End Moving Forward", newLeftPos + newRightPos);
    }
    public void driveStraightTwo(double meters){
        final double CONSTANT = 3.7; //theory
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int leftPos = leftWheel.getCurrentPosition();
        int rightPos = rightWheel.getCurrentPosition();
        int newLeftPos = leftPos + (int)(CONSTANT*meters);
        int newRightPos = rightPos + (int)(CONSTANT*meters);
        telemetry.addData("Init Moving Forward", newLeftPos + newRightPos);
        leftWheel.setTargetPosition(newLeftPos);
        rightWheel.setTargetPosition(newRightPos);
        telemetry.addData("End Moving Forward", newLeftPos + newRightPos);
    }
}

package org.firstinspires.ftc.robotcontroller.internal;

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


    //Four Wheel Drive
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
    Servo basket;

    //Fly Wheel Hinge Servo
    Servo hinge;

    @Override
    public void init(){

        brightWheel = hardwareMap.dcMotor.get("Back-Right Wheel");
        bleftWheel = hardwareMap.dcMotor.get("Back-Left Wheel");

        leftSlide = hardwareMap.dcMotor.get("Left Slide");
        rightSlide = hardwareMap.dcMotor.get("Right slide");

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

        //Lift Mechanism Reset
        leftSlide.setMode(STOP_AND_RESET_ENCODER);
        rightSlide.setMode(STOP_AND_RESET_ENCODER);

        //Park Rules (Precaution, may not be necessary)
        leftSlide.setZeroPowerBehavior(BRAKE);
        rightSlide.setZeroPowerBehavior(BRAKE);

        //Fly Wheels Initialization
        leftFly.setMode(RUN_WITHOUT_ENCODER);
        rightFly.setMode(RUN_WITHOUT_ENCODER);

        //Encoder Initialization
        brightWheel.setMode(RUN_USING_ENCODER);
        bleftWheel.setMode(RUN_USING_ENCODER);

        //Encoder Initialization
        leftSlide.setMode(RUN_USING_ENCODER);
        rightSlide.setMode(RUN_USING_ENCODER);

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

}

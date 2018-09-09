package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by david on 5/14/2018.
 */

public class RoverOpMode extends OpMode {
    //Four Wheel Drive
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
    public void init(){
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
        //Two Wheel Drive
        rightWheel.setPower(gamepad1.right_stick_y * 1); //may need to be flipped
        leftWheel.setPower(gamepad1.left_stick_y * -1); //may need to be flipped

        //Four Wheel Drive
        brightWheel.setPower(gamepad1.right_stick_y * 1); //may be flipped
        frightWheel.setPower(gamepad1.right_stick_y * 1); //may be flipped
        bleftWheel.setPower(gamepad1.left_stick_y * -1); //may be flipped
        brightWheel.setPower(gamepad1.left_stick_y * -1); //may be flipped

        //servo basic code
        if(gamepad1.a) //activate
            collector.setPosition(0);
        if (gamepad1.x) //reset
            collector.setPosition(.5);
        if (gamepad1.b) //activate
            arm.setPosition(.5);
        if (gamepad1.y) //reset
            arm.setPosition(0);
    }
}

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RoverOpMode extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

    //Servo basket;

    public void init(){
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        //basket = hardwareMap.servo.get("basket");
    }

    public void init_loop() {

    }

    @Override
    public void loop(){

        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);

        /*if(gamepad1.a){
            basket.setPosition(.9);
        }*/
    }
}

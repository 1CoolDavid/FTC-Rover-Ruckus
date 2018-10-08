package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RoverOpMode extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFly;
    DcMotor rightFly;



    //Servo basket;

    public void init(){
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        leftFly = hardwareMap.dcMotor.get("Left Flywheel");
        rightFly = hardwareMap.dcMotor.get("Right Flywheel");
        //basket = hardwareMap.servo.get("basket");
    }

    public void init_loop() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){

        leftMotor.setPower(gamepad1.left_stick_y * -1);
        rightMotor.setPower(gamepad1.right_stick_y);
        telemetry.addData("Left Position",leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());
        if(gamepad1.a){
            leftFly.setPower(1);
            rightFly.setPower(-1);
        }

    }

}

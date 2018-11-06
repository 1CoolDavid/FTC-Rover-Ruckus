package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

public class RoverOpMode extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFly;
    DcMotor rightFly;
    DcMotor spool;
    ColorSensor sensor;
    ColorSensor sensor2;
    Servo rightArm;
    Servo leftArm;
    double speed = 0.7;


    //Servo basket;

    public void init(){
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
       // leftFly = hardwareMap.dcMotor.get("Left Flywheel");
       // rightFly = hardwareMap.dcMotor.get("Right Flywheel");
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        sensor = hardwareMap.colorSensor.get("sensor");
        sensor2 = hardwareMap.colorSensor.get("sensor2");
        //spool = hardwareMap.dcMotor.get("spool");
        //basket = hardwareMap.servo.get("basket");
    }

    public void init_loop() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //spool.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void loop(){  

        leftMotor.setPower(gamepad1.left_stick_y * -0.5);
        rightMotor.setPower(gamepad1.right_stick_y * 0.5);
        //spool.setPower(gamepad1.right_trigger);
        //spool.setPower(gamepad1.left_trigger*-1);
        telemetry.addData("Left Position",leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());
        while(gamepad1.a){
            leftFly.setPower(1);
            rightFly.setPower(-1);
        }
        if(gamepad1.b){
            leftFly.setPower(0);
            rightFly.setPower(0);
        }
        if(gamepad1.y){
            speed = (speed == 0.7) ? 1 : 0.7;
        }
        if(gamepad1.dpad_up){
            leftArm.setPosition(0);
            rightArm.setPosition(.8);
        }
        if(gamepad1.dpad_down){
            leftArm.setPosition(1);
            rightArm.setPosition(0);
        }


        telemetry.addData("left arm position", leftArm.getPosition());
        telemetry.addData("right arm position", rightArm.getPosition());
        telemetry.update();

    }

  /*  public int goldFound(){
        //1 is left, 2 is middle, 3 is right
        if(sensor.red() > sensor.blue() && sensor.red() > sensor.green() && Math.abs(sensor.green() - sensor.red()) > 10){
            return 1;
        }
        if(sensor2.red() > sensor2.blue() && sensor2.red() > sensor2.green() && Math.abs(sensor2.green() - sensor2.red()) > 10){
            return 3;
        }
        return 2;
    }
    */

}

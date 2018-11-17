package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.BufferedReader;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

public class RoverOpMode extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftFly;
    DcMotor rightFly;
    DcMotor liftFlywheel;
    DcMotor spool;
    boolean in;
    boolean out;
    ColorSensor sensor;
    ColorSensor sensor2;
    Servo rightArm;
    Servo leftArm;
    double speed = .8;


    //Servo basket;

    public void init(){
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        leftFly = hardwareMap.dcMotor.get("Left Flywheel");
        rightFly = hardwareMap.dcMotor.get("Right Flywheel");
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        sensor = hardwareMap.colorSensor.get("sensor");
        sensor2 = hardwareMap.colorSensor.get("sensor2");
        liftFlywheel = hardwareMap.dcMotor.get("liftFly");
        boolean in =false;
        boolean out = false;
        //spool = hardwareMap.dcMotor.get("spool");
        //basket = hardwareMap.servo.get("basket");
    }

    public void init_loop() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftFlywheel.setZeroPowerBehavior(BRAKE);
        leftFly.setZeroPowerBehavior(FLOAT);
        rightFly.setZeroPowerBehavior(FLOAT);
        //spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //spool.setZeroPowerBehavior(BRAKE);
    }

    @Override
    public void loop(){  

        leftMotor.setPower(gamepad1.left_stick_y * -speed);
        rightMotor.setPower(gamepad1.right_stick_y * speed);
        telemetry.addData("Left Position",leftMotor.getCurrentPosition());
        telemetry.addData("Right Position", rightMotor.getCurrentPosition());

        if(gamepad1.b){
            speed = speed == 0.8 ? 1 : 0.8;
            sleep(500);
        }
        telemetry.addData("Speed", speed);

        if(gamepad1.dpad_left){
            leftArm.setPosition(0);
            rightArm.setPosition(1);
        }

        if(gamepad1.dpad_right){
            leftArm.setPosition(1);
            rightArm.setPosition(0);
        }

        if(gamepad1.a){
            in = !in;
            if(out)
                out=!out;
            sleep(250);
        }
        if(gamepad1.y){
            out = !out;
            if(in)
                in=!in;
            sleep(250);
        }
        /*if(gamepad1.dpad_up) {
            liftFlywheel.setPower(-0.4);
            sleep(900);
        }*/

        telemetry.addData("out", out);
        telemetry.addData("in", in);

        if(in) {
            leftFly.setPower(1);
            rightFly.setPower(-1);
        }
        else if(out) {
            leftFly.setPower(-1);
            rightFly.setPower(1);
        }
        else{
            leftFly.setPower(0);
            rightFly.setPower(0);
        }

        if(gamepad1.left_bumper){
            liftFlywheel.setPower(.4);
        }
            
        if(gamepad1.right_bumper){
            liftFlywheel.setPower(-.4);
        }

        if(!gamepad1.left_bumper && !gamepad1.right_bumper){
            liftFlywheel.setPower(0);
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
  public void sleep(long millis) {
      try {
          Thread.sleep(millis);
      } catch (Exception e) {
          System.out.print(e.getStackTrace());
      }
  }
}

package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class SquareAuton extends OpMode {

    final double pi = Math.PI;
    int step = 0;
    Servo rightArm;
    Servo leftArm;
    DcMotor leftFly;
    DcMotor rightFly;
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    ColorSensor sensor;
    ColorSensor sensor2;

    @Override
    public void init() {
    //    leftFly = hardwareMap.dcMotor.get("Left Flywheel");
      //  rightFly = hardwareMap.dcMotor.get("Right Flywheel");
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        sensor = hardwareMap.colorSensor.get("sensor");
        sensor2 = hardwareMap.colorSensor.get("sensor2");
        rightArm = hardwareMap.servo.get("rightArm");
        leftArm = hardwareMap.servo.get("leftArm");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void init_loop() {

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(){ //ai is just if statements; this is just if statements; therefore this is ai
        if(step == 0)
            driveForward(6);
        if(step == 1)
            turnRight(60);
        if(step == 2)
            driveBackward(6);
        if(step == 3)
            turnLeft(20);
        if(step == 4)
            driveForward(1);
        if(step == 5)
            turnRight(20);
        if(step == 6)
            driveBackward(1);
        if(step == 7){
            moveSensors(true, true);
        }
        if(goldFound() == 2 && step == 8){
            goldOnCenter();
            moveSensors(true, false);
            turnLeft(110);
            driveForward(25);
            //FlyOut();
        }
        else if (goldFound() == 1 && step == 8){
            goldOnLeft();
            moveSensors(false, true);
            turnRight(90);
            driveForward(17);
            turnRight(45);
            driveForward(10);
          //  FlyOut();
        }
        else if (goldFound() == 3 && step == 8){
            goldOnRight();
            moveSensors(true, false);
            turnLeft(90);
            driveForward(17);
            turnLeft(45);
            driveForward(10);
        //    FlyOut();
        }

        stopMotors();
        sleep(500);
        step++;
    }

    public void goldOnLeft(){
        //lift servo2
        driveBackward(17);
    }

    public void goldOnRight(){
        //lift servo1
        driveForward(17);
    }

    public void goldOnCenter(){
        turnRight(25); //may need adjusting

    }

 /*   public void stopFly(){
        leftFly.setPower(0);
        rightFly.setPower(0);
    }
    */

    /*public void FlyOut(){
        leftFly.setPower(-1);
        rightFly.setPower(1);
    }*/

    public void print(String s1, String s2) {
        telemetry.addData(s1,s2);
    }

    public void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {
            System.out.print(e.getStackTrace());
        }
    }

    public void stopMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void turnRight(long degrees){
        rightMotor.setPower(0);
        rightMotor.setPower(-0.5);
        leftMotor.setPower(0.5);
        sleep(((long)9.2)*degrees);
        stopMotors();
    }

    public void turnLeft(long degrees){
        rightMotor.setPower(0.5);
        leftMotor.setPower(-0.5);
        sleep(((long)9.2)*degrees);
        stopMotors();
    }

    public void driveForward(int in){
        //1 in = 34.5 milliseconds at speed 0.5
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        sleep(30 * in); //real value 30
    }

    /*public void driveForward(long in){
        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);
        sleep(((long)30) * in);
    }*/

    public void driveBackward(int in){
        //1 in = 34.5 milliseconds at speed 0.5
        leftMotor.setPower(-0.5);
        rightMotor.setPower(-0.5);
        sleep(30 * in); //real value 30
    }

    public int goldFound(){
        //1 is left, 2 is middle, 3 is right
        if(sensor.red() > sensor.blue() && sensor.red() > sensor.green() && Math.abs(sensor.green() - sensor.red()) > 10){
            return 1;
        }
        if(sensor2.red() > sensor2.blue() && sensor2.red() > sensor2.green() && Math.abs(sensor2.green() - sensor2.red()) > 10){
            return 3;
        }
        return 2;
    }
    public void moveSensors(boolean downOne, boolean downTwo){
       if(downOne == false){ //up
           leftArm.setPosition(0);
           leftArm.setPosition(0);
       }
       else{ //down
           leftArm.setPosition(1);
           leftArm.setPosition(1);
       }
       if(downTwo == false){ //up
           rightArm.setPosition(.8);
           rightArm.setPosition(.8);
       }
       else{ //down
           rightArm.setPosition(0);
           rightArm.setPosition(0);
       }

    }
}

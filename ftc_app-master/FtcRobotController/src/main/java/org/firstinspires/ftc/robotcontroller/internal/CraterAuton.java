package org.firstinspires.ftc.robotcontroller.internal;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.sql.Time;

@Autonomous
public class CraterAuton extends OpMode {
    final double pi = Math.PI;
    int step = 0;
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    ColorSensor sensor;
    ColorSensor sensor2;

    @Override
    public void init(){
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        sensor = hardwareMap.colorSensor.get("sensor");
        sensor2 = hardwareMap.colorSensor.get("sensor2");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){
        if(step == 0)
         driveForward(35);
        if(step == 1)
            stopMotors();
       step++;
    }

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

 /*  public void driveForward(long in){
       leftMotor.setPower(0.5);
       rightMotor.setPower(0.5);
       sleep(((long)30) * in);
   }*/

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
}

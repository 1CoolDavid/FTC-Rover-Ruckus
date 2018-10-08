package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.sql.Time;

@Autonomous
public class RoverAuton extends OpMode {
    final double rightTicks = -720;
    final double leftTicks = 720; //only 1440 for 4 wheel drive
    final double turnTicks = 720;
    final double pi = Math.PI;
    final double circum = 4 * pi;
    final double turnDegree = 60;
    int step = 0;
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init(){
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
    }

    @Override
    public void init_loop() {
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){
        if(step == 0)
            turn(45);
        step++;
    }

    public int calculateLeftTurn(int degrees){
        return (int)((degrees/turnDegree)*turnTicks + leftMotor.getCurrentPosition());
    }

    public int calculateRightTurn(int degrees){
        return (int)((degrees/turnDegree)*turnTicks + rightMotor.getCurrentPosition());
    }

    public int calculateLeftTicks(double in){
        int target = (int)((in/circum)*leftTicks + leftMotor.getCurrentPosition());

        return target;
    }
    public int calculateRightTicks(double in){
        int target = (int)((in/circum)*rightTicks + rightMotor.getCurrentPosition());

        return target;
    }

    public void stopMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void turn(int degrees){

        int leftTarget = calculateLeftTurn(degrees);
        int rightTarget = calculateRightTurn(degrees);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(leftTarget);
        rightMotor.setTargetPosition(rightTarget);

        leftMotor.setPower(1);
        rightMotor.setPower(1);

    }

    public void driveForward(double left, double right){
        //Create targets for ticks
        int leftTarget = calculateLeftTicks(left);
        int rightTarget = calculateRightTicks(right);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(leftTarget);
        rightMotor.setTargetPosition(rightTarget);

        leftMotor.setPower(1);
        rightMotor.setPower(1);
        telemetry.addData("Left Target", leftTarget);
        telemetry.addData("Right Target", rightTarget);

    }
}

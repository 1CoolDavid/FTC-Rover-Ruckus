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

    //TODO: Servo Controls - Latch, basket, and hinge (Assign Buttons)

    final static int WHEEL_ROTATION_TICKS = 1440; //Ticks per wheel rotation (number that appears) | could be 1000
    final static int WHEEL_ROTATION_DISTANCE = 19; //Distance travelled from one wheel rotation (in) | rounded up 6*pi
    final static int WHEEL_ROTATION_ANGLE = 0; //Angle created with one wheel making one rotation forward while the other goes backwards
    final static int LIFT_ROTATION_TICKS = 1440; //Ticks per lift motor rotation | could be 1000
    final static int LIFT_DISTANCE = 0; //Distance up per rotation
    final static int COLORHEX = 160;

    //Servo measurements
    final static double LEFT_SENSOR_LOWER_POS = 0; //Left color sensor servo unfolding position
    final static double LEFT_SENSOR_INITIAL_POS = 0; //Left color sensor servo initial position
    final static double RIGHT_SENSOR_LOWER_POS = 0; //Right color sensor servo unfolding position
    final static double RIGHT_SENSOR_INITIAL_POS = 0; //Right color sensor servo initial position
    final static double HINGE_INITIAL_POS = 0; //Hinge servo initial position
    final static double HINGE_LOWER_POS = 0; //Hing servo unfolded position
    final static double LATCHED = 0; //Latching servo close measurement
    final static double UNLATCHED = 0; //Latching servo open measurement

    //States throughout the game
    public enum State{
        PARK,
        GOLD,
        LOWER,
        MAKER,
        FAILED_GOLD
    }

    //States when finding gold
    public enum Gold{
        LEFT,
        MIDDLE,
        RIGHT,
        UNKNOWN
    }

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

        if(gamepad1.a){
            if(latch.getPosition() == UNLATCHED)
                latch.setPosition(LATCHED);
            else
                latch.setPosition(UNLATCHED);
        }

        if(gamepad1.b){
            if(hinge.getPosition() == HINGE_INITIAL_POS)
                hinge.setPosition(HINGE_LOWER_POS);
            else
                hinge.setPosition(HINGE_INITIAL_POS);
        }

        //Add basket code here ------


        //Debugging & encoder measuring purposes
        telemetry.addData("Back-Left Wheel", bleftWheel.getCurrentPosition());
        telemetry.addData("Back-Right Wheel", brightWheel.getCurrentPosition());
    }

}

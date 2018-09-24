package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.COLORHEX;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.Gold;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.HINGE_INITIAL_POS;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.HINGE_UNFOLD_POS;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.LIFT_DISTANCE;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.LIFT_ROTATION_TICKS;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.LATCHED;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.LEFT_SENSOR_INITIAL_POS;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.LEFT_SENSOR_UNFOLD_POS;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.RIGHT_SENSOR_INITIAL_POS;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.RIGHT_SENSOR_UNFOLD_POS;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.State;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.UNLATCHED;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.WHEEL_ROTATION_ANGLE;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.WHEEL_ROTATION_DISTANCE;
import static org.firstinspires.ftc.robotcontroller.internal.RoverOpMode.WHEEL_ROTATION_TICKS;

//Test class for now
public class RoverAuton extends OpMode{

    //TODO: Create Java Docs for servo methods

    boolean isLeftGrey;
    boolean isRightGrey;

    //Vision processing Sensors and hardware
    ColorSensor leftSensor;
    ColorSensor rightSensor;
    Servo leftColor;
    Servo rightColor;

    //Climbing mech
    Servo latch;
    DcMotor leftSlide;
    DcMotor rightSlide;

    //Fly Wheel mech
    Servo hinge;
    DcMotor leftFly;
    DcMotor rightFly;

    //Driving
    DcMotor bleftWheel;
    DcMotor brightWheel;



    State current = State.LOWER;
    Gold locate = Gold.UNKNOWN;

    public void init(){

        leftSensor = hardwareMap.colorSensor.get("leftSensor");
        rightSensor = hardwareMap.colorSensor.get("rightSensor");
        leftColor = hardwareMap.servo.get("Left Color");
        rightColor = hardwareMap.servo.get("Right Color");

        latch = hardwareMap.servo.get("Climb Latch");
        leftSlide = hardwareMap.dcMotor.get("Left Slide");
        rightSlide = hardwareMap.dcMotor.get("Right Slide");

        hinge = hardwareMap.servo.get("Fly Wheel Hinge");
        leftFly = hardwareMap.dcMotor.get("Left Fly Wheel");
        rightFly = hardwareMap.dcMotor.get("Right Fly Wheel");

        brightWheel = hardwareMap.dcMotor.get("Back-Right Wheel");
        bleftWheel = hardwareMap.dcMotor.get("Back-Left Wheel");

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

        if(current == State.LOWER){

            startLift(-10, -10, .4 );
            current = State.GOLD;

        }
        else if(current == State.GOLD){

            //Checking white-ish grey
            isLeftGrey = leftSensor.blue() >= COLORHEX && leftSensor.red() >= COLORHEX && leftSensor.green() >= COLORHEX;
            isRightGrey = rightSensor.blue() >= COLORHEX && rightSensor.red() >= COLORHEX  && rightSensor.green() >= COLORHEX;

            locate = isLeftGrey & isRightGrey ? Gold.MIDDLE : isLeftGrey && !isRightGrey ? Gold.LEFT : !isLeftGrey && isRightGrey ? Gold.RIGHT : Gold.UNKNOWN;

            if(locate == Gold.UNKNOWN){

                current = State.FAILED_GOLD;

            }
            else if(locate == Gold.RIGHT){

            }
            else if(locate == Gold.MIDDLE){

            }
            else if(locate == Gold.LEFT){

            }

        }
        else if(current == State.FAILED_GOLD){

        }
        else if(current == State.MAKER){

        }
        else if(current == State.PARK){

        }



    }

    /**
     * Sets drive motor power to 0 -- brakes
     */
    public void stopMotors(){

        bleftWheel.setPower(0);
        brightWheel.setPower(0);

    }

    /**
     * Sets lift motor power to 0 -- brakes
     */
    public void stopLift(){

        leftSlide.setPower(0);
        rightSlide.setPower(0);

    }

    /**
     * Drives forward a specific amount (in centimeters)
     * @param cm is the distance requested in centimeters
     * @param speed is how fast the motors will spin until distance is reached
     */
    public void moveForward(int cm, double speed){

        int bleftTarget = getBackLeftTarget(cm);
        int brightTarget = getBackRightTarget(cm);

        startDrive(bleftTarget, brightTarget, speed);

    }

    /**
     * Turns robot left a specific angle (degrees)
     * @param angle the requested angle in degrees
     * @param speed the speed at which the turn is made
     */
    public void turnLeft(int angle, double speed){

        int leftAngleTarget = getLeftAngleTarget(-angle);
        int rightAngleTarget = getRightAngleTarget(angle);

        startDrive(leftAngleTarget, rightAngleTarget, speed);

    }

    /**
     * Turns the robot right a specifc angle (degrees)
     * @param angle the requested angle in degrees
     * @param speed the speed at which teh turn is made
     */
    public void turnRight(int angle, double speed){

        int leftAngleTarget = getLeftAngleTarget(angle);
        int rightAngleTarget = getRightAngleTarget(-angle);

        startDrive(leftAngleTarget, rightAngleTarget, speed);

    }

    /**
     * Lifts elevator a specific distance (in centimeters)
     * @param cm is the distance the elevator will attempt to rise in centimeters
     * @param speed is how fast the motors will spin unitl distance is reached
     */
    public void lift(int cm, double speed){

        int leftTarget = getLeftLiftTarget(cm);
        int rightTarget = getRightLiftTarget(cm);

        startLift(leftTarget, rightTarget, speed);

    }


    /**
     * Sets encoders to run to target position located in @param.
     * //@param fleftTarget front left wheel target
     * //@param frightTarget front right wheel target
     * @param bleftTarget back left wheel target
     * @param brightTarget back right wheel target
     * @param speed the speed at which the target is reached
     */
    public void startDrive(int bleftTarget, int brightTarget, double speed){

        bleftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bleftWheel.setTargetPosition(bleftTarget);
        bleftWheel.setPower(speed);

        brightWheel.setTargetPosition(brightTarget);
        brightWheel.setPower(speed);

        while(isDriving()){
            //Filler for motors to run
        }

        stopMotors();

    }

    /**
     * Sets encoders for lifting motors to run to target position located in @param
     * @param leftTarget the target for the left motor
     * @param rightTarget the target for the right motor
     * @param speed the speed at which the target will be reached
     */
    public void startLift(int leftTarget, int rightTarget, double speed){
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setTargetPosition(leftTarget);
        leftSlide.setPower(speed);
        rightSlide.setTargetPosition(rightTarget);
        rightSlide.setPower(speed);

        while(isLifting()){
            //Filler for motors to run
        }

        stopLift();

    }

    /**
     * Calculates target position for back-right motor by dividing the wanted distance by the distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getBackRightTarget(int cm){

        return ((cm/WHEEL_ROTATION_DISTANCE)*WHEEL_ROTATION_TICKS)+brightWheel.getCurrentPosition();

    }

    /**
     * Calculates target position for back-left motor by dividing the wanted distance by the distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getBackLeftTarget(int cm){

        return ((cm/WHEEL_ROTATION_DISTANCE)*WHEEL_ROTATION_TICKS)+bleftWheel.getCurrentPosition();

    }

    /**
     * Calculates target position for thr left motor when turning. Divides the requested angle by
     * the angle reached by one rotation (forward, backward) multiplied by the number of ticks per rotation.
     * (current postition added for relativity)
     * @param angle is the requested angle for the robot to turn
     * @return the target position in ticks
     */
    public int getLeftAngleTarget(int angle){

        return ((angle/WHEEL_ROTATION_ANGLE)*WHEEL_ROTATION_TICKS)+bleftWheel.getCurrentPosition();

    }

    /**
     * Calculates target position for thr right motor when turning. Divides the requested angle by
     * the angle reached by one rotation (forward, backward) multiplied by the number of ticks per rotation.
     * (current postition added for relativity)
     * @param angle is the requested angle for the robot to turn
     * @return the target position in ticks
     */
    public int getRightAngleTarget(int angle){

        return ((angle/WHEEL_ROTATION_ANGLE)*WHEEL_ROTATION_TICKS)+brightWheel.getCurrentPosition();

    }

    /**
     * Calculates target position for "Left Slide" motor by dividing the wanted distance by the lift distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getLeftLiftTarget(int cm){

        return ((cm/LIFT_DISTANCE)*LIFT_ROTATION_TICKS)+leftSlide.getCurrentPosition();

    }

    /**
     * Calculates target position for "Right Slide" motor by dividing the wanted distance by the lift distance per
     * rotation and multiplying it be the amount of ticks per rotation. (current position added for relativity)
     * @param cm is the distance it needs to travel in centimeters
     * @return the target position in reference to motor ticks
     */
    public int getRightLiftTarget(int cm){

        return ((cm/LIFT_DISTANCE)*LIFT_ROTATION_TICKS)+rightSlide.getCurrentPosition();

    }

    /**
     * Checks if motors are currently trying to lift to a current target
     * @return true if motors are in active target run
     */
    public boolean isLifting(){

        return leftFly.isBusy() && rightFly.isBusy();

    }

    /**
     * Checks if motors are currently trying to reach a target
     * @return true if motors are in active target run
     */
    public boolean isDriving(){

        return bleftWheel.isBusy() && brightWheel.isBusy();

    }

    public void leftColorUnfold(){

        leftColor.setPosition(LEFT_SENSOR_UNFOLD_POS);

    }

    public void leftColorReset(){

        leftColor.setPosition(LEFT_SENSOR_INITIAL_POS);

    }

    public void rightColorUnfold() {

        rightColor.setPosition(RIGHT_SENSOR_UNFOLD_POS);

    }

    public void rightColorReset(){

        rightColor.setPosition(RIGHT_SENSOR_INITIAL_POS);

    }

    public void grab(){

        latch.setPosition(LATCHED);

    }

    public void release(){

        latch.setPosition(UNLATCHED);

    }

    public void flyWheelsOut(){

        hinge.setPosition(HINGE_UNFOLD_POS);

    }

    public void flyWheelsIn(){

        hinge.setPosition(HINGE_INITIAL_POS);

    }
}
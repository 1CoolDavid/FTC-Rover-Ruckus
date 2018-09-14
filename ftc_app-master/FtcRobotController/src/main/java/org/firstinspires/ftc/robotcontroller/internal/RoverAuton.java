package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class RoverAuton extends OpMode{

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    int cameraMonitorViewId;
    VuforiaTrackables roverTrackables;
    VuforiaTrackable roverTemplate;

    public void init(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQ3zNjP/////AAABmcWSY8mOBEOmuXrkWVqwvxiK9O6Zew0t1EC4AuYiSdiQwTkM/GV6PA9Y1edeBwRq8Xo7taqwe86tG/ZBib58h8qbP9UGnxLoJn9bh6rah92U9sso7CZXsSZpC5yrBMLqpZjsogqol+stBri7Im1A5E6e+eTW/cb2TKe6R92ttKGuZVDpqhFeY6bJ/qwMWPntPxBTrj3uVuLZTqEugv73d9FLtE2b/UDTTQlj30tpZS5Smiv+DPPgUwZzsiyK4wS/72LuBuqx2uX6YBtGRoJUV+3NwNpeSJ+hhg9xz6ZNcHbjQC3TQ8SPzl1w6RB7O3fvEb4tpU/V/Hj7Gpi5ktKzKImM17jiSvtKvmbbLwpSLyvL";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        roverTrackables = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        roverTemplate = roverTrackables.get(0);
        roverTemplate.setName("RoverVuMarkTemplate"); //for debug
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
    }

    public void init_loop(){
    }

    public void loop(){

    }
}

package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.odometry.OdometryCorrection;
import ca.mcgill.ecse211.finalproject.sensor.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Controller extends Thread implements UltrasonicController{
    private Navigation navigation;
    private FallingEdgeUSLocalization usl;
    private OdometryCorrection odometryCorrection;
    private LightLocalization lightLocalization;
    private ZiplineTraversal ziplineTraversal;


    public Controller(Odometer odometer,
                      SampleProvider usValue,
                      float[] usData,
                      SampleProvider csValue,
                      float[] csData,
                      EV3LargeRegulatedMotor leftMotor,
                      EV3LargeRegulatedMotor rightMotor,
                      EV3LargeRegulatedMotor ziplineMotor,
                      double WHEEL_RADIUS,
                      double TRACK
                      ){

        this.navigation = new Navigation(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK);


        this.usl = new FallingEdgeUSLocalization(odometer, usValue, usData, FallingEdgeUSLocalization.LocalizationType.FALLING_EDGE, leftMotor, rightMotor, navigation);

        this.lightLocalization = new LightLocalization(navigation, odometer, leftMotor, rightMotor, csValue, csData);

        //TODO:uncomment below
        //this.odometryCorrection = new OdometryCorrection(odometer, csValue, csData);
        
        // TODO: uncomment below
        //this.zipLineTraversal = new ZipLineTraversal(navigation, odometer, leftMotor, rightMotor, ziplineMotor, usValue, usData);

    }

    public void run() {
        usl.doLocalization();

        lightLocalization.doLocalization();

        //TODO: uncomment below
        //odometryCorrection.start();
        //zipLineTraversal.doTraversal();
    }

    @Override
    public void processUSData(float usData) {
        // TODO Auto-generated method stub

    }

    @Override
    public float readUSData() {
        // TODO Auto-generated method stub
        return 0;
    }

}

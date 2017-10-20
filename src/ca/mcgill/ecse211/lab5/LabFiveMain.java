package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.lab5.LightLocalization;
import ca.mcgill.ecse211.lab5.Odometer;
import ca.mcgill.ecse211.lab5.OdometeryDisplay;
import ca.mcgill.ecse211.lab5.UltrasonicLocalization;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LabFiveMain extends Thread {

    public static final EV3LargeRegulatedMotor leftMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

    public static final EV3LargeRegulatedMotor rightMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

    public static final EV3LargeRegulatedMotor ziplineMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

    /*private static final EV3ColorSensor colorSensor =
            new EV3ColorSensor(LocalEV3.get().getPort("S1"));*/

    //public static final double WHEEL_RADIUS = 2.2;
    //public static final double TRACK = 10.7;
    public static final int motorLow = 100; // Speed of slower rotating wheel (deg/sec)
    public static final int motorHigh = 200; // Speed of the faster rotating wheel (deg/sec)

    public static void main(String[] args) {
        int buttonChoice;

        // set up main display and controller objects
        final TextLCD t = LocalEV3.get().getTextLCD();
        Odometer odometer = new Odometer(leftMotor, rightMotor);

        do {
            // clear the display
            t.clear();

            // ask the user whether the motors should drive in a square or float
            t.drawString("< Drive|  Right >", 0, 0);
            t.drawString("       |         ", 0, 1);
            t.drawString("       |  Falling", 0, 2);
            t.drawString("       |  edge   ", 0, 3);
            t.drawString("       | 		 ", 0, 4);

            buttonChoice = Button.waitForAnyPress();
        } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

        if (buttonChoice == Button.ID_LEFT) {

            leftMotor.setSpeed(motorHigh);
            rightMotor.setSpeed(motorHigh);
            ziplineMotor.setSpeed(motorHigh);

            leftMotor.forward();
            rightMotor.forward();
            ziplineMotor.backward();
			 /* OdometeryDisplay odometryDisplay = new OdometeryDisplay(odometer, t, usLocalizer);
		      odometer.start();
		      odometryDisplay.start();

		      buttonChoice = Button.waitForAnyPress();
		      if(buttonChoice == Button.ID_LEFT){
		    	  LightLocalization.doLightLocalization();*/
            //}
        } //else {

			  /*OdometeryDisplay odometryDisplay = new OdometeryDisplay(odometer, t, usLocalizer);
		      odometer.start();
		      odometryDisplay.start();
		      buttonChoice = Button.waitForAnyPress();
		      if(buttonChoice == Button.ID_LEFT){
		    	  LightLocalization.doLightLocalization();*/
        //}
        //}

        while (Button.waitForAnyPress() != Button.ID_ESCAPE) ;
        System.exit(0);
    }

}

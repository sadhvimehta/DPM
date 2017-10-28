package ca.mcgill.ecse211.finalproject.main;

import ca.mcgill.ecse211.finalproject.drive.Navigation;
import ca.mcgill.ecse211.finalproject.localization.FallingEdgeUSLocalization;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.odometry.OdometryDisplay;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Main {

    public static final EV3LargeRegulatedMotor leftMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

    public static final EV3LargeRegulatedMotor rightMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

    public static final EV3LargeRegulatedMotor ziplineMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

    public static final Port usPort = LocalEV3.get().getPort("S1");

    public static final Port csPort = LocalEV3.get().getPort("S2");


    public static final double WHEEL_RADIUS = 2.05;
    public static final double TRACK = 15.7;
    public static final int motorLow = 100; // speed of slower rotating wheel (deg/sec)
    public static final int motorHigh = 200; // speed of the faster rotating wheel (deg/sec)
    public static int xPreMount = 0; // this is x0 coordinate
    public static int yPreMount = 0; // this is y0 coordinate
    public static int cornerNum = 0; // this is the corner number
    public static int xMount = 0; // this is xC coordinate
    public static int yMount = 0; // this is yC coordinate

    public static void main(String[] args) {
        int buttonChoice;
        final TextLCD t = LocalEV3.get().getTextLCD();

        Odometer odometer = new Odometer(leftMotor, rightMotor, ziplineMotor);
        OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);

        EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(usPort);
        SampleProvider usValue = usSensor.getMode("Distance");
        float[] usData = new float[usValue.sampleSize()];

        EV3ColorSensor csSensor = new EV3ColorSensor(csPort);
        SampleProvider csValue = csSensor.getRedMode();
        float[] csData = new float[csValue.sampleSize()];
        
	    MainMenuDisplay mainMenu = new MainMenuDisplay(t);

        do {
           
        	mainMenu.displayCoord(xPreMount, yPreMount);

            buttonChoice = Button.waitForAnyPress();
        } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ENTER);

        while (buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT) {
            if (buttonChoice == Button.ID_LEFT) {
                if (xPreMount == 8)
                    xPreMount = 0;
                else
                    xPreMount++;

               mainMenu.displayCoord(xPreMount, yPreMount);
            }

            if (buttonChoice == Button.ID_RIGHT) {
                if (yPreMount == 8)
                    yPreMount = 0;
                else
                    yPreMount++;

                mainMenu.displayCoord(xPreMount, yPreMount);
            }
            buttonChoice = Button.waitForAnyPress();
        }

        if (buttonChoice == Button.ID_ENTER) {

            do {
                mainMenu.displayCorner(cornerNum);
                buttonChoice = Button.waitForAnyPress();
            } while (buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ENTER);
        }

        while (buttonChoice == Button.ID_RIGHT) {
            if (cornerNum == 3)
                cornerNum = 0;
            else
                cornerNum++;

            mainMenu.displayCorner(cornerNum);
            buttonChoice = Button.waitForAnyPress();
        }

        if (buttonChoice == Button.ID_ENTER) {
            do {
            	
            	mainMenu.displayCoord(xMount, yMount);
                buttonChoice = Button.waitForAnyPress();
            }
            while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ENTER);
        }

        while (buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT) {
            if (buttonChoice == Button.ID_LEFT) {
                if (xMount == 8)
                    xMount = 0;
                else
                    xMount++;

                mainMenu.displayCoord(xMount, yMount);
            }

            if (buttonChoice == Button.ID_RIGHT) {
                if (yMount == 8)
                    yMount = 0;
                else
                    yMount++;

               mainMenu.displayCoord(xMount, yMount);
            }
            buttonChoice = Button.waitForAnyPress();
        }

        if (buttonChoice == Button.ID_ENTER) {

            Sound.setVolume(30);
            Sound.buzz();
    	   /*odometer.start();
           odometryDisplay.start();
           leftMotor.setSpeed(motorHigh);
           rightMotor.setSpeed(motorHigh);
           ziplineMotor.setSpeed(motorHigh);

           leftMotor.forward();
           rightMotor.forward();
           ziplineMotor.backward();*/

            odometer.start();
            odometryDisplay.start();
            
            Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK);

            /* instantiate FallingEdgeUSLocalization class */
            FallingEdgeUSLocalization usl = new FallingEdgeUSLocalization(odometer, usValue, usData, FallingEdgeUSLocalization.LocalizationType.FALLING_EDGE, leftMotor, rightMotor, navigation);
            //TODO:uncomment below
            //OdometryCorrection odometryCorrection = new OdometryCorrection(odometer, csValue, csData);
            LightLocalization lightLocalization = new LightLocalization(navigation, odometer, leftMotor, rightMotor, csValue, csData);
            // TODO: uncomment below
            //ZipLineTraversal zipLineTraversal = new ZipLineTraversal(navigation, odometer, leftMotor, rightMotor, ziplineMotor, usValue, usData);

            usl.doLocalization();
            
            lightLocalization.doLocalization();
            
            //TODO: uncomment below
            //odometryCorrection.start();
            //zipLineTraversal.doTraversal();
        }



        while (true) {
            if (Button.waitForAnyPress() == Button.ID_ENTER) {
                System.exit(0);
            }
        }

    }

}

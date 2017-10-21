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

    public static final double WHEEL_RADIUS = 2.2;
    public static final double TRACK = 16.0;
    public static final int motorLow = 100; // speed of slower rotating wheel (deg/sec)
    public static final int motorHigh = 200; // speed of the faster rotating wheel (deg/sec)
    public static int xPreMount = 0; // this is x0 coordinate
    public static int yPreMount = 0; // this is y0 coordinate
    public static int cornerNum = 0; // this is the corner number
    public static int xMount = 0; // this is xC coordinate
    public static int yMount = 0; // this is yC coordinate

    public static void main(String[] args) {
        int buttonChoice;
        String preMountCoordinates = "  " + xPreMount + "       |  " + yPreMount + "     ";

        // set up main display and controller objects
        final TextLCD t = LocalEV3.get().getTextLCD();
        Odometer odometer = new Odometer(leftMotor, rightMotor, ziplineMotor);
        OdometeryDisplay odometryDisplay = new OdometeryDisplay(odometer, t);

        do {
            // clear the display
            t.clear();

            // display default coordinates of (0,0)
            t.drawString("< X-0:    |  Y-0: >", 0, 0);
            t.drawString("          |        ", 0, 1);
            t.drawString(preMountCoordinates  , 0, 2);
            t.drawString("          |        ", 0, 3);

            buttonChoice = Button.waitForAnyPress();
        } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ENTER);
        
        while(buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT){
        	if(buttonChoice == Button.ID_LEFT){
        		if(xPreMount == 8)
        			xPreMount = 0;
        		else
        			xPreMount++;
        		
        		// clear the display
                t.clear();

                // update coorindates
                t.drawString("< X-0:    |  Y-0: >", 0, 0);
                t.drawString("          |        ", 0, 1);
                t.drawString("  " + xPreMount + "       |  " + yPreMount + "     ", 0, 2);
                t.drawString("          |        ", 0, 3);
        	}
        	
        	if(buttonChoice == Button.ID_RIGHT){
        		if(yPreMount == 8)
        			yPreMount = 0;
        		else
        			yPreMount++;
        		
        		// clear the display
                t.clear();

                // update coorindates
                t.drawString("< X-0:    |  Y-0: >", 0, 0);
                t.drawString("          |        ", 0, 1);
                t.drawString("  " + xPreMount + "       |  " + yPreMount + "     ", 0, 2);
                t.drawString("          |        ", 0, 3);
        	}
            buttonChoice = Button.waitForAnyPress();
        }

        if (buttonChoice == Button.ID_ENTER) {
        	
        	do{
        		// clear the display
                t.clear();

                // choose corner
                t.drawString("Choose corner", 0, 0);
                t.drawString(cornerNum + " >", 0, 1);
                buttonChoice = Button.waitForAnyPress();
        	} while(buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ENTER);
        }
            
        if(buttonChoice == Button.ID_RIGHT){
        	while(buttonChoice == Button.ID_RIGHT){
        		if(cornerNum == 3)
        			cornerNum = 0;
        		else
        			cornerNum++;
        		
           		// clear the display
                t.clear();

                // choose corner
                t.drawString("Choose corner", 0, 0);
                t.drawString(cornerNum + " >", 0, 1);
        	}
        	buttonChoice = Button.waitForAnyPress();
        }
        	
       if (buttonChoice == Button.ID_ENTER) {
    	   do{
                    // clear the display
                    t.clear();

                    // display default coordinates of (0,0)
                    t.drawString("< X-C:    |  Y-C: >", 0, 0);
                    t.drawString("          |        ", 0, 1);
                    t.drawString("  " + xMount + "       |  " + yMount + "     ", 0, 2);
                    t.drawString("          |        ", 0, 3);

                    buttonChoice = Button.waitForAnyPress();
            } while(buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ENTER);
       }
       
       while(buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT){
    	   if(buttonChoice == Button.ID_LEFT){
    		   if(xMount == 8)
    			   xMount = 0;
               else
            	   xMount++;
                		
                // clear the display
                t.clear();

                // update coorindates
                t.drawString("< X-C:    |  Y-C: >", 0, 0);
                t.drawString("          |        ", 0, 1);
                t.drawString("  " + xMount + "       |  " + yMount + "     ", 0, 2);
                t.drawString("          |        ", 0, 3);
           }
                	
          if(buttonChoice == Button.ID_RIGHT){
                if(yMount == 8)
                	yMount = 0;
                else
                	yMount++;
                		
                // clear the display
                t.clear();

                // update coorindates
                t.drawString("< X-C:    |  Y-C: >", 0, 0);
                t.drawString("          |        ", 0, 1);
                t.drawString("  " + xMount + "       |  " + yMount + "     ", 0, 2);
                t.drawString("          |        ", 0, 3);
         }
         buttonChoice = Button.waitForAnyPress();
       }
       
       if(buttonChoice == Button.ID_ENTER){
    	   odometer.start();
           odometryDisplay.start();
           leftMotor.setSpeed(motorHigh);
           rightMotor.setSpeed(motorHigh);
           ziplineMotor.setSpeed(motorHigh);

           leftMotor.forward();
           rightMotor.forward();
           ziplineMotor.backward();
       }
            
			 /* OdometeryDisplay odometryDisplay = new OdometeryDisplay(odometer, t, usLocalizer);
		      odometer.start();
		      odometryDisplay.start();

		      buttonChoice = Button.waitForAnyPress();
		      if(buttonChoice == Button.ID_LEFT){
		    	  LightLocalization.doLightLocalization();*/
            //}
         //else {

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

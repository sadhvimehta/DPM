package ca.mcgill.ecse211.finalproject.main;

import ca.mcgill.ecse211.finalproject.controller.Controller;
import ca.mcgill.ecse211.finalproject.controller.FallingEdgeUSLocalization;
import ca.mcgill.ecse211.finalproject.controller.LightLocalization;
import ca.mcgill.ecse211.finalproject.controller.Navigation;
import ca.mcgill.ecse211.finalproject.display.OdometryDisplay;
import ca.mcgill.ecse211.finalproject.display.WiFiConnect;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
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


/**
 * Main class responsible for declaring and initializing all other classes.
 *
 */
public class CaptureFlagMain {
	
	/**
	 * Left motor of robot.
	 */
    public static final EV3LargeRegulatedMotor leftMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
    
    /**
     * Right motor of robot.
     */
    public static final EV3LargeRegulatedMotor rightMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
    
    /**
     * Motor used to traverse zipline.
     */
    public static final EV3LargeRegulatedMotor ziplineMotor =
            new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    
    /**
     * Ultrasonic sensor port.
     */
    public static final Port usPort = LocalEV3.get().getPort("S1");
    
    /**
     * Color sensors port.
     */
    public static final Port csPort = LocalEV3.get().getPort("S2");
    
    /**
     * IP address of server
     */
    private static final String serverIP = "192.168.2.18";
    /**
     * Team number
     */
    private static final int teamNumber = 15;
    /**
     * Radius of right/left wheel
     */
    public static final double WHEEL_RADIUS = 2.1;
	/**
	 * Radius of zipline wheel
	 */
	public static final double ZIPLINE_WHEEL_RADIUS = 1.25;
    /**
     * Distance between two wheels of robot
     */
    public static double TRACK = 16.1; //15.9
    
    public static final double balanceConstant = 1.005; // constant to balance out wheel imbalance
    /**
     * Slow motor speed in deg/sec
     */
    public static final int motorLow = 100; 
    /**
     * Fast motor speed in deg/sec
     */
    public static final int motorHigh = 200;
    /**
     * Starting corner
     */
    public static int startingCorner = 0; // this is the corner number
    /**
     * Team color
     */
    public static String teamColor; // this is the team color that specifies whether to go by zipline or river first.
    /**
     * Color of block to capture
     */
    public static int flagColor; // this is flag color

    /**
     * Lower left corner of starting zone (x)
     */
	public static int LL_x;

	public static int LL_y;

	public static int UR_x;

	public static int UR_y;

	public static int LL_search_x;

	public static int LL_search_y;

	public static int UR_search_x;

	public static int UR_search_y;

	public static int ziplineEndPoint_red_x;

	public static int ziplineEndPoint_red_y;

	public static int ziplineOther_red_x;

	public static int LL_horizontalShallow_y;

	public static int LL_horizontalShallow_x;

	public static int LL_verticalShallow_y;

	public static int ziplineOther_red_y;

	public static int UR_horizontalShallow_x;

	public static int UR_horizontalShallow_y;

	public static int LL_verticalShallow_x;

	public static int UR_verticalShallow_x;

	public static int UR_verticalShallow_y;

	public static int ziplineEndPoint_green_x;

	public static int ziplineEndPoint_green_y;

	public static int ziplineOther_green_x;

	public static int ziplineOther_green_y;
	
	public static boolean doCorrection = false;
    
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

        Controller controller = new Controller(odometer, usValue, usData, csValue, csData, leftMotor, rightMotor, ziplineMotor);

        WiFiConnect wifiConnection = new WiFiConnect(serverIP, teamNumber, false); // get input from server
        do {
           
        	wifiConnection.startWifiInitialization();

            buttonChoice = Button.waitForAnyPress();
        } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_ENTER);

        if (buttonChoice == Button.ID_ENTER) {

            odometer.start();
            odometryDisplay.start();
            controller.startCourseTraversal();

        }

        while (true) {
            if (Button.waitForAnyPress() == Button.ID_ENTER) {
                System.exit(0);
            }
        }

    }

}

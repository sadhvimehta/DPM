package ca.mcgill.ecse211.finalproject.main;

import ca.mcgill.ecse211.finalproject.controller.Controller;
import ca.mcgill.ecse211.finalproject.display.OdometryDisplay;
import ca.mcgill.ecse211.finalproject.display.WiFiConnect;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * Main class responsible for declaring and initializing all other classes. As we wanted a certain
 * level of abstraction, we made a controller class that would contain all the logic, while main
 * would simply call them and start the necessary threads.
 */
public class CaptureFlagMain {

  /** Left motor of robot. */
  public static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /** Right motor of robot. */
  public static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /** Motor used to traverse zipline. */
  public static final EV3LargeRegulatedMotor ziplineMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

  /** Ultrasonic sensor port. */
  public static final Port usPort = LocalEV3.get().getPort("S1");

  /** Color sensor port for light localization. */
  public static final Port csPort = LocalEV3.get().getPort("S2");
  /** Color sensor port for block detection. */
  public static final Port blockcsPort = LocalEV3.get().getPort("S4");

  /** IP address of server */
  private static final String serverIP = "192.168.2.3";
  /** Team number */
  private static final int teamNumber = 15;
  /** Radius of right/left wheel */
  public static final double WHEEL_RADIUS = 2.1;
  /** Radius of zipline wheel */
  public static final double ZIPLINE_WHEEL_RADIUS = 1.25;
  /** Distance between two wheels of robot */
  public static double TRACK = 15.90; // DO NOT CHANGE TRACK WHATSOEVER

  /** Number of tiles within map. */
  public static int MAP_SIZE = 12;

  /** Tested constant responsible for making right wheel go faster to balance drift in wheels. */
  public static final double balanceConstant = 1.0072; // constant to balance out wheel imbalance
  /** Starting corner */
  public static int startingCorner = -1; // this is the corner number
  /** Team color */
  public static String
      teamColor; // this is the team color that specifies whether to go by zipline or river first.
  /** Color of block to capture */
  public static int flagColor; // this is flag color
  /** Lower left corner of starting zone (x) */
  public static int LL_x;
  /** Lower left corner of starting zone (y) */
  public static int LL_y;
  /** Upper right corner of starting zone (x) */
  public static int UR_x;
  /** Upper right corner of starting zone (x) */
  public static int UR_y;
  /** Lower left corner of search zone (x) */
  public static int LL_search_x;
  /** Lower left corner of search zone (y) */
  public static int LL_search_y;
  /** Upper right corner of search zone (x) */
  public static int UR_search_x;
  /** Upper right corner of search zone (y) */
  public static int UR_search_y;
  /** End point of zipline (x) */
  public static int ziplineEndPoint_red_x;
  /** End point of zipline (y) */
  public static int ziplineEndPoint_red_y;
  /** Point to travel to after end point of zipline (x) */
  public static int ziplineOther_red_x;
  /** Lower left corner of horizontal segment of shallow water (y) */
  public static int LL_horizontalShallow_y;
  /** Lower left corner of horizontal segment of shallow water (x) */
  public static int LL_horizontalShallow_x;
  /** Lower left corner of vertical segment of shallow water (y) */
  public static int LL_verticalShallow_y;
  /** Point to travel to after end point of zipline (y) */
  public static int ziplineOther_red_y;
  /** Upper right corner of horizontal segment of shallow water (x) */
  public static int UR_horizontalShallow_x;
  /** Upper right corner of horizontal segment of shallow water (y) */
  public static int UR_horizontalShallow_y;
  /** Lower left corner of vertical segment of shallow water (x) */
  public static int LL_verticalShallow_x;
  /** Upper right corner of vertical segment of shallow water (x) */
  public static int UR_verticalShallow_x;
  /** Upper right corner of vertical segment of shallow water (y) */
  public static int UR_verticalShallow_y;
  /** Start point of zipline (x) */
  public static int ziplineEndPoint_green_x;
  /** Start point of zipline (y) */
  public static int ziplineEndPoint_green_y;
  /** Premount point of zipline (x) */
  public static int ziplineOther_green_x;
  /** Premount point of zipline (y) */
  public static int ziplineOther_green_y;
  /** Boolean indicating whether to perform odometry correction or not */
  public static boolean doCorrection = false;

  /** Search lower left corner in our zone (x) */
  public static int LL_mysearch_x;
  /** Search lower left corner in our zone (y) */
  public static int LL_mysearch_y;
  /** Search upper right corner in our zone (x) */
  public static int UR_mysearch_x;
  /** Search upper right corner in our zone (y) */
  public static int UR_mysearch_y;
  /** Lower left corner of red zone (x) */
  public static int LL_redZone_x;
  /** Lower left corner of red zone (y) */
  public static int LL_redZone_y;
  /** Upper right corner of red zone (x) */
  public static int UR_redZone_x;
  /** Upper right corner of red zone (y) */
  public static int UR_redZone_y;
  /** Lower left corner of green zone (x) */
  public static int LL_greenZone_x;
  /** Lower left corner of green zone (y) */
  public static int LL_greenZone_y;
  /** Upper right corner of green zone (x) */
  public static int UR_greenZone_x;
  /** Upper right corner of green zone (y) */
  public static int UR_greenZone_y;

  public static void main(String[] args) {
    final TextLCD t = LocalEV3.get().getTextLCD();

    Odometer odometer = new Odometer(leftMotor, rightMotor);
    OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t);

    EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(usPort);
    SampleProvider usValue = usSensor.getMode("Distance");
    float[] usData = new float[usValue.sampleSize()];

    EV3ColorSensor csSensor = new EV3ColorSensor(csPort);
    SampleProvider csValue = csSensor.getRedMode();
    float[] csData = new float[csValue.sampleSize()];

    EV3ColorSensor blockcsSensor = new EV3ColorSensor(blockcsPort);
    SampleProvider blockcsValue = blockcsSensor.getRGBMode();
    float[] blockcsData = new float[blockcsValue.sampleSize()];

    Controller controller =
        new Controller(
            odometer,
            usValue,
            usData,
            csValue,
            csData,
            blockcsValue,
            blockcsData,
            leftMotor,
            rightMotor,
            ziplineMotor);

    WiFiConnect wifiConnection =
        new WiFiConnect(serverIP, teamNumber, false); // get input from server

    do {

      wifiConnection.startWifiInitialization();

    } while (startingCorner
        == -1); // will intialize starting corner to -1 so that no button press is needed for the
                // robot to operate after the data has been transmited

    odometer.start();
    odometryDisplay.start();
    controller.startCourseTraversal();

    while (true) {
      if (Button.waitForAnyPress() == Button.ID_ENTER) {
        System.exit(0);
      }
    }
  }
}

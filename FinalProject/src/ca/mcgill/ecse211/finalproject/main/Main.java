package ca.mcgill.ecse211.finalproject.main;

import ca.mcgill.ecse211.finalproject.controller.Controller;
import ca.mcgill.ecse211.finalproject.controller.FallingEdgeUSLocalization;
import ca.mcgill.ecse211.finalproject.controller.LightLocalization;
import ca.mcgill.ecse211.finalproject.controller.Navigation;
import ca.mcgill.ecse211.finalproject.display.MainMenuDisplay;
import ca.mcgill.ecse211.finalproject.display.OdometryDisplay;
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
 * <h1>Design Principle Methods Team 15 Final Project</h1>
 *
 * //TODO: write a bit more on what the project is. maybe like an intro or smth
 *
 * <p style="text-indent: 30px">
 * The goal of this project was to build robot using the lego Mindstorms NXT parts and make it accomplish a set of
 * tasks, requirements. The flow of the competition is started off as our robot must know the layout of the arena. Using
 * the <i>Wifi</i> class, which is provided, the robot will be sent the map. To allow the user to input various values,
 * the <i>MainMenuDisplay</i> class output instructions to the user to fill in the missing values. Next, the robot must
 * localize itself in the arena. With two sets of variables, orientation and position, the orientation is found with the
 * help of the <i>FallingEdgeUSLocalization</i> class. Next, the position are filled in using the
 * <i>LightLocalization</i> class which fills in the X and Y of the robot in the arena. To allow the robot to keep track
 * of its position all along the competition our first thread is implemented, an odometer with the <i>Odometer</i>
 * class. Knowing the value of threads, a thread is dedicated to the odometer as when do not want the odometer to lose
 * track of the position and orientation due to an other method running. To then allow the user to get feedback on the
 * status of the robot, an other thread was used in <i>OdometryDisplay</i> class as we want the screen to be constantly
 * updated. Next, our third thread is dedicated to the class <i>OdometryCorrection</i>. This class takes care of
 * correcting the odometer as the robot as the competition unfolds by detection of grid lines drawn on the arena. This
 * class must absolutely be a thread as we would not want to miss a line capture.
 *
 * <p style="text-indent: 30px">
 *  Then, the robot finally starts moving with our final thread, <i>Controller</i> class. This class implements
 * the logic of the movement of the robot throughout the map by controlling the remaining classes, as we wanted the main
 * thread to simply instantiate variables, and start the other threads. The <i>Navigation</i> class takes care of moving
 * the robot across various environments, whether it be a gridded floor, a zipline, or an imaginary river in the middle
 * of the map. When the terrain happens to be a zipline, or a river, the classes <i>ZiplineTraversal</i>, and
 * <i>RiverTraversal</i> are respectively called aid the navigation. These classes therefore help the robot cross from
 * its side of the map to the other side. The class <i>BlockDetection</i>, then helps with finding blocks, and
 * identifying which of them is the opponents flag. Once it has been captured, the robot must return to its side with
 * the help of the <i>Navigation</i> class once again. All along, the <i>Controller</i> makes sure that the robot is not
 * running into anything with the help of <i>ObstacleAvoidance</i> class if the robot gets too close to any object.
 *
 * <p style="text-indent: 30px">
 *  The last part of the structure of this project are the interfaces <i>LightController</i> and
 * <i>UltrasonicController</i>. These interfaces are the basic methods that are needed to get back the data form the
 * sensors. Various classes therefore, implement them, and possibly override their methods to tailor the interpretation
 * of the data according to need.
 *
 */
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

        Controller controller = new Controller(odometer, usValue, usData, csValue, csData, leftMotor, rightMotor, ziplineMotor, WHEEL_RADIUS, TRACK);

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

            odometer.start();
            odometryDisplay.start();

            //TODO: make sure that controller thread does this stuff before removing
            /*Navigation navigation = new Navigation(odometer, leftMotor, rightMotor, WHEEL_RADIUS, TRACK);
            FallingEdgeUSLocalization usl = new FallingEdgeUSLocalization(odometer, usValue, usData, FallingEdgeUSLocalization.LocalizationType.FALLING_EDGE, leftMotor, rightMotor, navigation);
            //OdometryCorrection odometryCorrection = new OdometryCorrection(odometer, csValue, csData);
            LightLocalization lightLocalization = new LightLocalization(navigation, odometer, leftMotor, rightMotor, csValue, csData);
            //ZipLineTraversal zipLineTraversal = new ZipLineTraversal(navigation, odometer, leftMotor, rightMotor, ziplineMotor, usValue, usData);*/


            controller.start();
        }



        while (true) {
            if (Button.waitForAnyPress() == Button.ID_ENTER) {
                System.exit(0);
            }
        }

    }

}

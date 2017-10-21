package ca.mcgill.ecse211.lab5;

// imports needed for this lab

import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class UltrasonicLocalization {
    private static final int ROTATE_SPEED = 100;
    private static final long LOOP_TIME = 5;
    private static final double TAU = Math.PI * 2;
    ArrayList<Double[]> lastNValue = new ArrayList<>();
    // other classes needed for this lab
    private Odometer odometer;
    private SampleProvider usSensor;
    private float[] usData;
    private LocalizationType localizationType;
    private Navigation navigation;

    private EV3LargeRegulatedMotor leftMotor, rightMotor;

    // variables needed for this lab
    private double alpha, beta;
    private long startTime, correctionStart, correctionEnd;
    private double biggest, smallest, timeBiggest, timeSmallest;

    TextLCD t;

    public UltrasonicLocalization(
            Odometer odometer,
            EV3UltrasonicSensor usSensor,
            float[] usData,
            LocalizationType localizationType,
            Navigation navigation,
            EV3LargeRegulatedMotor leftMotor,
            EV3LargeRegulatedMotor rightMotor,
            TextLCD t) {
        this.odometer = odometer;
        this.usSensor = usSensor;
        this.usData = usData;
        this.localizationType = localizationType;
        this.navigation = navigation;

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        resetMotor();
        this.t = t;

        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter("log.txt"));
            writer.close();
        } catch (Exception e) {

        }
    }

    /**
     * the main method which localizes the robot through ultrasonic
     */
    public void localize() {
        // checks the time at which the localization starts
        startTime = System.currentTimeMillis();

        // different methods depending on which edge was selected
        if (localizationType == LocalizationType.FALLING_EDGE) {
            findFallingEdge();
        } else {
            findRisingEdge();
        }
    }

    public void optLocalize() {

        LocalizationType caughtEdge = null;

        startTime = System.currentTimeMillis();

        navigation.turnCW(360);

        // this takes care of checking if a falling edge has been crossed to assign it to alpha
        while (odometer.getTheta() < TAU) {
            if (fallingEdgeCaught()) {
                alpha = odometer.getTheta();
                caughtEdge = LocalizationType.FALLING_EDGE;
                break;
            }
            else if (risingEdgeCaught()) {
                alpha = odometer.getTheta();
                caughtEdge = LocalizationType.RISING_EDGE;
                break;
            }
        }

        // checks for the other falling edge to assign to beta
        while (odometer.getTheta() < TAU) {
            if (caughtEdge == LocalizationType.RISING_EDGE && fallingEdgeCaught()) {
                resetMotor();
                beta = odometer.getTheta();
                break;
            }
            else if (caughtEdge == LocalizationType.FALLING_EDGE && risingEdgeCaught()) {
                resetMotor();
                beta = odometer.getTheta();
                break;
            }
        }

        // method calculates our delta theta, updates the odometer, and spins to face the 0 degree
        updateOdometerTheta(alpha, beta);
    }

    /**
     * this method gets the data from the ultrasonic and only returns values under or equal to 100
     */
    private float getFilteredData() {
        correctionStart = System.currentTimeMillis();

        usSensor.fetchSample(usData, 0);
        float distance = usData[0] * 100;
        float returnedDistance;

        if (distance > 100) {
            returnedDistance = 100;
        } else {
            returnedDistance = distance;
        }

        // the correctionstart and correctionend are to make sure that a value is taken once every
        // LOOP_TIME
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < LOOP_TIME) {
            try {
                Thread.sleep(LOOP_TIME - (correctionEnd - correctionStart));
            } catch (InterruptedException e) {
            }
        }

        return returnedDistance;
    }

    /**
     * this method finds the alpha and beta for the falling edges, and updates the odometer
     */
    public void findFallingEdge() {
        navigation.turnCW(360);

        // this takes care of checking if a falling edge has been crossed to assign it to alpha
        while (odometer.getTheta() < TAU) {
            if (fallingEdgeCaught()) {
                resetMotor();
                alpha = odometer.getTheta();
                break;
            }
        }

        navigation.turnCCW(360);

        try {
            TimeUnit.SECONDS.sleep(2);
        } catch (Exception e) {
        }

        // checks for the other falling edge to assign to beta
        while (odometer.getTheta() > 0) {
            if (fallingEdgeCaught()) {
                resetMotor();
                beta = odometer.getTheta();
                break;
            }
        }

        // method calculates our delta theta, updates the odometer, and spins to face the 0 degree
        updateOdometerTheta(alpha, beta);
    }

    /**
     * this method indicates whether a falling edge has actually been crossed
     */
    public boolean fallingEdgeCaught() {
        // adds the latest data to an array which then sets the biggest value and the smallest value
        lastNValueAdd(getFilteredData());
        lastNValueFind();

        // if the biggest value is above a threshold value and the smallest as well, then it is a
        // falling
        // edge if the biggest's time was before the smallest's time
        if (biggest > 80 && smallest < 20 && timeBiggest < timeSmallest) {
            this.lastNValue.clear();
            Sound.setVolume(30);
            Sound.beep();
            return true;
        } else {
            return false;
        }
    }

    /**
     * method finds the alpha and beta for the rising edges edges, and updates the odometer
     */
    public void findRisingEdge() {
        navigation.turnCW(360);

        // this takes care of checking if a rising edge has been crossed to assign it to alpha
        while (odometer.getTheta() < TAU) {
            if (risingEdgeCaught()) {
                resetMotor();
                alpha = odometer.getTheta();
                break;
            }
        }

        navigation.turnCCW(360);

        try {
            TimeUnit.SECONDS.sleep(2);
        } catch (Exception e) {
        }

        while (odometer.getTheta() > 0) {
            if (risingEdgeCaught()) {
                resetMotor();
                beta = odometer.getTheta();
                break;
            }
        }

        // method calculates our delta theta, updates the odometer, and spins to face the 0 degree
        updateOdometerTheta(alpha, beta);
    }

    /**
     * this method indicates whether a rising edge has actually been crossed
     */
    public boolean risingEdgeCaught() {
        // adds the latest data to an array which then sets the biggest value and the smallest value
        lastNValueAdd(getFilteredData());
        lastNValueFind();

        // if the biggest value is above a threshold value and the smallest as well, then it is a rising
        // edge if the biggest's time was after the smallest's time
        if (biggest > 80 && smallest < 20 && timeBiggest > timeSmallest) {
            this.lastNValue.clear();
            Sound.setVolume(30);
            Sound.beep();
            return true;
        } else {
            return false;
        }
    }

    /**
     * method sets the biggest value and the smallest value, and their time in the last n value
     */
    public void lastNValueFind() {
        biggest = -200;
        smallest = 200;
        timeBiggest = -1000;
        timeSmallest = 1000;

        for (int i = 0; i < this.lastNValue.size(); i++) {
            if (this.lastNValue.get(i)[0] > biggest) {
                biggest = this.lastNValue.get(i)[0];
                timeBiggest = lastNValue.get(i)[1];
            }
            if (this.lastNValue.get(i)[0] < smallest) {
                smallest = this.lastNValue.get(i)[0];
                timeSmallest = lastNValue.get(i)[1];
            }
        }
    }

    /**
     * method adds in values to the array with logic
     */
    public void lastNValueAdd(double value) {
        Double[] entry = {value, (double) System.currentTimeMillis() - startTime};
        // the array doesnt take chucks of the values and risk to miss a big difference.
        // Instead we slide along
        if (this.lastNValue.size() > 40) {
            // the oldest value is removed to make space
            this.lastNValue.remove(0);
            this.lastNValue.add(entry);
        } else {
            // if the array happens to be less that, simply add them
            this.lastNValue.add(entry);
        }
    }

    /**
     * method resets the motors
     */
    private void resetMotor() {
        leftMotor.stop(true);
        rightMotor.stop(true);
        // leftMotor.rotate(0);
        // rightMotor.rotate(0);
        for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[]{leftMotor, rightMotor}) {
            motor.setAcceleration(3000);
            motor.setSpeed(ROTATE_SPEED);
        }
    }

    /**
     * method which uses alpha and beta to calculate delta theta, update odometer's theta, and turn to
     * face the 0 degree
     */
    private void updateOdometerTheta(double alpha, double beta) {
        double dTheta;

        // dTheta depends on which of alpha and beta is larger
        if (alpha < beta && localizationType == LocalizationType.FALLING_EDGE
                || alpha > beta && localizationType == LocalizationType.RISING_EDGE) {
            dTheta = (5 * TAU / 8) - ((alpha + beta) / 2); // 225 degrees minus average of angles
            Sound.setVolume(50);
            Sound.buzz();
        } else {

            //
            if (localizationType == LocalizationType.RISING_EDGE) {
                dTheta = (TAU / 8) - ((alpha + beta) / 2 - TAU / 72);
            } else {
                //

                dTheta = (TAU / 8) - ((alpha + beta) / 2);
            } //
        }

        t.drawString("alpha: " + alpha, 0, 4);
        t.drawString("beta: " + beta, 0, 5);
        t.drawString("dtheta: " + dTheta, 0, 6);

        odometer.setTheta(odometer.getTheta() + dTheta);

        navigation.turnTo(0);
    }

    public void getData(){
        navigation.turnCW(360);
        while (odometer.getTheta() < 360) {
            try {
                BufferedWriter writer = new BufferedWriter(new FileWriter("log.txt", true));
                writer.write(String.valueOf(odometer.getTheta() + ", " + getFilteredData() + "\n"));
                writer.close();
            }
            catch (Exception e) {

            }
        }
    }

    // Constants needed for this lab
    public enum LocalizationType {
        FALLING_EDGE,
        RISING_EDGE
    }
}

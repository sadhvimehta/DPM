package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

public class OdometryCorrection extends Thread {

    private static final long CORRECTION_PERIOD = 10; // all the variables used for the correction
    private static final long LOOP_TIME = 10;
    private static final double SQUARE_SIDE = 30.48;
    private static final double SENSOR_DISTANCE = 6.35;
    private static final double TAU = Math.PI * 2;

    TextLCD temp;
    ArrayList<Float[]> lastNValueL = new ArrayList<>();
    ArrayList<Float[]> lastNValueR = new ArrayList<>();
    private Odometer odometer;
    private EV3ColorSensor colorSensorL, colorSensorR;
    private float[] colorDataL, colorDataR;
    private long correctionStart, correctionEnd, startTime;
    private float[] biggestL, biggestR, smallestL, smallestR;
    private boolean leftHasPastLine, rightHasPastLine;
    private float[] lastLeftLine, lastRightLine;
    private float radOff;
    private double[] XYOff;

    private double ds;
    private double tandssd;

    private Navigation navigation;


    public OdometryCorrection(Odometer odometer, TextLCD temp, EV3ColorSensor colorSensorL, EV3ColorSensor colorSensorR, float[] colorDataL, float[] colorDataR, Navigation navigation) {

        this.odometer = odometer; // instantiates the odometer

        this.colorSensorL = colorSensorL;
        this.colorSensorR = colorSensorR;
        this.colorDataL = colorDataL;
        this.colorDataR = colorDataR;

        this.temp = temp; // only used to display brightness on the screen

        Sound.setVolume(30); // allows us to hear the beep for when a line has been crossed

        this.navigation = navigation;

        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter("log.txt"));
            writer.close();
        } catch (Exception e) {

        }
    }

    private float[] getFilteredData() {
        correctionStart = System.currentTimeMillis();

        colorSensorL.getRedMode().fetchSample(colorDataL, 0);
        colorSensorR.getRedMode().fetchSample(colorDataR, 0);
        float colorL = colorDataL[0] * 100;
        float colorR = colorDataR[0] * 100;

        for (float color : new float[] {colorL, colorR}){
            if (color > 100) {
                color = 100;
            }
            else if (color < -100) {
                color = -100;
            }
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

        float[] colors = new float[] {colorL, colorR};

        return colors;
    }

    private void leftPastline() { // the idea of this filter is to only look at an N number of previous values
        biggestL = new float[]{-200, -1000}; // since crossing a line causes a drop and a rise in the derivative, the filter
        smallestL = new float[]{200, 1000}; // only considers a line crossed if the biggest value is higher than some threshold
        for (int i = 0; i < lastNValueL.size(); i++) {
                                            // and the reverse for the lowest value, thus creating one beep per line
            if (lastNValueL.get(i)[0] > biggestL[0]) {
                biggestL[0] = lastNValueL.get(i)[0];
                biggestL[1] = lastNValueL.get(i)[1];
            }
            if (lastNValueL.get(i)[0] < smallestL[0]) {
                smallestL[0] = lastNValueL.get(i)[0];
                smallestL[1] = lastNValueL.get(i)[1];
            }
        }
        if (biggestL[0] > 5 && smallestL[0] < -5) { // if a sample is considered to be a line, the array is cleared as to not
            // retrigger an other time
            lastNValueL.clear();
            leftHasPastLine = true;
            lastLeftLine = new float[] {smallestL[1]};
        }
    }

    private void rightPastline() { // the idea of this filter is to only look at an N number of previous values
        biggestR = new float[]{-200, -1000}; // since crossing a line causes a drop and a rise in the derivative, the filter
        smallestR = new float[]{200, 1000}; // only considers a line crossed if the biggest value is higher than some threshold
        for (int i = 0; i < lastNValueR.size(); i++) {
            // and the reverse for the lowest value, thus creating one beep per line
            if (lastNValueR.get(i)[0] > biggestR[0]) {
                biggestR[0] = lastNValueR.get(i)[0];
                biggestR[1] = lastNValueR.get(i)[1];
            }
            if (lastNValueR.get(i)[0] < smallestR[0]) {
                smallestR[0] = lastNValueR.get(i)[0];
                smallestR[1] = lastNValueR.get(i)[1];
            }
        }
        if (biggestR[0] > 5 && smallestR[0] < -5) { // if a sample is considered to be a line, the array is cleared as to not
            // retrigger an other time
            lastNValueR.clear();
            rightHasPastLine = true;
            lastRightLine = new float[] {smallestR[1]};
        }
    }

    public void lastNValueLAdd(float brightness) {
        Float[] entry = {brightness, (float) System.currentTimeMillis() - startTime};
        // the array doesnt take chucks of the values and risk to miss a big difference.
        // Instead we slide along
        if (lastNValueL.size() > 50) {
            // the oldest value is removed to make space
            lastNValueL.remove(0);
            lastNValueL.add(entry);
        } else {
            // if the array happens to be less that, simply add them
            lastNValueL.add(entry);
        }
    }

    public void lastNValueRAdd(float brightness) {
        Float[] entry = {brightness, (float) System.currentTimeMillis() - startTime};
        // the array doesnt take chucks of the values and risk to miss a big difference.
        // Instead we slide along
        if (lastNValueR.size() > 50) {
            // the oldest value is removed to make space
            lastNValueR.remove(0);
            lastNValueR.add(entry);
        } else {
            // if the array happens to be less that, simply add them
            lastNValueR.add(entry);
        }
    }

    public double closestX() {
        return Math.round(odometer.getX()/SQUARE_SIDE) * SQUARE_SIDE;
    }

    public double closestY() {
        return Math.round(odometer.getY() / SQUARE_SIDE) * SQUARE_SIDE;

    }

    public boolean isCloseXYAxis() {
        if (isCloseXAxis() && isCloseYAxis()){
            return true;
        }
        else {
            return false;
        }
    }

    public boolean isCloseXAxis() {
        if (odometer.getX() < closestX() + 3 && odometer.getX() > closestX() - 3) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean isCloseYAxis() {
        if (odometer.getY() < closestY() + 3 && odometer.getY() > closestY() - 3) {
            return true;
        } else {
            return false;
        }
    }

    public EV3LargeRegulatedMotor firstWheel() {
        if (lastLeftLine[0] < lastRightLine[0]/*smallestL[1] < smallestR[1]*/) {
            return LabFiveMain.rightMotor;
        } else {
            return LabFiveMain.leftMotor;
        }
    }

    public double calculateRadOff() {
        if (firstWheel() == LabFiveMain.rightMotor) {
            ds = Math.abs((System.currentTimeMillis() - startTime) - lastRightLine[0]) / 10000 * (firstWheel().getSpeed() * LabFiveMain.WHEEL_RADIUS * TAU/ 360);
        }
        else {
            ds = Math.abs((System.currentTimeMillis() - startTime) - lastLeftLine[0]) / 10000 * (firstWheel().getSpeed() * LabFiveMain.WHEEL_RADIUS * TAU / 360);
        }

        tandssd = Math.atan( ds/ SENSOR_DISTANCE);
        if (firstWheel() == LabFiveMain.rightMotor) {
            //offset = /*TAU / 8*/0;
        }
        else {
            tandssd = TAU/4 - tandssd;
        }

        try {
            BufferedWriter writertemp = new BufferedWriter(new FileWriter("log.txt", true));
            writertemp.append("deltas: " + ds + " tandssd: "+ tandssd + " time between line and now: " + Math.abs((System.currentTimeMillis() - startTime) - lastRightLine[0]) / 10000 + "cm per s: " + (firstWheel().getSpeed() * LabFiveMain.WHEEL_RADIUS * TAU / 360) +"\n"); // writes to the log file the time and the brightness
            writertemp.close();
        } catch (IOException e) {
            System.console().printf("no error here!");
        }

        return tandssd;
    }

    public double[] calculateXYOff() {
        double radoff = calculateRadOff();
        double a = 5.507;
        double b = Math.sqrt(Math.pow(ds, 2) + Math.pow(SENSOR_DISTANCE, 2)) * Math.sin(TAU/4 - 2 * radoff);
        double c = Math.pow(a, 2) + Math.pow(b, 2) - 2 * a * b * Math.cos(TAU/6.569 - 2 * radoff);
        double A = Math.asin(a * Math.sin(TAU/6.569 + 2 * radoff)/c);
        double x;
        double y;
        if (firstWheel() == LabFiveMain.rightMotor){
            x = c * Math.sin(A);
            y = c * Math.cos(A);
        }
        else {
            x = c * Math.cos(A);
            y = c * Math.sin(A);
        }

        try {
            BufferedWriter writertemp = new BufferedWriter(new FileWriter("log.txt", true));
            writertemp.append("a: " + a + " b: " + b + " c: " + c + " A: " + A + "\n"); // writes to the log file the time and the brightness
            writertemp.close();
        } catch (IOException e) {
            System.console().printf("no error here!");
        }

        return new double[] {x, y};
    }

    public void start() {
        float[] lastBrightness = {0, 0}; // this variable and the next one are used to apply a differentiation filter to the data
        float[] presentBrightness; // the differentiation filter allows the robot to still operate regardless of the light level
        float[] dbdt = {0, 0}; // the instantaneous differentiation of the brightness d/dt(brightness)
        boolean firstpassby = true; // since there is no past value for the first value, this sets helps set the last
        // value for the first time running

        startTime = System.currentTimeMillis();

        try {
            BufferedWriter writertemp = new BufferedWriter(new FileWriter("log.txt"));
            writertemp.close();
        } // simply creates the log file / clear the previous log file
        catch (IOException e) {
            System.console().printf("error here");
        }

        while (true) {
            correctionStart = System.currentTimeMillis();

            presentBrightness = getFilteredData();


            if (firstpassby) {
                lastBrightness = presentBrightness; // simply sets the previous brightness to the current brightness if the
                // code has been run for the first time
                firstpassby = false;
            }

            for (int i = 0; i < 2; i++) {
                dbdt[i] = presentBrightness[i] - lastBrightness[i]; // get the current brightness and sets the derivative
            }

            lastBrightness = presentBrightness; // update the last brightness

            temp.drawString(String.valueOf(dbdt), 0, 3); // writes the derivative on the brick

            // TODO Place correction implementation here

            /*try {
                BufferedWriter writertemp = new BufferedWriter(new FileWriter("log.txt", true));
                // writertemp.append(String.valueOf(correctionStart - startTime) + ", " +
                // String.valueOf(colorData[0] * 100) + "\n");
                writertemp.append(
                        String.valueOf(correctionStart - startTime) + ", " + String.valueOf(dbdt[1]) + "\n"); // writes to the log file the time and the brightness
                writertemp.close();
            } catch (IOException e) {
                System.console().printf("no error here!");
            }*/

            lastNValueLAdd(dbdt[0]); // adds the derivative to a filter (explained lower down)
            lastNValueRAdd(dbdt[1]);

            /*if (this.pastline()) { // if the filter has sensed that a line has been crossed:
                Sound.beep();
                if (odometer.getTheta() > Math.PI / 4 && odometer.getTheta() < 3 * Math.PI / 4) { // right    // and the robot is moving right
                    odometer.setX(x * 30.48);
                } else if (odometer.getTheta() > 7 * Math.PI / 4 || odometer.getTheta() < Math.PI / 4) { // up       // and the robot is moving up
                    odometer.setY(y * 30.48);
                } else if (odometer.getTheta() > 5 * Math.PI / 4 && odometer.getTheta() < 7 * Math.PI / 4) { // left     // and the robot is moving left
                    odometer.setX(x * 30.48);
                } else if (odometer.getTheta() > 3 * Math.PI / 4 && odometer.getTheta() < 5 * Math.PI / 4) { // down     // and the robot is moving down
                    odometer.setY(y * 30.48);
                }
            }*/

            leftPastline();
            rightPastline();

            if (leftHasPastLine && rightHasPastLine) {
                if (true/*isCloseXYAxis()*/) {
                    Sound.setVolume(30);
                    Sound.beep();

                    radOff = (float) calculateRadOff();
                    XYOff = calculateXYOff();

                    RegulatedMotor[] synclist = {LabFiveMain.rightMotor};
                    LabFiveMain.leftMotor.synchronizeWith(synclist);
                    LabFiveMain.leftMotor.startSynchronization();
                    LabFiveMain.leftMotor.stop(true);
                    LabFiveMain.rightMotor.stop(true);
                    LabFiveMain.leftMotor.endSynchronization();

                    try {
                        BufferedWriter writertemp = new BufferedWriter(new FileWriter("log.txt", true));
                        writertemp.append("1xy: " + odometer.getX()+ ", "+odometer.getY() + " rad: "+ odometer.getTheta() + "\n"); // writes to the log file the time and the brightness
                        writertemp.close();
                    } catch (IOException e) {
                        System.console().printf("no error here!");
                    }

                    try {
                        BufferedWriter writertemp = new BufferedWriter(new FileWriter("log.txt", true));
                        writertemp.append("calculated xy: " + XYOff[0] + ", " + XYOff[1] + " rad: " + radOff + "\n"); // writes to the log file the time and the brightness
                        writertemp.close();
                    } catch (IOException e) {
                        System.console().printf("no error here!");
                    }

                    odometer.setX(-XYOff[0]);
                    odometer.setY(-XYOff[1]);
                    odometer.setTheta(odometer.getTheta() + radOff);

                    try {
                        BufferedWriter writertemp = new BufferedWriter(new FileWriter("log.txt", true));
                        writertemp.append("xy: " + odometer.getX() + ", " + odometer.getY() + " rad: " + odometer.getTheta() + "\n"); // writes to the log file the time and the brightness
                        writertemp.close();
                    } catch (IOException e) {
                        System.console().printf("no error here!");
                    }

                    leftHasPastLine = false;
                    rightHasPastLine = false;
                    navigation.travelTo(0, 0);
                }
                else if (isCloseXAxis()) {

                }
                else if (isCloseYAxis()) {

                }
            }


            // this ensure the odometry correction occurs only once every period
            correctionEnd = System.currentTimeMillis();
            if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
                try {
                    Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
                } catch (InterruptedException e) {
                    // there is nothing to be done here because it is not
                    // expected that the odometry correction will be
                    // interrupted by another thread
                }
            }
        }
    }

}

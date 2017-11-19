package ca.mcgill.ecse211.finalproject.controller;

import ca.mcgill.ecse211.finalproject.main.CaptureFlagMain;
import ca.mcgill.ecse211.finalproject.odometry.Odometer;
import ca.mcgill.ecse211.finalproject.sensor.LightController;
import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

import java.util.ArrayList;

public class TwoLightLocalization{

	private static final long CORRECTION_PERIOD = 10; // all the variables used for the correction
	private static final long LOOP_TIME = 10;
	private static final double SQUARE_SIDE = 30.48;
	private static final double SENSOR_DISTANCE = 6.35;
	private static final double TAU = Math.PI * 2;

	ArrayList<Float[]> lastNValueL = new ArrayList<>();
	ArrayList<Float[]> lastNValueR = new ArrayList<>();
	private Odometer odometer;
	private SampleProvider colorSensorL, colorSensorR;
	private float[] colorDataL, colorDataR;
	private long correctionStart, correctionEnd, startTime;
	private float[] biggestL, biggestR, smallestL, smallestR;
	private boolean leftHasPastLine, rightHasPastLine;
	private float[] lastLeftLine, lastRightLine;
	private float radOff;
	private double[] XYOff;

	private float ds;
	private double tandssd;

	private Navigation navigation;
	
	private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private final int ROTATE_SPEED = 150;


	public TwoLightLocalization(Odometer odometer, SampleProvider colorSensorL, SampleProvider colorSensorR,
			float[] colorDataL, float[] colorDataR, Navigation navigation, EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {

		this.odometer = odometer; // instantiates the odometer

		this.colorSensorL = colorSensorL;
		this.colorSensorR = colorSensorR;
		this.colorDataL = colorDataL;
		this.colorDataR = colorDataR;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		Sound.setVolume(30); // allows us to hear the beep for when a line has been crossed

		this.navigation = navigation;
	}

	// performs initial light localization after ultrasonic to get x,y,theta offset
	public void doTwoLightLocalization(){
		
		float[] lastBrightness = {0, 0};
		float[] currentBrightness = {0,0};
		float[] dbdt = {0, 0}; // the instantaneous differentiation of the brightness d/dt(brightness)
		boolean passLeftLine = false;
		boolean passRightLine = false;
		
		lastNValueLAdd(dbdt[0]); // adds the derivative to a filter (explained lower down) for the first pass
		lastNValueRAdd(dbdt[1]);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
		leftMotor.forward();
		rightMotor.forward();
		leftPastline();
		rightPastline();

		while(leftHasPastLine == false && rightHasPastLine == false){
			passLeftLine = leftPastline(); 
			passRightLine = rightPastline(); 
			currentBrightness = getFilteredData();
			System.out.println("left " + currentBrightness[0]);
			System.out.println("right " + currentBrightness[1]);
			for (int i = 0; i < 2; i++) {
				dbdt[i] = currentBrightness[i] - lastBrightness[i]; // get the current brightness and sets the derivative
			}
			lastNValueLAdd(dbdt[0]); // adds the derivative to a filter (explained lower down)
			lastNValueRAdd(dbdt[1]);
			lastBrightness = currentBrightness;
		}
		if(leftHasPastLine){
			System.out.println("LEFT STOPPED");
			leftMotor.stop(false);
			rightMotor.stop(false);
			rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
			rightMotor.rotate(Navigation.convertDistance(CaptureFlagMain.WHEEL_RADIUS, 5.00));
			rightMotor.setSpeed((int) (ROTATE_SPEED * CaptureFlagMain.balanceConstant));
			while(rightPastline() == false){
				rightMotor.backward();
			}
			rightMotor.stop(false);
		}
		
		if(rightHasPastLine){
			System.out.println("RIGHT STOPPED");
			rightMotor.stop(false);
			leftMotor.stop(false);
			leftMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(Navigation.convertDistance(CaptureFlagMain.WHEEL_RADIUS, 5.00));
			leftMotor.setSpeed(ROTATE_SPEED);
			while(leftPastline() == false){
				leftMotor.backward();
			}
			leftMotor.stop(false);
		}
		
	
	}
	public float[] getFilteredData() {
		correctionStart = System.currentTimeMillis();

		colorSensorL.fetchSample(colorDataL, 0);
		colorSensorR.fetchSample(colorDataR, 0);
		float colorL = colorDataL[0] * 100;
		float colorR = colorDataR[0] * 100;

		for (float color : new float[]{colorL, colorR}) {
			if (color > 100) {
				color = 100;
			} else if (color < -100) {
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

		float[] colors = new float[]{colorL, colorR};

		return colors;
	}

	public boolean leftPastline() { // the idea of this filter is to only look at an N number of previous values
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
			lastLeftLine = new float[]{smallestL[1]};
			return true;
		}
		else
			return false;
	}

	public boolean rightPastline() { // the idea of this filter is to only look at an N number of previous values
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
			lastRightLine = new float[]{smallestR[1]};
			return true;
		}
		else
			return false;
	}

	//left
	public void lastNValueLAdd(float brightness) {
		Float[] entry = {brightness, (float) System.currentTimeMillis() - startTime};
		// the array doesnt take chunks of the values and risk to miss a big difference.
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

	//right
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
		return Math.round(odometer.getX() / SQUARE_SIDE) * SQUARE_SIDE;
	}

	public double closestY() {
		return Math.round(odometer.getY() / SQUARE_SIDE) * SQUARE_SIDE;

	}

	public boolean isCloseXYAxis() {
		if (isCloseXAxis() && isCloseYAxis()) {
			return true;
		} else {
			return false;
		}
	}

	public boolean isCloseXAxis() {
		if (odometer.getX() < closestX() + 3 && odometer.getX() > closestX() - 3) {
			return true;
		} else {
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
			return CaptureFlagMain.rightMotor;
		} else {
			return CaptureFlagMain.leftMotor;
		}
	}

	public double calculateRadOff() {
		if (firstWheel() == CaptureFlagMain.rightMotor) {
			ds = Math.abs((System.currentTimeMillis() - startTime) / 1000 - lastRightLine[0]) * (float) (firstWheel().getSpeed() * CaptureFlagMain.WHEEL_RADIUS * TAU / 360);
		} else {
			ds = Math.abs((System.currentTimeMillis() - startTime) / 1000 - lastLeftLine[0]) * (float) (firstWheel().getSpeed() * CaptureFlagMain.WHEEL_RADIUS * TAU / 360);
		}

		tandssd = Math.atan(ds / SENSOR_DISTANCE);
		double offset;
		if (isCloseXYAxis()) {
			offset = /*TAU / 8*/0;
		} else {
			offset = 0;
		}

		return tandssd + offset;
	}

	public double[] calculateXYOff() {
		double radoff = calculateRadOff();
		double a = 5.507;
		double b = Math.sqrt(Math.pow(ds, 2) + Math.pow(SENSOR_DISTANCE, 2)) * Math.sin(TAU / 4 - 2 * radoff);
		double c = Math.pow(a, 2) + Math.pow(b, 2) - 2 * a * b * Math.cos(TAU / 6.569 - 2 * radoff);
		double A = Math.asin(a * Math.sin(TAU / 6.569 + 2 * radoff) / c);
		double x;
		double y;
		if (firstWheel() == CaptureFlagMain.rightMotor) {
			x = c * Math.sin(A);
			y = c * Math.cos(A);
		} else {
			x = c * Math.cos(A);
			y = c * Math.sin(A);
		}

		return new double[]{x, y};
	}

}

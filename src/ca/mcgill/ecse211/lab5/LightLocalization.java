package ca.mcgill.ecse211.lab5;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


public class LightLocalization extends Thread{

	private Navigation navigation;
	private Odometer odo;
	private SampleProvider colorValue;
	private SensorModes csSensor;
	private float[] colorData;
	private double lightValueCurrent, lightValuePrev;
	private double thetaX;
	private double thetaY;
	private double positionX;
	private double positionY;
	private double dT;
	private static Port csPort = LocalEV3.get().getPort("S2");
	
	
	private boolean atApproxOrigin = false; 
	
	private int lineCounter;
	private double[] saveLineAngles;
	
	private final float LIGHT_DIFF_THRESHOLD = 40;
	private static final int WAIT_PERIOD = 1000; //in milliseconds
	private final double SENSOR_TO_WHEEL = 14.0; //distance between the wheels and the sensor
	private final double LINE_OFFSET = SENSOR_TO_WHEEL * 1.5;
	
	private EV3LargeRegulatedMotor leftMotor,rightMotor;

	
	public LightLocalization(Navigation navigation, Odometer odo, EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.navigation = navigation;
		this.csSensor= new EV3ColorSensor(csPort);
		this.colorValue = csSensor.getMode("Red"); //red light sensor because we need to measure the intensity of the reflected red light (black vs light wood)
		this.colorData = new float[csSensor.sampleSize()];
		  
		
	}
	
	public void run() {
		doLocalization();
	}

	public void doLocalization() {	
		//1st, get the robot close to where the origin is
		
		goToEstimateOrigin();
		
		//2nd, turn around the origin and detect the lines
		checkLines();
		
		calculatePosition();
	
		goToOrigin();
		
	}		
	
	//Polls the color sensor
		private float getData() {
			colorValue.fetchSample(colorData, 0);
			float color = colorData[0] * 1000;
					
			return color;
		}

		private void goToEstimateOrigin(){
			
			//turn 45 degrees to face origin
			this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
			this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);	
			this.leftMotor.rotate(Navigation.convertAngle(LabFiveMain.WHEEL_RADIUS,LabFiveMain.TRACK,45),true);
			this.rightMotor.rotate(-Navigation.convertAngle(LabFiveMain.WHEEL_RADIUS,LabFiveMain.TRACK,45),false);
			
			//then go straight
			this.leftMotor.setSpeed(Navigation.FORWARD_SPEED);
			this.rightMotor.setSpeed(Navigation.FORWARD_SPEED);	
			this.leftMotor.forward();
			this.rightMotor.forward();
			
			lightValuePrev = getData();    //save previous value, aka bord colour
					
			while(!atApproxOrigin){ //boolean to check if we have arrived or not
				lightValueCurrent = getData(); //update data

				//If the difference in colour intensity is bigger than a chosen threshold, a line was detected
				if(lightValueCurrent <= 350){ //if we detect a line
					//Sound.beep();
					this.leftMotor.setSpeed(0);
					this.rightMotor.setSpeed(0);		//stop
					atApproxOrigin = true;
					Sound.beep();
				}
				lightValuePrev = lightValueCurrent;
			
			}
			//Sound.beep();
			//go backwards so the front wheels are at the origin and not the sensor
			this.leftMotor.setSpeed(Navigation.FORWARD_SPEED);
			this.rightMotor.setSpeed(Navigation.FORWARD_SPEED);
			
			this.leftMotor.rotate(-Navigation.convertDistance(LabFiveMain.WHEEL_RADIUS, LINE_OFFSET),true);
			this.rightMotor.rotate(-Navigation.convertDistance(LabFiveMain.WHEEL_RADIUS, LINE_OFFSET),false);	
			
		}
		
		private void checkLines(){
			//it turns anti clockwise, so 1st line it sees in neg y, then pos x, then pos y, then neg x
			
			//Set up variables
			lineCounter = 0;
			saveLineAngles = new double[4];
			
			
			this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
			this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);	
			this.leftMotor.rotate(Navigation.convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, 360), true);
			this.rightMotor.rotate(-Navigation.convertAngle(LabFiveMain.WHEEL_RADIUS, LabFiveMain.TRACK, 360), true);
			
			
					//Runs until it has detected 4 lines
					while(lineCounter < 4){
						lightValueCurrent = getData();  
						//If the difference in colour intensity is bigger than a chosen threshold, a line was detected
						if(lightValueCurrent <= 350) 	
						{
							//Store angles in variable for future calculations
							//saveLineAngles[lineCounter] = saveLinePosition[2];
							saveLineAngles[lineCounter] = this.odo.getTheta();
							
							Sound.beep();
							
							lineCounter++;
							
							//Makes the thread sleep as to not detect the same line twice
							sleepThread();
							
						}
											
					}
					
					this.leftMotor.setSpeed(0);
					this.rightMotor.setSpeed(0);
			
		}
		
		private void calculatePosition(){
			//Trigonometry calculations from tutorial
			thetaY = saveLineAngles[3] - saveLineAngles[1]; //Y+ - Y-
			thetaX = saveLineAngles[2] - saveLineAngles[0];	//X+ - X-
			
			//positionY = LINE_OFFSET *Math.cos(Math.toRadians(thetaY/2));
			//positionX = LINE_OFFSET *Math.cos(Math.toRadians(thetaX/2));
			
			//positionX = -SENSOR_TO_WHEEL*Math.cos(Math.PI*thetaX/(360));
			//positionY = -SENSOR_TO_WHEEL*Math.cos(Math.PI*thetaY/(360));
			
			positionX = -SENSOR_TO_WHEEL*Math.cos((thetaX)/2);
			positionY = -SENSOR_TO_WHEEL*Math.cos((thetaY)/2);
					
			
			dT = 270 + (thetaY/2) - saveLineAngles[3]; //y-
			
			//Wraps the angle for positive Y axis
			//if (dT > 180) {
			//	dT += 180;
			//}
			
			//Updates odometer to actual values
			this.odo.setX(positionX);
			this.odo.setY(positionY);
			//this.odo.setTheta(Math.atan2(positionY, positionX)+180);
			this.odo.setTheta(this.odo.getTheta() + dT);
						
			//Sound.twoBeeps();
			//Travel to origin	
			
		}
		
		
		private void goToOrigin(){
			
			
			navigation.travelTo(0, 0);			
			Sound.beep();
			//Turn to 0deg
			/*this.leftMotor.setSpeed(Navigation.ROTATE_SPEED);
			this.rightMotor.setSpeed(Navigation.ROTATE_SPEED);	
			this.leftMotor.rotate(Navigation.convertAngle(LocalizationLab.WHEEL_RADIUS,LocalizationLab.TRACK,0),true);
			this.rightMotor.rotate(-Navigation.convertAngle(LocalizationLab.WHEEL_RADIUS,LocalizationLab.TRACK,0),false);
				*/
			navigation.turnTo(0);
			
			
		}
		
		public static void sleepThread() {
			try {
				Thread.sleep(WAIT_PERIOD);
			} catch (InterruptedException e) {
			}
		}
		
		
		
}


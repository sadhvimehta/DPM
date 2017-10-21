package ca.mcgill.ecse211.lab5;



import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class FeedBackNavigation extends Thread {
	
	private static final double TILE_SIZE = 30.48;
	
	public static final int FORWARD_SPEED = 200;
	public static final int ROTATE_SPEED = 100;
	private double minAngle;
	public Odometer odometer;
	public EV3LargeRegulatedMotor leftMotor;
	public EV3LargeRegulatedMotor rightMotor;
	public double destAngle;
	private static final double DIST_ERROR = 1.0; 
	
	//constructor
		public FeedBackNavigation(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
			this.odometer = odometer;
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
			
		}
		
		public void run() {
			travelTo(2*TILE_SIZE,2*TILE_SIZE);
		}
		
		
		//travel to location(x,y)
		public void travelTo (double x, double y){ 
			
			//calculate minAngle and turn
			minAngle = calculateMinAngle(x,y);
			turnTo(minAngle);
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();		
			
			//while the robot doesn't arrive its destination, keep going straight
			while(!hasArrivedDestination(x,y)){ 
			
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);
				leftMotor.forward();
				rightMotor.forward();

			}
			
			//after the robot arrived, stop motors
			leftMotor.setSpeed(0); 
			rightMotor.setSpeed(0);
		}
		
		
		//method to turn to theta degree
		public void turnTo(double theta) {
			
			//find minimum angle to turn
			double angle = theta - this.odometer.getTheta();
			
			if (angle < - Math.PI) {
				angle = angle+ 2*Math.PI;
			}
			
			if (angle > Math.PI) {
				angle -= 2*Math.PI;
			}
			
			if(angle ==0 || angle == 2*Math.PI) {
				angle = 0;
			}
			
		
			this.leftMotor.setSpeed(ROTATE_SPEED);
			this.rightMotor.setSpeed(ROTATE_SPEED);	
			this.leftMotor.rotate(convertAngle(LabFiveMain.WHEEL_RADIUS,LabFiveMain.TRACK,Math.toDegrees(angle)),true);
			this.rightMotor.rotate(-convertAngle(LabFiveMain.WHEEL_RADIUS,LabFiveMain.TRACK,Math.toDegrees(angle)),false);
			Sound.beep();
			
		}
		//calculate minimum angle to turn
		public double calculateMinAngle(double x, double y)
		{
			double angle = Math.atan2(x - this.odometer.getX(), y - this.odometer.getY());
			angle  = angle - this.odometer.getTheta();
			if(angle==0 || angle == 2*Math.PI)
			{
				angle = 0;
			}
			else if(angle >  Math.PI)
			{
				angle -= (2*Math.PI);
			}
			else if(angle < -Math.PI){
				
				angle += (2*Math.PI);
			}
			
			
			return angle;
			
		}
		
		public boolean hasArrivedDestination(double x, double y){
			
			if (Math.abs(y-this.odometer.getY())<= DIST_ERROR && Math.abs(x-this.odometer.getX()) <= DIST_ERROR){
				return true;	//return true when arrived destination
			}
			else{
				return false;
			}
		}
		
		//method to set Speeds of motors
		 public void setSpeeds(int lSpd, int rSpd) {
				this.leftMotor.setSpeed(lSpd);
				this.rightMotor.setSpeed(rSpd);
				if (lSpd < 0)
					this.leftMotor.backward();
				else
					this.leftMotor.forward();
				if (rSpd < 0)
					this.rightMotor.backward();
				else
					this.rightMotor.forward();
			}
		 
		 //method to stop the motors
		 public void stopMotor() {
				this.leftMotor.stop(true);
				this.rightMotor.stop(false);
			}
		 
		 public static int convertDistance(double radius, double distance) {
			    return (int) ((180.0 * distance) / (Math.PI * radius));
		 }

		 public static int convertAngle(double radius, double width, double angle) {
			    return convertDistance(radius, Math.PI * width * angle / 360.0);
		 }
}

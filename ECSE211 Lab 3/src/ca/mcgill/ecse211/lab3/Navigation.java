package ca.mcgill.ecse211.lab3;



import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;
public class Navigation extends Thread{
	
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	
	private static final int FORWARD_SPEED = 200;
    private static final int ROTATE_SPEED = 120;
	private static final double TILE_SIZE = 30.48;
	
	private  double LastX,LastY,LastT; 
	
	private boolean navigation;
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,Odometer odometer) throws OdometerExceptions {

		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.navigation = false;
		}
	
	private void travelTo(double x, double y) {
		//This  method  causes  the  robot  to  travel  to  the  absolute  field  location  (x,  y)
		
		LastX = odometer.getXYT()[0];//The initial value of x,y, and theta
		LastY = odometer.getXYT()[1];
		LastT = odometer.getXYT()[2];
		
		double deltaX = x - LastX;// Calculate the change in the values of x, y,and theta.
		double deltaY = y - LastY;
		double deltaT = Math.atan2(deltaX, deltaY) - LastT;
	    double Distance = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);//calculate the distance need to travel
	
	    turnTo(deltaT);// turn with the degree theta
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
	

		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, Distance), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, Distance), false);
		
		leftMotor.stop(true);// stop the robot
		rightMotor.stop(true);
		this.navigation = true;
	}

	
	public void run() {
	
	//Map 1
		
		travelTo(0.0, 2 * TILE_SIZE);
		travelTo(TILE_SIZE, TILE_SIZE);
		travelTo(2 * TILE_SIZE, 2 * TILE_SIZE);
	    travelTo(2*TILE_SIZE, TILE_SIZE);
	    travelTo(TILE_SIZE, 0.0);
			
			

//	//Map 2
//	
//	    travelTo(TILE_SIZE,TILE_SIZE);
//	    travelTo(0.00       ,2*TILE_SIZE);
//	    travelTo(2*TILE_SIZE,2*TILE_SIZE);
//	    travelTo(2*TILE_SIZE,TILE_SIZE);
//	    travelTo(TILE_SIZE, 0.00);
//	    
//	//Map 3 
//	    
//	    travelTo(TILE_SIZE,0.00);
//	    travelTo(2*TILE_SIZE,TILE_SIZE);
//	    travelTo(2*TILE_SIZE,2*TILE_SIZE);
//	    travelTo(0.00       ,2*TILE_SIZE);
//	    travelTo(TILE_SIZE, TILE_SIZE);
//	    
//	//Map 4
//	    travelTo(0.00,   TILE_SIZE);
//	    travelTo(TILE_SIZE, 2*TILE_SIZE);
//	    travelTo(TILE_SIZE, 0.00);
//	    travelTo(2*TILE_SIZE, TILE_SIZE);
//	    travelTo(2*TILE_SIZE, 2*TILE_SIZE);
//	    
	
	
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	private void turnTo(double theta) {
		//this method causes the robot to turn (on point) to the absolute headingtheta. 
		//This method should turn a MINIMAL angle to its target.
		theta = theta * 180.0 / Math.PI;// convert the angle from radian to degree
		
		leftMotor.setSpeed(ROTATE_SPEED);// Set speed to move in a straight line
		rightMotor.setSpeed(ROTATE_SPEED);
		
		if (theta <= -180) {//when the angle is larger than 180 degree
			theta += 180* 2;
		} else if (theta > 180) {//when the angle is smaller than 180 degree
			theta -= 180 * 2;
		}
		
		if (theta < 0) {// if the angle is negative, the robot will turn left
			leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -theta), true);
			rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -theta), false);

		} else {//if the angle is positive, the robot will turn right
			
			leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, theta ), true);
			rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, theta), false);
		}
		
		
	}

	  private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }//number of wheel turns, to make it easier to calculate theta

		  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);// convert the angle unit from radian
		  }
}

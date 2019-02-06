/**
 * This class extends Thread. It allows the robot to 
 * execute a simple navigation on a pre-defined path. 
 * It first instantiates the robot's motors and the 
 * odometers, then it follows one of the four maps that
 * are implemented in the class. 
 * 
 * Additionally, a constructor was set up for this class
 * so that it can be implemented on the main program, 
 * which is Lab3. 
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao SHu
 * @version 05-02-2019
 * 
 */


package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;
public class Navigation extends Thread{
	
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	
	private static final int FORWARD_SPEED = 200;
    private static final int ROTATE_SPEED = 120;
	public static final double TILE_SIZE = 30.48;
	
	private  double LastX,LastY,LastT; 
	
	private boolean isNavigating ;
	
	//constructor
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,Odometer odometer) throws OdometerExceptions {

		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.isNavigating = false;
		}
	

	// run method (required to implement Thread) 
	public void run() {//Travel to different coordinates in the maps
	
//	//Map 1
		
		travelTo(0.0, 2.0);
		travelTo(1.0, 1.0);
		travelTo(2.0, 2.0);
	    travelTo(2.0, 1.0);
	    travelTo(1.0, 0.0);
			
			
//	//Map 2
//	
//	    travelTo(1.0, 1.0);
//	    travelTo(0.0, 2.0);
//	    travelTo(2.0, 2.0);
//	    travelTo(2.0, 1.0);
//	    travelTo(1.0, 0.0);
	    
//	//Map 3 
//	    
//	    travelTo(1.0, 0.0);
//	    travelTo(2.0, 1.0);
//	    travelTo(2.0, 2.0);
//	    travelTo(0.0, 2.0);
//	    travelTo(1.0, 1.0);
//	    
//	//Map 4
//	    travelTo(0.0, 1.0);
//	    travelTo(1.0, 2.0);
//	    travelTo(1.0, 0.0);
//	    travelTo(2.0, 1.0);
//	    travelTo(2.0, 2.0);
	
	
	}
	/**
	 *This method commands the robot to travel to the absolute field location (x,  y).
	 * 
	 * @param x
	 * @param y
	 * @return void 
	 * 
	 */
	
	public void travelTo(double x, double y) {
		
		LastX = odometer.getXYT()[0];//The initial value of x,y, and theta
		LastY = odometer.getXYT()[1];
		LastT = odometer.getXYT()[2]* Math.PI/180;
		
		double deltaX = (x * TILE_SIZE) - LastX;// Calculate the change in the values of x, y,and theta.
		double deltaY = (y * TILE_SIZE) - LastY;
		double deltaT = Math.atan2(deltaX, deltaY) - LastT;
	    double Distance = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);//calculate the distance need to travel
	    Distance = Math.sqrt(Distance);
	    turnTo(deltaT);// turn with the degree theta
		
	    leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, Distance), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, Distance), false);
		
		rightMotor.stop(true);
		leftMotor.stop(true);// stop the robot
		
	
		isNavigating = false;
	}
	
 /**
 *This method causes the robot to turn (on point) to the 
 *absolute heading Theta .This method should turn a MINIMAL 
 *angle to its target.
 * 
 * @param theta
 * @return void
 * 
 */
	
	private void turnTo(double theta) {
		
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
	
	/**
	 *This method returns true if another thread has
	 * called travelTo() or turnTo() and the method 
	 * has yet to return; false otherwise.
	 * 
	 * @param void
	 * @return true/false
	 * 
	 */
	
	 boolean isNavigating() {
		if((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else 
			return false;
		}
	 
	 /**
		 * This method allows the conversion of a 
		 * distance to the total rotation of each
		 * wheel need to cover that distance.
		 * 
		 * @param Lab3.WHEEL_RAD
		 * @param distance
		 * @return (int)distance converted
		 * 
		 */

	  private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }
	  
	  /**
	   * This is a static method that converts the 
	   * angle needed to turn at a corner to the 
	   * equivalent total distance covered by the 
	   * rotation. 
	   * 
	   * @param radius
	   * @param width
	   * @param angle
	   * @return (int) rotation distance
	   * 
	   */

		  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }
}

package ca.mcgill.ecse211.lab5;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import ca.mcgill.ecse211.odometer.*;

/**
 * This class contains all the methods that contribute to making
 * the robot move
 * 
 * @author Carlo D'Angelo
 *
 */
public class Navigation {

  public final static int ROTATION_SPEED = 100;
  private final static int FORWARD_SPEED = 200; 
  
  private static final double TILE_SIZE = 30.48;
  
  private double radius = Lab5.WHEEL_RAD;
  private double track = Lab5.TRACK;
  
  private Odometer odo;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
 
  public static double minAng;

  /**
   * This is the default constructor of this class
   * @param leftMotor left motor of robot
   * @param rightMotor right motor of robot
   * @throws OdometerExceptions
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
	odo = Odometer.getOdometer();
	this.leftMotor = leftMotor;
	this.rightMotor = rightMotor;
	}
  
  /**
   * Method that allows robot to travel from its current position to any other point
   * @param x x coordinate of desired destination
   * @param y y coordinate of desired destination
   */
  public void travelTo(double x, double y) {
	double currentX = odo.getXYT()[0];
	double currentY = odo.getXYT()[1];

	double deltaX = x * TILE_SIZE - currentX;
	double deltaY = y * TILE_SIZE - currentY;
	
	double currentA = odo.getXYT()[2];
	
	// find minimum angle
	minAng = -currentA + Math.atan2(deltaX, deltaY) * 180 / Math.PI;
	
	// special cases so that robot does not turn at maximum angle
	if (minAng < 0 && (Math.abs(minAng) >= 180)) {
		minAng += 360;
	}else if (minAng > 0 && (Math.abs(minAng) >= 180)){
		minAng -= 360;
	}
	turnTo(minAng);
	
	double distance = Math.hypot(deltaX, deltaY);

	leftMotor.setSpeed(FORWARD_SPEED);
	rightMotor.setSpeed(FORWARD_SPEED);

	leftMotor.rotate(convertDistance(radius, distance), true);
	rightMotor.rotate(convertDistance(radius, distance), false);
	
  }

  /**
   * Method that allows the robot to rotate in place 
   * @param theta angle amount (in degrees) that you want the robot to rotate in place
   */
  public void turnTo(double theta) {
	leftMotor.setSpeed(ROTATION_SPEED);
	rightMotor.setSpeed(ROTATION_SPEED);
	leftMotor.rotate(convertAngle(radius, track, theta), true);
	rightMotor.rotate(-convertAngle(radius, track, theta), false);

  }
  
  /**
   * Method that allows the robot to move backward
   * @param distance distance that you want the robot to travel backward
   */
  public void driveBack(double distance) {
	  leftMotor.setSpeed(FORWARD_SPEED);
	  rightMotor.setSpeed(FORWARD_SPEED);

	  leftMotor.rotate(-convertDistance(radius, distance), true);
	  rightMotor.rotate(-convertDistance(radius, distance), false);
  }
  
  /**
   * Method that allows the robot to move forward
   * @param distance distance that you want the robot to travel forward
   */
  public void driveForward(double distance) {
	  leftMotor.setSpeed(FORWARD_SPEED);
	  rightMotor.setSpeed(FORWARD_SPEED);

	  leftMotor.rotate(convertDistance(radius, distance), true);
	  rightMotor.rotate(convertDistance(radius, distance), false);
  }
  
  /**
   * Method that allows the robot to approach a can
   * @param distance distance that you want the robot to travel forward
   */
  public void moveToCan(double distance) {
	  leftMotor.setSpeed(FORWARD_SPEED);
	  rightMotor.setSpeed(FORWARD_SPEED);

	  leftMotor.rotate(convertDistance(radius, distance), true);
	  rightMotor.rotate(convertDistance(radius, distance), false);
  }
  
	/**
	 * This method converts a distance into the total rotation (in degrees) of 
	 * each wheel needed to cover that distance 
	 * 
	 * @param radius radius of wheel
	 * @param distance distance that you want the robot to move
	 * @return total rotation (in degrees) of each wheel needed to cover a distance
	 */
	public static int convertDistance(double radius, double distance) {
	  return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * This method converts a rotation in place into the total 
	 * rotation (in degrees) of each wheel needed to cause that rotation
	 * 
	 * @param radius radius of wheel
	 * @param width distance between centers of wheels
	 * @param angle angle (in place) that you want the robot to rotate
	 * @return total rotation (in degrees) of each wheel needed to cause a rotation in place
	 */
	public static int convertAngle(double radius, double width, double angle) {
	  return convertDistance(radius, Math.PI * width * angle / 360.0);
	}  
  
}

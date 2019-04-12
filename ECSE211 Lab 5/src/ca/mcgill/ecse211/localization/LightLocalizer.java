package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab5.*;

/**
 * This class simulates the function of light localization
 * 
 * @author Carlo D'Angelo
 *
 */
public class LightLocalizer {

  public final static int ROTATION_SPEED = 100;
  private final static int FORWARD_SPEED = 150; 
  private static final double TILE_SIZE = 30.48;
  
  private final static int COLOUR_DIFF = 20;  
  private final static double LIGHT_LOC_DISTANCE = 14.5;
  private final static int EXTRA_DISTANCE = 5;
  private static final double TURN_ERROR = 10;
  
  private double radius = Lab5.WHEEL_RAD;
  private double track = Lab5.TRACK;
  
  private Odometer odo;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
 
  double[] linePosition;
  private Navigation navigator;
  
  private SampleProvider csLineDetector;
  private float[] csData;

  /**
   * This is the default constructor of this class
   * @param leftMotor left motor of robot
   * @param rightMotor right motor of robot
   * @param csLineDetector sample provider from which to fetch light sensor data
   * @param csData array in which to receive the light sensor data
   * @param navigator instance of Navigator class
   * @throws OdometerExceptions
   */
  public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		  				  SampleProvider csLineDetector, float[] csData, Navigation navigator) throws OdometerExceptions {
	odo = Odometer.getOdometer();
	this.leftMotor = leftMotor;
	this.rightMotor = rightMotor;
	linePosition = new double[4];
	this.csLineDetector = csLineDetector;
	this.csData = csData;
	this.navigator = navigator;
	}

  /**
   * Method that allows the robot to perform light localization
   * @param pointX x coordinate of desired localization point
   * @param pointY y coordinate of desired localization point
   */
  public void lightLocalize(double pointX, double pointY) {
	  
      leftMotor.setSpeed(ROTATION_SPEED);
	  rightMotor.setSpeed(ROTATION_SPEED);
	  
	  int count = 0;
	  float firstReading = readLineDarkness();
	  float sample;
	  while (count < 4) {

		leftMotor.forward();
		rightMotor.backward();

		sample = readLineDarkness();

		if (100*Math.abs(sample - firstReading)/firstReading > COLOUR_DIFF) {
          linePosition[count] = odo.getXYT()[2];
		  Sound.beep();
		  count++;
		}
	  }

	  leftMotor.stop(true);
	  rightMotor.stop();

	  double deltaX, deltaY, angleX, angleY, deltaA;

	  angleY = linePosition[3] - linePosition[1];
	  angleX = linePosition[2] - linePosition[0];

	  deltaX = -LIGHT_LOC_DISTANCE * Math.cos(Math.toRadians(angleY / 2));
	  deltaY = -LIGHT_LOC_DISTANCE * Math.cos(Math.toRadians(angleX / 2));
	  
	  deltaA = 90 - (angleY / 2.0) - TURN_ERROR;
	  
	  leftMotor.rotate(Navigation.convertAngle(radius, track, deltaA), true);
	  rightMotor.rotate(-Navigation.convertAngle(radius, track, deltaA));

	  odo.setXYT(pointX * TILE_SIZE + deltaX, pointY * TILE_SIZE + deltaY, 0.0);
	  
	  navigator.travelTo(pointX, pointY);

	  leftMotor.rotate(-Navigation.convertAngle(radius, track, Navigation.minAng), true);
	  rightMotor.rotate(Navigation.convertAngle(radius, track, Navigation.minAng));
	  
	  leftMotor.stop(true);
	  rightMotor.stop();

  }
  
  /**
   * Method that moves the robot closer to the localization point in preparation
   * for the actual light localization
   */
  public void moveClose() {
	
    navigator.turnTo(45);

	leftMotor.setSpeed(FORWARD_SPEED);
	rightMotor.setSpeed(FORWARD_SPEED);
	
	leftMotor.rotate(Navigation.convertDistance(radius, EXTRA_DISTANCE), true);
	rightMotor.rotate(Navigation.convertDistance(radius, EXTRA_DISTANCE));

  }
  
  /**
   * Method that fetches data from the light sensor
   * @return darkness (value between 0-1) of what the light sensor is reading multiplied by 1000 
   */
  private float readLineDarkness() {
	  csLineDetector.fetchSample(csData, 0);
	  return csData[0] * 1000;
  }
	
}
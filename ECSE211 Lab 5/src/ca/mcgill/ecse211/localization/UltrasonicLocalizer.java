package ca.mcgill.ecse211.localization;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab5.*;

/**
 * This class simulates the function of ultrasonic localization
 * 
 * @author Carlo D'Angelo
 *
 */
public class UltrasonicLocalizer {

	public static final int ROTATION_SPEED = 110;
	
	public static final double CRITICAL_DISTANCE = 30.00;
	public static final double NOISE_MARGIN = 2.00;

	private static final double TURN_ERROR = 15;
  
	private double radius = Lab5.WHEEL_RAD;
	private double track = Lab5.TRACK;
 
	private Odometer odo;
	private SampleProvider usDistance;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
  
	private double deltaTheta;
	
	/**
	 * This is the default constructor of this class
	 * @param leftMotor left motor of robot
	 * @param rightMotor right motor of robot
	 * @param usDistance sample provider from which to fetch ultrasonic sensor data
	 * @param usData array in which to receive the ultrasonic sensor data
	 * @throws OdometerExceptions
	 */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		SampleProvider usDistance, float[] usData) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usDistance = usDistance;
		this.usData = usData;

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}
	
	/**
	 * Method that allows the robot to perform falling edge localization 
	 */
	public void fallingEdge() {

		double angleA, angleB, turningAngle = 0;

		// Get first angle
		while (readUSDistance() < CRITICAL_DISTANCE + NOISE_MARGIN) {
			leftMotor.forward();
			rightMotor.backward();
		}
		
		while (readUSDistance() > CRITICAL_DISTANCE) {
			leftMotor.forward();
			rightMotor.backward();
		}
		
		angleA = odo.getXYT()[2];

		// Get second angle
		while (readUSDistance() < CRITICAL_DISTANCE + NOISE_MARGIN) {
			leftMotor.backward();
			rightMotor.forward();
		}

		while (readUSDistance() > CRITICAL_DISTANCE) {
			leftMotor.backward();
			rightMotor.forward();
		}
		angleB = odo.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// Calculation of angle that makes robot's heading face 0 degrees
		if (angleA < angleB) {
			deltaTheta = (360 - angleB) + ((angleA + angleB) / 2) - 225 + TURN_ERROR;
			turningAngle = deltaTheta;

		} else if (angleA > angleB) {
			deltaTheta = -angleB + (angleA + angleB) / 2 - 45 + TURN_ERROR;
			turningAngle = deltaTheta;
		}

		leftMotor.rotate(Navigation.convertAngle(radius, track, turningAngle), true);
		rightMotor.rotate(-Navigation.convertAngle(radius, track, turningAngle), false);
		odo.setTheta(0.0);

	}
	/**
	 * Method that fetches data from the ultrasonic sensor
	 * @return distance (cm) from the wall
	 */
	private int readUSDistance() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

}
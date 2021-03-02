

/**
 * This class implements the first step in the localization process, that is to say ultrasonic
 * localization. Essentially, it implements the ultrasonic sensor to filter distances through either
 * the falling edge or the raising edge process. Then, once the filtering is done, two angles (alpha
 * and beta), and the robot uses those to orient its heading in the direction of increasing y.
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu
 * @version 12-02-2019
 * 
 * 
 */

// import package
package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import java.util.Arrays;

import static ca.mcgill.ecse211.lab4.Lab4.*;
import static ca.mcgill.ecse211.lab4.Navigation.*;

public class UltrasonicLocalizer {

  
  /**
   * The ultrasonic data.
   */
  private static float[] usData = new float[usDistance.sampleSize()];

  /**
   * distance value to select appropriate readings from the ultrasonic sensor once it detects
   * abnormally large values
   */
  private static final double THRESHOLD = 22.00;// the value of d
  /**
   * distance value to select appropriate readings from the ultrasonic sensor once it detects
   * abnormally small values
   */
  private static final double MARGIN = 2.00;// the value of k
  
  /**
   * The angle recorded when the sensor detects the wall.
   */
  private static double alpha;
  private static double beta;
  private static double theta;
  private static double angleToTurn;
  
  /**
   * The offset for the turning angle.
   */
  private static double RISING_ANGLE = 180;

  static {
    LEFT_MOTOR.setSpeed(ROTATION_SPEED);
    RIGHT_MOTOR.setSpeed(ROTATION_SPEED);
  }

  /**
   * A method to localize position using the falling edge.
   */
  public static void fallingEdge() {
    // Rotate to open space
    while (medianFilter() < THRESHOLD + MARGIN) {
      counterclockwise();
    }
    // Rotate to the first wall
    while (medianFilter() > THRESHOLD) {
      counterclockwise();
    }

    // record angle
    alpha = odometer.getXYT()[2];
    // make a sound so that we can know the angle is recorded
    Sound.beep();
    stopMotors();
    // rotate out of the wall
    while (medianFilter() < THRESHOLD + MARGIN) {
      clockwise();
    }

    // rotate to the second wall
    while (medianFilter() > THRESHOLD) {
      clockwise();
    }
    Sound.beep();
    beta = odometer.getXYT()[2];

    stopMotors();

    // calculate angle of rotation by using the formula given in the tutorial
    theta = calculateTheta(alpha, beta);

    if (Math.abs(alpha - beta) <= 100) { // when the robot is facing the wall, it will turn around
      angleToTurn = theta + odometer.getXYT()[2] + RISING_ANGLE;
    } else {// when the robot is not facing the wall
      angleToTurn = theta + odometer.getXYT()[2];

    }
    // rotate robot to the theta = 0.0
    // introduce a fix error correction
    turnBy(angleToTurn);

    // set odometer to x, y,theta = 0
    odometer.setXYT(0.0, 0.0, 0.0);

  }

  /**
   * A method to localize position using the rising edge
   * 
   * @param alpha
   * @param beta
   * @param angleToTurn
   */
  public static void risingEdge() {
    // Rotate to the wall
    while (medianFilter() > THRESHOLD) {
      counterclockwise();
    }
    // Rotate until it sees the open space
    while (medianFilter() < THRESHOLD + MARGIN) {
      counterclockwise();
    }

    alpha = odometer.getXYT()[2];
    // make a sound so that we can know the angle is recorded
    Sound.beep();

    // rotate the other way until it sees the wall
    while (medianFilter() > THRESHOLD) {
      clockwise();
    }

    // rotate until it sees open space
    while (medianFilter() < THRESHOLD + MARGIN) {
      clockwise();
    }

    beta = odometer.getXYT()[2];
    Sound.beep();
    stopMotors();

    theta = calculateTheta(alpha, beta) + RISING_ANGLE;

    angleToTurn = theta + odometer.getXYT()[2];

    turnBy(angleToTurn);

    odometer.setXYT(0.0, 0.0, 0.0);
  }

  /**
   * This method use a median filter to filter the data collected by the sensor and toss out the
   * invalid sample
   * 
   * This method can help avoid the effect caused by invalid readings.
   * 
   * @param void
   * @return median value
   * 
   */
  private static double medianFilter() {
    double[] filterData = new double[5]; // use an array to store readings
    for (int i = 0; i < 5; i++) { // take 5 readings
      usDistance.fetchSample(usData, 0);
      filterData[i] = usData[0] * 100.0; // put the readings in array and amplify them
    }
    Arrays.sort(filterData); // sort the array
    return filterData[2]; // take median value
  }


  /**
   * This method is used to calculate the angle to be added to the heading reported by the odometer
   * based on the value of alpha and beta
   * 
   * @param alpha
   * @param beta
   * @return theta
   */
  private static double calculateTheta(double alpha, double beta) {// calculate the value of theta
    double theta = 0;
    if (alpha < beta) {
      theta = 45 - (alpha + beta) / 2;

    } else if (alpha > beta) {
      theta = 225 - (alpha + beta) / 2;
    }

    return theta;

  }

}

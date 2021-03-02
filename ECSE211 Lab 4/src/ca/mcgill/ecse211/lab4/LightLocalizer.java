package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import java.util.Arrays;

import static java.lang.Math.*;
import static ca.mcgill.ecse211.lab4.Lab4.*;
import static ca.mcgill.ecse211.lab4.Navigation.*;

/**
 * This class implements light localization to complete the localization process for the robot. It
 * detects a given amount of black lines on the test surface. Then, it uses the angles at which the
 * robot detected those lines to orient itself towards the origin.
 * 
 * @author Cristian Ciungu
 * @author Hao Shu
 * @version 12-02-2019
 */
public class LightLocalizer {

  // robot constants
  /**
   * The length from the head of the robot to the sensor.
   */
  private final static double SENSOR_LENGTH = 15.00;

  private static double lightReceived;

  private static SampleProvider color = COLOR_SENSOR.getMode("Red");
  static float[] colorData = new float[COLOR_SENSOR.sampleSize()];
  static int nBlackLinesSeen = 0; // a counter is used to record the number of black lines detected
  
  /**
   * Records the angle of the robot at each black line
   */
  static double[] lineNum = new double[5];

  private static double alpha; // the turning angle on x-axis and y-axis
  private static double beta;
  private static double deltaAlpha;// the change in distance in x-axis and y-axis
  private static double deltaBeta;
  private static double angleError = 15;// offset for the turning angle

  /**
   * This method localizes the robot using the light sensor to precisely move to the right location
   * 
   * @param lightReceived
   * @param nBlackLinesSeen
   */
  public static void localize() {
    setSpeed(ROTATION_SPEED);

    // Rotate and scan four lines, record angle respectively
    // increase the counter variable by 1 for each time detection of line

    while (nBlackLinesSeen < 5) {
      counterclockwise();

      lightReceived = medianFilter();// get data from the median filter

      // lines detected,20 means low density of reflection
      if (lightReceived < 20) {
        lineNum[nBlackLinesSeen] = odometer.getXYT()[2];
        Sound.beep();// give a sound so that we can know that the lines are detected
        nBlackLinesSeen++;
      }
    }
    // when the robot scans a same line for twice, the robot will stop
    stopMotors();

    // Get our location from origin using the calculated angles
    alpha = lineNum[3] - lineNum[1];// angle in x-axis
    beta = lineNum[2] - lineNum[0];// angle in y-axis

    // use the formula given in tutorial to calculate the position in x and y axis
    deltaAlpha = -SENSOR_LENGTH * cos(toRadians(alpha / 2));
    deltaBeta = -SENSOR_LENGTH * cos(toRadians(beta / 2));

    // travel to origin from current calculated position
    odometer.setXYT(deltaBeta, deltaAlpha, odometer.getXYT()[2] + angleError);
    travelTo(0.00, 0.00);

    setSpeed(ROTATION_SPEED);

    // if we are not facing 0.0 then turn ourselves with a certain angle to face 0.0
    if (odometer.getXYT()[2] <= 355 && odometer.getXYT()[2] >= 5.0) {
      Sound.beep();
      turnBy(odometer.getXYT()[2]);
    }

    stopMotors();
    odometer.setXYT(0.00, 0.00, 0.00);// set the value of x, y, and theta to zero.
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
      color.fetchSample(colorData, 0);
      filterData[i] = colorData[0] * 100.0; // put the readings in array and amplify them
    }
    Arrays.sort(filterData); // sort the array
    return filterData[2]; // take median value
  }

}

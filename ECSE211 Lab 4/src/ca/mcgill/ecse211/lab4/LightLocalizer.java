
package ca.mcgill.ecse211.lab4;


import lejos.hardware.Sound;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import java.util.Arrays;

public class LightLocalizer implements Runnable {

  // robot constants
  public final static int ROTATION_SPEED = 100;
  private final static double SENSOR_LENGTH = 15.00;
  private final static int FORWARD_SPEED = 80;
  private final static int ROTATE_SPEED = 100;
  
  private Odometer odometer;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  // Instantiate the EV3 ColorSensor
  private static final EV3ColorSensor ColorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S2"));
  private double light_received;

  private SampleProvider Color;
  float[] colorData;
  double[] linecounter;

  private double LastX,LastY,LastT;
  private double alpha, beta;
  private double delta_alpha, delta_beta,deltaX,deltaY;
  private double angleError =30;

  public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {

    this.odometer = odometer;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    this.Color = ColorSensor.getMode("Red");
    colorData = new float[ColorSensor.sampleSize()];
    linecounter = new double[5];
    }

    /**
     * This method localizes the robot using the light sensor to precisely move to
     * the right location
     * @param light_received
     * @param counter
     */
  public void run() {

    int counter = 0;
      leftMotor.setSpeed(ROTATION_SPEED);
      rightMotor.setSpeed(ROTATION_SPEED);

      //Rotate and scan four lines, record angle respectively
      //increase the counter variable for 1, each time detection of line
      while (counter < 5) {

        leftMotor.backward();
        rightMotor.forward();

        light_received = medianFilter();

        //detect lines, 0.37 determine by test and represents low density of reflection
        if (light_received < 30) {
          linecounter[counter] = odometer.getXYT()[2];
          Sound.beep();
          counter++;
            }
        }

        leftMotor.stop(true);
        rightMotor.stop();

     

      // Get our location from origin using the calculated angles
     
      alpha = linecounter[3] - linecounter[1];// angle in x-axis
      beta = linecounter[2] - linecounter[0];// angle in y-axis
      //use the formula given in tutorial to calculate the position in x and y axis
      delta_alpha = -SENSOR_LENGTH * Math.cos(Math.toRadians(alpha / 2));
      delta_beta = -SENSOR_LENGTH * Math.cos(Math.toRadians(beta / 2));
      //angle correction from tutorial
      deltaY = 3*Math.PI/4 - linecounter[0] + (beta/ 2.0);

      // travel to origin from current calculated position
      odometer.setXYT(delta_beta, delta_alpha, odometer.getXYT()[2]+angleError);
      this.travelTo(0.00, 0.00);

      leftMotor.setSpeed(ROTATION_SPEED );
      rightMotor.setSpeed(ROTATION_SPEED);

      // if we are not facing 0.0 then turn ourselves so that we are
      if (odometer.getXYT()[2] <= 355 && odometer.getXYT()[2] >= 5.0) {
        Sound.beep();
        leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -odometer.getXYT()[2]+ deltaY), true);
        rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -odometer.getXYT()[2] + deltaY), false);
        }

        leftMotor.stop(true);
        rightMotor.stop();
        odometer.setTheta(0.00);

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
   * This method allows the conversion of an 
   * angle to the total distance that the robot
   * needs to cover. 
   * 
   * @param radius
   * @param wideltaTh
   * @param angle
   * @return (int)angle converted 
   * 
   */
  
  private static int convertAngle(double radius, double wideltaTh, double angle) {
      return convertDistance(radius, Math.PI * wideltaTh * angle / 360.0);
  }

  /**
   * This method use a median filter to filter the data collected by the sensor.
   * This method can help avoid the effect caused by unusual readings.
   *  
   * 
   * @param void 
   * @return median value
   * 
   */
  
  private double medianFilter() {
    double[] filterData = new double[5]; //use an array to store readings
    for (int i = 0; i < 5; i++) { // take 5 readings
      Color.fetchSample(colorData, 0);
      filterData[i] = colorData[0] * 100.0; // put the readings in array and amplify them
    }
    Arrays.sort(filterData); // sort the array
    return filterData[2]; // take median value
  }

  /**
   * This method make the robot go to the point (x,y)
   * @param x
   * @param y
   * @param currentx
   * @param currenty
   * @param currentTheta
   * @param deltax
   * @param deltay
   * @param mTheta
   */
  public void travelTo(double x, double y) {

    LastX = odometer.getXYT()[0];// The initial value of x,y, and theta
    LastY = odometer.getXYT()[1];
    LastT = odometer.getXYT()[2] * Math.PI / 180;

    double deltaX = x - LastX;// Calculate the change in the values of x, y,and theta.
    double deltaY = y - LastY;
    double deltaT = Math.atan2(deltaX, deltaY) - LastT;
    double Distance = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);// calculate the distance need to
                                                                // travel
    Distance = Math.sqrt(Distance);
    turnTo(deltaT);// turn with the degree theta

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, Distance), true);
    rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, Distance), false);

    rightMotor.stop(true);
    leftMotor.stop(true);// stop the robot
  }
  
  /**
   * This method causes the robot to turn 
   * (on point) to the absolute heading theta. 
   * This method should turn a MINIMAL angle 
   * to its target.
   * 
   * @param theta
   * @return void
   * 
   */
  public void turnTo(double theta) {

    theta = theta * 180.0 / Math.PI;// convert the angle from radian to degree

    leftMotor.setSpeed(ROTATE_SPEED);// Set speed to move in a straight line
    rightMotor.setSpeed(ROTATE_SPEED);

    if (theta <= -180) {// when the angle is larger than 180 degree
      theta += 180 * 2;
    } else if (theta > 180) {// when the angle is smaller than 180 degree
      theta -= 180 * 2;
    }

    if (theta < 0) {// if the angle is negative, the robot will turn left
      leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -theta), true);
      rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -theta), false);

    } else {// if the angle is positive, the robot will turn right

      leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
      rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
    }

  }
    
}
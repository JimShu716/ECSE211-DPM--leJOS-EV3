package ca.mcgill.ecse211.lab4;


import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import java.util.Arrays;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;

public class UltrasonicLocalizer {

  private EV3LargeRegulatedMotor leftMotor, rightMotor;

  private Odometer odometer;
  private SampleProvider usDistance;
  private float[] usData;

  private double distance;// the distance from the wall
  private double alpha, beta, theta;

  public static final double THRESHOLD = 40.00;// the value of d
  public static final double MARGIN = 10.00; // the value of k
  public static final int FORWARD_SPEED = 100;

  /**
   * UltrasonicLocalizer constructor
   * 
   * @param leftMotor
   * @param rightMotor
   * @param odometer
   * @throws OdometerExceptions
   */
  public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      Odometer odometer) throws OdometerExceptions {

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.odometer = odometer;


    SensorModes ultrasonicSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
    this.usDistance = ultrasonicSensor.getMode("Distance");
    this.usData = new float[usDistance.sampleSize()];
  }


  public void fallingedge() {


    distance = medianFilter();

    // rotates counterclockwise until distance from wall is bigger than THRESHOLD
    while (this.distance < MARGIN) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.backward();
      rightMotor.forward();
    }


    // rotates counterclockwise until distance from wall is smaller than 40
    while (this.distance > THRESHOLD) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.backward();
      rightMotor.forward();

    }

    alpha = odometer.getXYT()[2]; // record the value of the angle
    Sound.beep(); // make a sound after recording the angle

    // rotates clockwise until distance is greater than THRESHOLD
    while (this.distance < MARGIN) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward();
      rightMotor.backward();
    }

    // rotates clockwise until distance small than 40
    while (this.distance > THRESHOLD) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);

    }

    beta = odometer.getXYT()[2]; // record angle reading
    Sound.beep();

    leftMotor.stop();
    rightMotor.stop();

    theta = calculateTheta(alpha, beta);

    // turns to adjust position; results in robot positioned at
    // angle = 0 degrees
    leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
    rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
    odometer.setTheta(0);// reset the odometer
    leftMotor.stop();
    rightMotor.stop();
  }

  /**
   * rising edge method called by run method positions robot at angle of 0 degrees rising edge
   * method is called when the robot is originally facing a wall
   */
  public void risingEdge() {
    distance = medianFilter();


    // rotates counterclowise until distance smaller than 30
    while (this.distance > MARGIN) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.backward();
      rightMotor.forward();
    }


    // rotates counterclowise until distance greater than 40
    while (this.distance < THRESHOLD) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.backward();
      rightMotor.forward();
    }
    leftMotor.stop();
    rightMotor.stop();

    alpha = odometer.getXYT()[2];// record the first angle
    Sound.beep();

    // rotates clockwise until distance smaller than 30
    while (this.distance > MARGIN) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward();
      rightMotor.backward();

    }
    leftMotor.stop();
    rightMotor.stop();

    // rotates clockwise until distance greater than 40
    while (this.distance < THRESHOLD) {
      leftMotor.setSpeed(FORWARD_SPEED);
      rightMotor.setSpeed(FORWARD_SPEED);
      leftMotor.forward();
      rightMotor.backward();

    }
    leftMotor.stop();
    rightMotor.stop();

    beta = odometer.getXYT()[2]; // record the second angle
    Sound.beep();

    // turns to adjust position; results in robot positioned at angle = 0 degrees
    
    theta = calculateTheta(alpha, beta);
    leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
    rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta ), false);
    odometer.setTheta(0);//reset the odometer
    leftMotor.stop();
    rightMotor.stop();
  }


  /**
   * This method use a median filter to filter the data collected by the sensor and toss out the in
   * valid sample
   * 
   * This method can help avoid the effect caused by invalid readings.
   * 
   * @param void
   * @return median value
   * 
   */
  private double medianFilter() {
    double[] filterData = new double[5]; // use an array to store readings
    for (int i = 0; i < 5; i++) { // take 5 readings
      usDistance.fetchSample(usData, 0);
      filterData[i] = usData[0] * 100.0; // put the readings in array and amplify them
    }
    Arrays.sort(filterData); // sort the array
    return filterData[2]; // take median value
  }



  private double calculateTheta(double alpha, double beta) {// calculate the value of theta

    if (alpha < beta) {
      theta = 45 - (alpha + beta) / 2;

    } else if (alpha > beta) {
      theta = 225 - (alpha + beta) / 2;
    }

    return theta;

  }


  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This method allows the conversion of an angle to the total distance that the robot needs to
   * cover.
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
}

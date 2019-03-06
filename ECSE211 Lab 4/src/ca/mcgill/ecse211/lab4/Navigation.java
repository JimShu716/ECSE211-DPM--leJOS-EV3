package ca.mcgill.ecse211.lab4;

import static ca.mcgill.ecse211.lab4.Lab4.*;
import static java.lang.Math.*;


/**
 * Class for static navigation methods.
 */
public class Navigation {

  /**
   * Turns by specified angle.
   * 
   * @param angleToTurn
   */
  public static void turnBy(double angleToTurn) {
    LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, angleToTurn), true);
    RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, angleToTurn), false);
  }

  /**
   * Rotates clockwise.
   */
  public static void clockwise() {
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.backward();
  }
  
  /**
   * Rotates counterclockwise.
   */
  public static void counterclockwise() {
    LEFT_MOTOR.backward();
    RIGHT_MOTOR.forward();
  }
  
  /**
   * Sets the speed for both motors.
   */
  public static void setSpeed(int speed) {
    LEFT_MOTOR.setSpeed(ROTATION_SPEED);
    RIGHT_MOTOR.setSpeed(ROTATION_SPEED);
  }
  
  /**
   * Stops the motors.
   */
  public static void stopMotors() {
    LEFT_MOTOR.stop(true);
    RIGHT_MOTOR.stop();
  }
  
  /**
   * This method make the robot go to the point (x,y)
   * 
   * @param x
   * @param y
   */
  public static void travelTo(double x, double y) {

    double LastX = odometer.getXYT()[0];// The initial value of x,y, and theta
    double LastY = odometer.getXYT()[1];
    double LastT = odometer.getXYT()[2] * Math.PI / 180;

    double deltaX = x - LastX;// Calculate the change in the values of x, y,and theta.
    double deltaY = y - LastY;
    double deltaT = atan2(deltaX, deltaY) - LastT;
    double distance = pow(deltaX, 2) + pow(deltaY, 2);// calculate the distance need to
                                                                // travel
    distance = sqrt(distance);
    turnTo(deltaT);// turn with the degree theta

    LEFT_MOTOR.setSpeed(FORWARD_SPEED);
    RIGHT_MOTOR.setSpeed(FORWARD_SPEED);

    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, distance), true);
    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, distance), false);

    RIGHT_MOTOR.stop(true);
    LEFT_MOTOR.stop(true);// stop the robot
  }

  
  /**
   * This method causes the robot to turn (on point) to the absolute heading theta. This method
   * should turn a MINIMAL angle to its target.
   * 
   * @param theta
   */
  public static void turnTo(double theta) {

    theta = theta * 180.0 / Math.PI;// convert the angle from radian to degree

    LEFT_MOTOR.setSpeed(ROTATION_SPEED);// Set speed to move in a straight line
    RIGHT_MOTOR.setSpeed(ROTATION_SPEED);

    if (theta <= -180) {// when the angle is larger than 180 degree
      theta += 180 * 2;
    } else if (theta > 180) {// when the angle is smaller than 180 degree
      theta -= 180 * 2;
    }

    if (theta < 0) {// if the angle is negative, the robot will turn left
      LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, -theta), true);
      RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, -theta), false);

    } else {// if the angle is positive, the robot will turn right

      LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
      RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
    }

  }
  
  /**
   * This method implement the conversion of a distance to rotation of each wheel need to cover the
   * distance.
   * 
   * @param radius
   * @param distance
   * @return
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This method implement the conversion of a angle to rotation of each wheel need to cover the
   * distance.
   * 
   * @param radius
   * @param distance
   * @param angle
   * @return
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  
}

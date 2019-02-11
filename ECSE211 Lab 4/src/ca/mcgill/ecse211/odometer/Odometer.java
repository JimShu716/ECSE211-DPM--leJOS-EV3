/**
 * 
 * This class implements the odometer.
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu
 * @version 05-02-2019
 */

package ca.mcgill.ecse211.odometer;


import lejos.hardware.motor.EV3LargeRegulatedMotor;



public class Odometer extends OdometerData implements Runnable {



  private OdometerData odoData;

  private static Odometer odo = null; // Returned as singleton



  // Motors and related variables

  private int leftMotorTachoCount;

  private int rightMotorTachoCount;

  private EV3LargeRegulatedMotor leftMotor;

  private EV3LargeRegulatedMotor rightMotor;



  private final double TRACK;

  private final double WHEEL_RAD;



  private double[] position;



  private double LastTacho_Right = 0.00;

  private double LastTacho_Left = 0.00;



  private double deltaTacho_Right;

  private double deltaTacho_Left;// change in tacho of the right motor and the left motor



  private double distance_Left;

  private double distance_Right;



  private double deltaDistance;

  private double deltaAngle;

  private double T = 0, X = 0, Y = 0;

  private double deltaT, deltaX, deltaY;



  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms



  /**
   * 
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * 
   * cannot be accessed externally.
   * 
   * 
   * 
   * @param leftMotor
   * 
   * @param rightMotor
   * 
   * @throws OdometerExceptions
   * 
   */

  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,

      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {

    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z

    // manipulation methods

    this.leftMotor = leftMotor;

    this.rightMotor = rightMotor;



    // Reset the values of x, y and z to 0

    odoData.setXYT(0, 0, 0);



    this.leftMotorTachoCount = 0;

    this.rightMotorTachoCount = 0;



    this.TRACK = TRACK;

    this.WHEEL_RAD = WHEEL_RAD;



  }



  /**
   * 
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * 
   * 
   * @param leftMotor
   * 
   * @param rightMotor
   * 
   * @return new or existing Odometer Object
   * 
   * @throws OdometerExceptions
   * 
   */

  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,

      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)

      throws OdometerExceptions {

    if (odo != null) { // Return existing object

      return odo;

    } else { // create object and return it

      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

      return odo;

    }

  }



  /**
   * 
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * 
   * odometer object has been created
   * 
   * 
   * 
   * @return error if no previous odometer exists
   * 
   */

  public synchronized static Odometer getOdometer() throws OdometerExceptions {



    if (odo == null) {

      throw new OdometerExceptions("No previous Odometer exits.");



    }

    return odo;

  }



  /**
   * 
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * 
   * OdometerData class to implement the odometer.
   * 
   */

  // run method (required for Thread)

  public void run() {

    long updateStart, updateEnd;



    while (true) {

      updateStart = System.currentTimeMillis();



      leftMotorTachoCount = leftMotor.getTachoCount();

      rightMotorTachoCount = rightMotor.getTachoCount();



      deltaTacho_Left = leftMotorTachoCount - LastTacho_Left;// calculate the change in tacho

      deltaTacho_Right = rightMotorTachoCount - LastTacho_Right;



      distance_Left = Math.PI * WHEEL_RAD * deltaTacho_Left / 180;

      distance_Right = Math.PI * WHEEL_RAD * deltaTacho_Right / 180;// calculate the distance
                                                                    // travelled by the motors



      deltaDistance = (distance_Left + distance_Right) / 2.00; // calculate the change in distance

      deltaAngle = (distance_Left - distance_Right) / TRACK;



      LastTacho_Left = leftMotorTachoCount; // update the tachocount

      LastTacho_Right = rightMotorTachoCount;



      T += deltaAngle; // calculate the current value of Theta



      deltaT = deltaAngle * 180 / Math.PI;

      deltaX = deltaDistance * Math.sin(T); // calculate the change in the value of x and y

      deltaY = deltaDistance * Math.cos(T);



      X += deltaX;// calculate current value of X and Y

      Y += deltaY;



      odo.update(deltaX, deltaY, deltaT);// update the change in values of X,Y and Theta



      // this ensures that the odometer only runs once every period

      updateEnd = System.currentTimeMillis();

      if (updateEnd - updateStart < ODOMETER_PERIOD) {

        try {

          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));

        } catch (InterruptedException e) {

          // there is nothing to be done

        }

      }

    }

  }



}

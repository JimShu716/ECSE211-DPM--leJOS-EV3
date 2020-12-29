/**
 * This class stores and provides a thread-safe access to the odometer data.
 *
 * @author1 Cristian Ciungu
 * @author2 Hao SHu
 * @version 12-02-2019
 *
 */

// import package

package ca.mcgill.ecse211.odometer;

// import utility classes for multi-threading

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;



public class OdometerData {

  // Position parameters

  private volatile double x; // x-axis position
  private volatile double y; // y-axis position
  private volatile double theta; // Head angle

  // Class control variables

  private volatile static int numberOfInstances = 0; // Number of OdometerData objects instantiated
  private static final int MAX_INSTANCES = 1; // Maximum number of OdometerData instances

  // Thread control tools

  private static Lock lock = new ReentrantLock(true); // Fair lock for concurrent writing
  private volatile boolean isReseting = false; // Indicates if a thread is trying to reset any
                                               // position parameters
  private Condition doneReseting = lock.newCondition(); // Let other threads know that a reset
                                                        // operation is over

  private static OdometerData odoData = null; // Instantiates an OdometerData constructor

  /**
   * 
   * Default constructor. The constructor is private. A factory is used instead, such that only one
   * instance of this class is ever created.
   * 
   */

  protected OdometerData() {

    this.x = 0;
    this.y = 0;
    this.theta = 0;

  }

  /**
   * 
   * OdometerData factory. It returns an OdometerData instance and makes sure that only one instance
   * is ever created. If the user tries to instantiate multiple objects, the method throws a
   * MultipleOdometerDataException.
   * 
   * @return An OdometerData object
   * @throws OdometerExceptions
   * 
   */

  public synchronized static OdometerData getOdometerData() throws OdometerExceptions {

    if (odoData != null) {

      return odoData; // Return existing object

    } else if (numberOfInstances < MAX_INSTANCES) { // create object and return it

      odoData = new OdometerData();
      numberOfInstances += 1;
      return odoData;

    } else {

      throw new OdometerExceptions("Only one intance of the Odometer can be created.");

    }
  } // end getOdometerData()

  /**
   * 
   * This method returns the Odometer data. It writes the current position and orientation of the
   * robot onto the odoData array. odoData[0] = x, odoData[1] = y; odoData[2] = theta.
   * 
   * @param void
   * @return odometer position (x,y,theta)
   * 
   */

  public double[] getXYT() {

    double[] position = new double[3]; // Instantiates odometer position (x,y,theta)

    lock.lock(); // Locks the Odometer thread to get odometer values

    try {

      while (isReseting) { // If a reset operation is being executed, wait until it is over.

        doneReseting.await(); // Using await() is lighter on the CPU than busy-waiting.

      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;

    } catch (InterruptedException e) {

      e.printStackTrace(); // Print exception to screen

    } finally {

      lock.unlock(); // Resume the Odometer thread

    }

    return position;

  } // end getXYT()

  /**
   * 
   * This method calls dx, dy and dtheta to the current values of x, y and theta, respectively. This
   * is useful when one implements odometry.
   * 
   * @param dx
   * @param dy
   * @param dtheta
   * @return void
   * 
   */

  public void update(double dx, double dy, double dtheta) {

    lock.lock(); // Locks the Odometer thread to update odometer values

    isReseting = true; // Initiates reseting

    try {

      x += dx; // Update x values
      y += dy; // Update y values
      theta = (theta + (360 + dtheta) % 360) % 360; // Keeps the angle updates within 360 degrees
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are done reseting

    } finally {

      lock.unlock(); // Resume the Odometer thread

    }

  } // end update()

  /**
   * 
   * This method overrides the values of x, y and theta. It is useful for odometry correction.
   * 
   * @param x
   * @param y
   * @param theta
   * @return void
   * 
   */

  public void setXYT(double x, double y, double theta) {

    lock.lock(); // Locks the Odometer threads to apply odometer correction

    isReseting = true; // Initiates reseting

    try {

      this.x = x; // Update x value
      this.y = y; // Update y value
      this.theta = theta; // Update theta value
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are done reseting

    } finally {

      lock.unlock(); // Resume the Odometer thread

    }

  } // end setXYT()



  /**
   * 
   * This method overrides x values on the odometer. This is useful when only the x value has to be
   * corrected on the odometer.
   * 
   * @param x
   * @return void
   * 
   */

  public void setX(double x) {

    lock.lock(); // Locks the Odometer thread to apply corrections

    isReseting = true; // Instantiates reseting

    try {

      this.x = x; // Update x value
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are done reseting

    } finally {

      lock.unlock(); // Resume the Odometer thread

    }

  } // end setX()


  /**
   * 
   * This method overrides y values on the odometer. This is useful when only the y value has to be
   * corrected on the odometer.
   * 
   * @param y the value of y
   * @return void
   * 
   */

  public void setY(double y) {

    lock.lock(); // Locks the Odometer thread to apply corrections
    isReseting = true; // Instantiates reseting

    try {

      this.y = y; // Update y values
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are done reseting

    } finally {

      lock.unlock(); // Resume the Odometer thread

    }

  } // end setY()

  /**
   * 
   * This method overrides theta values on the odometer. This is useful when only the theta value
   * has to be corrected on the odometer.
   * 
   * @param theta
   * @return void
   * 
   */

  public void setTheta(double theta) {

    lock.lock(); // Locks the Odometer thread to apply corrections
    isReseting = true; // Instantiates reseting

    try {

      this.theta = theta; // Update theta values
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are done reseting

    } finally {

      lock.unlock(); // Resume Odometer thread

    }

  } // end setTheta

} // end OdometerData

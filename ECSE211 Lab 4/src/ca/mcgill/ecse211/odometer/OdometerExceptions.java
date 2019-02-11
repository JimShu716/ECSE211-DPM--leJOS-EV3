/**
 * This class extends Exception. It is used to handle errors when it comes to the implementation of
 * the light sensor.
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu
 * @version 05-02-2019
 *
 */

// package

package ca.mcgill.ecse211.odometer;

public class OdometerExceptions extends Exception {

  public OdometerExceptions(String Error) {

    super(Error);

  }
} // end OdometerExceptions

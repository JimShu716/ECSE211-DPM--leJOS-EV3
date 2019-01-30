/**
 * This class is used to handle errors when it comes to the implementation 
 * of the light sensor.
 * 
 *@author1 Cristian Ciungu
 *@author2 Hao Shu
 *@version 2019-01-29
 *
 *
 */


// package

package ca.mcgill.ecse211.odometer;

public class OdometerExceptions extends Exception {

	public OdometerExceptions(String Error) {

		super(Error);

	}
} // end OdometerExceptions
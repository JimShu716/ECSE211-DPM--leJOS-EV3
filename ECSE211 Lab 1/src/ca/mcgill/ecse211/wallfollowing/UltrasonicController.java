/**
 * This is an interface that implements the processUSData and readUSDistance methods.
 * It allows the US sensor readings to be stored and to be filtered according to defined
 * range of values. 
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu 
 * @version 2019-01-22
 * 
 */

// declare package

package ca.mcgill.ecse211.wallfollowing;



public interface UltrasonicController {



  public void processUSData(int distance);



  public int readUSDistance();

} // end interface 
/**

 * Control of the wall follower is applied periodically by the UltrasonicPoller thread. Distance sampling
 * is executed using an infinite while loop . We estimate that the us.fetchSample and cont.processUSData
 * methods operate in about 20mS. We also set the thread to sleep for 5 mS at the end of each loop. 
 * Therefore, one loop cycle is approximately 25 ms, which generates a sampling rate of 1/25 ms,
 * or about 40 Hz. 
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu 
 * @version 2019-01-22
 * 
 */

// declare package

package ca.mcgill.ecse211.lab3;

// imports 

import lejos.robotics.SampleProvider;



public class UltrasonicPoller extends Thread {

	// parameters

	private SampleProvider us;

	private UltrasonicController cont;

	private float[] usData;



	// define constructor for the ultrasonic sensor 

	public UltrasonicPoller(SampleProvider us, float[] usData, UltrasonicController cont) {

		this.us = us;

		this.cont = cont;

		this.usData = usData;

	} 


	public void run() {

		int distance;

		// continuous distance sampling 
		
		while (true) {

			us.fetchSample(usData, 0); // acquire data

			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int

			cont.processUSData(distance); // execute with respect to the distance 

			
			try {

				Thread.sleep(5); // sleep for 5 ms

			} catch (Exception e) {

			} 

		}

	} // end run



} // end UltrasonicPoller 
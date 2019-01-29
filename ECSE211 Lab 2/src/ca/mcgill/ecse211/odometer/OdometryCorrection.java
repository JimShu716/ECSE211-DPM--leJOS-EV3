/*

 * OdometryCorrection.java

 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.ev3.LocalEV3;

import lejos.hardware.sensor.EV3ColorSensor;

import lejos.hardware.sensor.SensorModes;

import lejos.robotics.SampleProvider;

import lejos.hardware.Sound;

import java.util.*; 


public class OdometryCorrection implements Runnable {

	private static final long CORRECTION_PERIOD = 10;

	private Odometer odometer;

	private static final EV3ColorSensor ColorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));// set the color
																										// sensor

	private static final double TILE_Size = 30.48;

	private double deltaT = 0.00;// the correction of theta

	private double deltaX = 0.00;// the correction of x

	private double deltaY = 0.00;// the correction of y

	private int xLine_num = 0; // number of the black lines detected when walking in x axis

	private int yLine_num = 0;// number of the black lines detected when walking in y axis

	private float[] sampleData;

	private SampleProvider Color;
	
	

	/**
	 * 
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to
	 * 
	 * ensure thread safety.
	 * 
	 * 
	 * 
	 * @throws OdometerExceptions
	 * 
	 */

	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

		this.Color = ColorSensor.getMode("Red");// set the color

		this.sampleData = new float[ColorSensor.sampleSize()]; // collect sample datas

	}

	/**
	 * 
	 * Here is where the odometer correction code should be run.
	 * 
	 * 
	 * 
	 * @throws OdometerExceptions
	 * 
	 */

	// run method (required for Thread)
	
	
	
	private float filter() {// use a median filter to filter the sensor data
		
		float[] filterData = {};//use an array to store data
		  
		for (int i =0;i<7;i++) {
	
			Color.fetchSample(sampleData, 0);//obtain the data
		
			
		
		filterData[i] = sampleData[i];
		 Arrays.sort(filterData); // place the data in order
		 
			
			}
	
		if (sampleData[0]>filterData[3]) {//use the median to set an upper limit
			
			sampleData[0] = filterData[3];
		}else {
			
			 sampleData[0]= sampleData[0];
			
			}
		

		return sampleData[0];
		
	}

	public void run() {

		long correctionStart, correctionEnd;

		float light_received;

		while (true) {

			correctionStart = System.currentTimeMillis();

			Color.fetchSample(sampleData, 0);

			light_received = filter(); // the filter is used for the more accurate sensor data

			double[] xytData = odometer.getXYT(); // using get method from OdometerData

			if (light_received < 0.24) {

				Sound.beep();// make a sound when detect the lines

				if (xytData[2] < 10 || xytData[2] > 350) {// when there is no turning, the robot is walking in y-axis
                                                          // should not use &&
					deltaY = xytData[1] - (TILE_Size * yLine_num);

					odometer.setY(xytData[1] - deltaY);

					yLine_num++;

				}

				else if (xytData[2] > 80 && xytData[2] < 100) {// after making the first right angle turn, the robot
																// begins to travel in x-axis

					deltaX = xytData[0] - (TILE_Size * xLine_num);

					odometer.setX(xytData[0] - deltaX);

					xLine_num++;
				}

				else if (xytData[2] > 170 && xytData[2] < 190) {// after making the 2nd turn, the robot begin to travel
																// in the negative direction of y-axis

					yLine_num--;// reduce the count before correction
					deltaY = xytData[1] - (TILE_Size * yLine_num);
					odometer.setY(xytData[1]-deltaY );

				}

				else if (xytData[2] > 260 && xytData[2] < 300) {// after making the 3rd turn, the robot begin to travel
																// int the negative direction of x-axis

					xLine_num--;

					deltaX = xytData[0] - (TILE_Size * xLine_num);
					odometer.setX(xytData[0]- deltaX );
					odometer.setY(xytData[1]-1.5);//use a offset to correct y
				

				}

			}

			// this ensure the odometry correction occurs only once every period

			correctionEnd = System.currentTimeMillis();

			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {

				try {

					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));

				} catch (InterruptedException e) {

					// there is nothing to be done here

				}

			}

		}

	}

}
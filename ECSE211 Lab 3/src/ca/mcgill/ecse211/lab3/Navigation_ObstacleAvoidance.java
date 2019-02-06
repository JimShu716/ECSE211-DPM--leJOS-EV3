/**
 * This class is an extension of Thread, and it 
 * allows the robot to avoid obstacles on its 
 * path. Essentially, it uses the same methods 
 * as the Navigation class, but it also implements 
 * the medianFilter(): the method returns the median 
 * value of a given number of readings from the 
 * ultrasonic sensor, which is then used to compute 
 * the distance between the robot and the obstacle.
 * If the distance computed is too small, i.e. the 
 * robot is too close to the obstacle, then obstacle 
 * avoidance is initiated. The mechanism doesn't rely
 * on the Bang-Bang controller from Lab1; it rather 
 * uses a series of hard-coded actions to avoid a 
 * wooden block with width 10 cm and length 25  cm.
 * 
 * Additionally, a constructor was set up for this 
 * class so that we can implement navigation with 
 * obstacle avoidance into our main class, Lab3. 
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu
 * 
 * @version 05-02-2019
 * 
 */

package ca.mcgill.ecse211.lab3; 

import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;

import java.util.Arrays;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;


public class Navigation_ObstacleAvoidance extends Thread {

	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usDistance ;
	
	private static Odometer odometer;
	private OdometerData odoData;
	
	public static final int FORWARD_SPEED = 175;
	private static final int ROTATE_SPEED = 100;
	
	double LastX,LastY,LastT;
	double deltaX, deltaY, deltaT;

	int counter = 0;//use a counter to calculate the position the robot should be after avoiding the obstacle
	
	private double[][] path = new double[][]{	//Travel to different coordinates in the maps

		
//		  map 1:
		  { 0*Navigation.TILE_SIZE, 2*Navigation.TILE_SIZE} ,
		  { 1*Navigation.TILE_SIZE, 1*Navigation.TILE_SIZE }, 
		  { 2*Navigation.TILE_SIZE, 2*Navigation.TILE_SIZE }, 
		  { 2*Navigation.TILE_SIZE, 1*Navigation.TILE_SIZE },
		  { 1*Navigation.TILE_SIZE, 0*Navigation.TILE_SIZE }
//		    
//		    
//		  map 2: 
//		  { 1*Navigation.TILE_SIZE, 1*Navigation.TILE_SIZE }, 
//		  { 0*Navigation.TILE_SIZE, 2*Navigation.TILE_SIZE }, 
//		  { 2*Navigation.TILE_SIZE, 2*Navigation.TILE_SIZE }, 
//		  { 2*Navigation.TILE_SIZE, 1*Navigation.TILE_SIZE }, 
//		  { 1*Navigation.TILE_SIZE, 0*Navigation.TILE_SIZE }
//		  
//		  
//		  map 3: 
//		  { 1*Navigation.TILE_SIZE, 0*Navigation.TILE_SIZE }, 
//		  { 2*Navigation.TILE_SIZE, 1*Navigation.TILE_SIZE }, 
//		  { 2*Navigation.TILE_SIZE, 2*Navigation.TILE_SIZE }, 
//		  { 0*Navigation.TILE_SIZE, 2*Navigation.TILE_SIZE }, 
//		  { 1*Navigation.TILE_SIZE, 1*Navigation.TILE_SIZE }
//		  
//		  map 4: 
//		  { 0*Navigation.TILE_SIZE, 1*Navigation.TILE_SIZE }, 
//		  { 1*Navigation.TILE_SIZE, 2*Navigation.TILE_SIZE }, 
//		  { 1*Navigation.TILE_SIZE, 0*Navigation.TILE_SIZE }, 
//		  { 2*Navigation.TILE_SIZE, 1*Navigation.TILE_SIZE },
//		  { 2*Navigation.TILE_SIZE, 2*Navigation.TILE_SIZE }
//    
	};
	

    //class constructor
	public Navigation_ObstacleAvoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) throws OdometerExceptions {
		this.odometer = Odometer.getOdometer();
		Navigation_ObstacleAvoidance.leftMotor = leftMotor;
		Navigation_ObstacleAvoidance.rightMotor = rightMotor;
		odoData = OdometerData.getOdometerData();
		
		//setup  ultrasonic sensor
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
		usDistance = usSensor.getMode("Distance"); 
		this.usData = new float[usDistance.sampleSize()]; 		
			
		}

		
	public void run() {
		
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();//stop the robot after arriving at one point of the map
		}
		try {
			Thread.sleep(1500);
		} catch (InterruptedException e) {
		}
		
		
		
		while(counter < path.length) {//use the counter to obtain the coordinates and run the robot use travelTo method
			travelTo(path[counter][0], path[counter][1]);
			counter++;
		}
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
		      usDistance.fetchSample(usData, 0); 
		      filterData[i] = usData[0] * 100.0; // put the readings in array and amplify them
		    }
		    Arrays.sort(filterData); // sort the array
		    return filterData[2]; // take median value
		  }
	

	/**
	 * This method causes the robot to travel to the 
	 * absolute field location (x, y), specified in 
	 * tile points. This method should continuously 
	 * call turnTo(double theta) and then set the 
	 * motor speed to forward(straight). This will 
	 * make sure that your heading is updated until 
	 * you reach your exact goal. This method will 
	 * poll the odometer for information. This method 
	 * also avoids if there is an obstacle on the way
	 * by going around it in a square like manner.
	 * 
	 * @param x
	 * @param y
	 * @return void 
	 * 
	 */
	
	void travelTo(double x, double y) {
		//This  method  causes  the  robot  to  travel  to  the  absolute  field  location  (x,  y)
		
				LastX = odometer.getXYT()[0];//The initial value of x,y, and theta
				LastY = odometer.getXYT()[1];
				LastT = odometer.getXYT()[2]* Math.PI/180;
				
				deltaX = x - LastX;// Calculate the change in the values of x, y,and theta.
				deltaY = y - LastY;
				deltaT = Math.atan2(deltaX, deltaY) - LastT;
			    double Distance = Math.pow(deltaX, 2) + Math.pow(deltaY, 2);//calculate the distance need to travel
			    Distance = Math.sqrt(Distance);
			    
			      if(deltaY>=0) {
						deltaT=Math.atan(deltaX/deltaY);
					}
					else if(deltaY<=0&&deltaX>=0) {
						deltaT=Math.atan(deltaX/deltaY)+Math.PI;
					}
					else {
						deltaT=Math.atan(deltaX/deltaY)-Math.PI;
					}
			      double  ThetaToTurn= (deltaT-LastT); // calculate the angle needs to turn
			      
			      turnTo(ThetaToTurn*180/Math.PI); 
			      
				leftMotor.setSpeed(FORWARD_SPEED);
				
				rightMotor.setSpeed(FORWARD_SPEED);
				
				leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, Distance), true);
				rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, Distance), true);

	
		while(isNavigating()) {
			
			
			double distance = medianFilter();//obtain the data from the filter
			double distError = distance - Lab3.bandCenter;//calculate distance error
			
			
			if(distError<-15) {// when gets close to the obstacle
			
				 if(LastX < 2.3*Navigation.TILE_SIZE&& LastX > 1.3*Navigation.TILE_SIZE 
						&& LastY < 2.3*Navigation.TILE_SIZE&&LastY > 1.3*Navigation.TILE_SIZE){// in these cases the robot needs to turn left or it will fall out of the board.So turn left
					leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);  
					rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
					leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), true);//travel a distance after turning
					rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), false);
					leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);//turn again
					rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
					leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 40), true);
					rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 40), false);
				}
				//else it will turn right 
				else {
				leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);   
				rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
				leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), true);//travel a distance after turning
				rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 30), false);
				leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);//turn again
				rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
				leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 35), true);
				rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 35), false);
				}
				counter--;//correct the coordinates
			}
			}
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
	private void turnTo(double theta) {
		
		
		leftMotor.setSpeed(ROTATE_SPEED);// Set speed to move in a straight line
		rightMotor.setSpeed(ROTATE_SPEED);
		
		if (theta <= -180) {//when the angle is larger than 180 degree
			theta += 180* 2;
		} else if (theta > 180) {//when the angle is smaller than 180 degree
			theta -= 180 * 2;
		}
		
		if (theta < 0) {// if the angle is negative, the robot will turn left
			leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -theta), true);
			rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -theta), false);

		} else {//if the angle is positive, the robot will turn right
			
			leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, theta ), true);
			rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, theta), false);
		}
		
	}
	/**
	 *This method returns true if another thread has
	 * called travelTo() or turnTo() and the method 
	 * has yet to return; false otherwise.
	 * 
	 * @param void
	 * @return true/false
	 * 
	 */
		
	boolean isNavigating() {
		if((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else 
			return false;
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
}
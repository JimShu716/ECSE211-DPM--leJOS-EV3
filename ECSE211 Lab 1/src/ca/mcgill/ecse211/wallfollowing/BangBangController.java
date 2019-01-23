/**
 * This class implements UltrasonicController, and also provides a 
 * constructor for a Bang Bang controller. Bang Bang control allows 
 * the autonomous tribot to execute drastic path corrections. The 
 * magnitude of these corrections were determined after several trial 
 * runs . The lack of accuracy of these corrections are compensated by 
 * setting lower running speeds for the tribot's motors.
 * 
 * @author1 Cristian Ciungu
 * @author2 Hao Shu 
 * @version 2019-01-22
 * 
 */




// declare package;

package ca.mcgill.ecse211.wallfollowing;


// imports

import lejos.hardware.motor.*;



public class BangBangController implements UltrasonicController {


  // parameters 
  private final int bandCenter;

  private final int bandWidth;

  private final int motorLow;

  private final int motorHigh;

   private int distance;

// define a constructor for the Bang-Bang controller

  public BangBangController(int bandCenter, int bandWidth,  int motorLow,int motorHigh) {


    this.bandCenter = bandCenter;

    this.bandWidth = bandWidth;

    this.motorLow = motorLow;

    this.motorHigh = motorHigh;

    // Set robot in motion 

    WallFollowingLab.leftMotor.setSpeed(motorHigh); 

    WallFollowingLab.rightMotor.setSpeed(motorHigh);

    WallFollowingLab.leftMotor.forward();

    WallFollowingLab.rightMotor.forward();

  }







  // filter the US sensor readings 

  public void processUSData(int distance) {

    this.distance = distance;

    

    //compute the error

    int distError = this.distance-this.bandCenter;

    if(Math.abs(distError)<=bandWidth) {//when within the tolerance, the robot moves in a straight line

    	return;

    }

    else if (distError>0) {

    	if(distError>12) {//too far away from the wall

    		WallFollowingLab.rightMotor.setSpeed(motorHigh-20); // when the robot comes too far away, it will turn faster

            WallFollowingLab.leftMotor.setSpeed(motorLow);

            WallFollowingLab.rightMotor.forward();

            WallFollowingLab.leftMotor.forward();

    	}

    	else{WallFollowingLab.rightMotor.setSpeed(motorHigh-40); // Start robot moving closer to the wall

        WallFollowingLab.leftMotor.setSpeed(motorLow);

        WallFollowingLab.rightMotor.forward();

        WallFollowingLab.leftMotor.forward();

    }

    	}

    else if (distError<0) {

    	if(distError<-3) { //too close to the wall

    		WallFollowingLab.rightMotor.setSpeed(motorHigh); // when the robot comes too close, it will turn away faster.

            WallFollowingLab.leftMotor.setSpeed(motorHigh);

            WallFollowingLab.rightMotor.backward();

            WallFollowingLab.leftMotor.forward();

    	}else

    	{

    	WallFollowingLab.rightMotor.setSpeed(motorLow); // Start robot turning away from the wall

        WallFollowingLab.leftMotor.setSpeed(motorHigh);

        WallFollowingLab.rightMotor.forward();

        WallFollowingLab.leftMotor.forward();

    


  }}
    } // end processUSData



  // returns the distance reading from the US sensor 

  public int readUSDistance() {

    return this.distance;

  } // end readUSDistance 

} // end BangBangController 
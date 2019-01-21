

/*
 * lab group 12
 * 
 * @author1: Hao Shu
 * @author2: Cristian Ciungu
 */


package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandWidth;
  private final int motorLow;
  private final int motorHigh;
  
 
  private int distance;

  public BangBangController(int bandCenter, int bandWidth,  int motorLow,int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandWidth = bandWidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
 
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }



@Override
  public void processUSData(int distance) {
    this.distance = distance;
    
    //create the error
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
    	if(distError<-5) {//to close to the wall
    		WallFollowingLab.rightMotor.setSpeed(motorLow); // when the robot comes too close, it will turn away faster.
            WallFollowingLab.leftMotor.setSpeed(motorHigh+125);
            WallFollowingLab.rightMotor.forward();
            WallFollowingLab.leftMotor.forward();
    	}else
    	{
    	
  
    	WallFollowingLab.rightMotor.setSpeed(motorLow); // Start robot turning away from the wall
        WallFollowingLab.leftMotor.setSpeed(motorHigh);
        WallFollowingLab.rightMotor.forward();
        WallFollowingLab.leftMotor.forward();
    
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
  }}}

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}

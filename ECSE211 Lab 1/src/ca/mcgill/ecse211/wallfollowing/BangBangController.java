package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandWidth;
  private final int motorLowInward;
  private final int motorHighInward;
  private final int motorLowOutward;
  private final int motorHighOutward;
  
 
  private int distance;

  public BangBangController(int bandCenter, int bandWidth, int motorLowInward, int motorHighInward, int motorLowOutward,int motorHighOutward) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandWidth = bandWidth;
    this.motorLowOutward = motorLowOutward;
    this.motorHighOutward = motorHighInward;
    this.motorLowInward = motorLowInward;
    this.motorHighInward = motorHighInward;
    WallFollowingLab.leftMotor.setSpeed(motorHighInward); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHighInward);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }



@Override
  public void processUSData(int distance) {
    this.distance = distance;
    
    //create the error
    float distError = distance-bandCenter;
    if(Math.abs(distError)<=bandWidth) {//when within the tolerance
    	return;
    }
    else if (distError<0) {
    	if(distError<-4) {
    		WallFollowingLab.leftMotor.setSpeed(motorHighInward); // Start robot moving forward
            WallFollowingLab.rightMotor.setSpeed(motorLowInward);
            WallFollowingLab.leftMotor.forward();
            WallFollowingLab.rightMotor.forward();
    	}
    	else{WallFollowingLab.leftMotor.setSpeed(motorHighOutward); // Start robot moving forward
        WallFollowingLab.rightMotor.setSpeed(motorLowOutward);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }
    	}
    else if (distError>0) {
    	if(distError>4) {
    		WallFollowingLab.leftMotor.setSpeed(motorLowInward); // Start robot moving forward
            WallFollowingLab.rightMotor.setSpeed(motorHighInward);
            WallFollowingLab.leftMotor.forward();
            WallFollowingLab.rightMotor.forward();
    	}
    	
  
    	WallFollowingLab.leftMotor.setSpeed(motorLowOutward); // Start robot moving forward
        WallFollowingLab.rightMotor.setSpeed(motorHighOutward);
        WallFollowingLab.leftMotor.forward();
        WallFollowingLab.rightMotor.forward();
    }
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}


/*
 * lab group 12
 * 
 * @author1: Hao Shu
 * @author2: Cristian Ciungu
 */


package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  private static final int constant = 2 ; //set the proportional constant

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;


  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    
  int distError=this.distance - this.bandCenter;
  int gained_Error=distError*constant;//define the gained error
  
  if(gained_Error>=255) {
	  gained_Error=255;//to make sure the gained error won't be too large
  }
  
  if(Math.abs(distError)<=bandWidth) {//when within the tolerance, the robot moves in a straight line
  	return;
  }else if (distError>0) {
  	if(distError>10) {//too far away from the wall
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+(int)(gained_Error/1.5)); // when the robot comes too far away, it will turn faster
        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-(int)(gained_Error/1.5));
        WallFollowingLab.rightMotor.forward();
        WallFollowingLab.leftMotor.forward();
	}
	else{
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+(int)(gained_Error/1.5)); // Start robot moving closer to the wall
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.rightMotor.forward();
    WallFollowingLab.leftMotor.forward();
}
	}
else if (distError<0) {
	if(distError<-3) {//too close to the wall
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED); // when the robot comes too close, it will turn away faster.
        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
        WallFollowingLab.rightMotor.backward();
        WallFollowingLab.leftMotor.forward();
	}else
	{


		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-(int)(gained_Error/1.5)); // when the robot comes too close, it will turn away faster.
        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+(int)(gained_Error/1.5)); // Start robot turning away from the wall
    WallFollowingLab.rightMotor.forward();
    WallFollowingLab.leftMotor.forward();

}}

    // TODO: process a movement based on the us distance passed in (P style)
  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}

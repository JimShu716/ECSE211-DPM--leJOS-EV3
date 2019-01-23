/**
* This class provides a constructor for a P-type controller. 
* P-type control allows the autonomous tribot to execute path 
* corrections that are proportional to the magnitude of the
* error computed. The distance readings are filtered using 
* the processUSData method. 
* 
* Repeated abnormally large readings from the ultrasonic sensor
* are ignored. 
*
*@author1 Cristian Ciungu
*@author2 Hao Shu 
*@version 2019-01-22
*
 */


// declare package

package ca.mcgill.ecse211.wallfollowing;

// imports 

import lejos.hardware.motor.EV3LargeRegulatedMotor;



public class PController implements UltrasonicController {



  // constants 

  private static final int MOTOR_SPEED = 200;

  private static final int FILTER_OUT = 40;

  private static final int constant = 2 ; //set the proportional constant


  // parameters 
  private final int bandCenter;

  private final int bandWidth;

  private int distance;

  private int filterControl;



  // define constructor for PController 

  public PController(int bandCenter, int bandwidth) {

    this.bandCenter = bandCenter;

    this.bandWidth = bandwidth;

    this.filterControl = 0;



    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward

    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);

    WallFollowingLab.leftMotor.forward();

    WallFollowingLab.rightMotor.forward();

  }



  

  public void processUSData(int distance) {



    // rudimentary filter - toss out invalid samples corresponding to null

    // signal.

    // (n.b. this was not included in the Bang-bang controller, but easily

    // could have).

    //

    if (distance>=255 && filterControl < FILTER_OUT) {

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

  int gained_Error=Math.abs(distError*constant);//define the gained error

  

  if(gained_Error>=200) {

	  gained_Error=200;//to make sure the gained error won't be too larges

  }

  

  if(Math.abs(distError)<=bandWidth) {//when within the tolerance, the robot moves in a straight line

	  	return;
  }

  else if (Math.abs(distError)>bandWidth&&distError>0) {

	  	if(distError>3) {//too far away from the wall

			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+(int)(gained_Error/3.9)); // when the robot comes too far away, it will turn faster

	        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-(int)(gained_Error/1.7));
	        WallFollowingLab.rightMotor.forward();

	        WallFollowingLab.leftMotor.forward();

		}

		else{

			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+(int)(gained_Error/3.5)); // Start robot moving closer to the wall

	    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-(int)(gained_Error/1.9));

	    WallFollowingLab.rightMotor.forward();

	    WallFollowingLab.leftMotor.forward();

	}

		}
else if (Math.abs(distError)>bandWidth&&distError<1) {

	if(distError>=-13&&distError< 0) {//too close to the wall

		WallFollowingLab.rightMotor.setSpeed(0); // when the robot comes too close, it will turn away

        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-(int)(gained_Error/1.5));//reduce the turning speed so it won't turn too much

        WallFollowingLab.rightMotor.backward();

        WallFollowingLab.leftMotor.forward();

	}else if(distError <-13){
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+(int)(gained_Error)); // when the robot comes too close, it will go backwards to adjust.

        WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-(int)(gained_Error));

        WallFollowingLab.rightMotor.backward();

        WallFollowingLab.leftMotor.backward();
		
	}

	


}



  } // end processUSData





  public int readUSDistance() {

    return this.distance;

  } // end readUSDistance 


}
package ca.mcgill.ecse211.searchzonelocator;

import ca.mcgill.ecse211.lab5.Navigation;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * This class handles the robot's navigation from the starting corner
 * to the search zone after localization is complete
 * 
 * @author Carlo D'Angelo
 *
 */
public class SearchZoneLocator {

	private Odometer odo;
	private static final double TILE_SIZE = 30.48;
	private int LLx, LLy, SC;
	private LightLocalizer lightLocalizer;
	private Navigation navigator;
	
	/**
	 * This is the default constructor of this class
	 * @param SC starting corner of robot
	 * @param LLx x position of lower left corner of search zone
	 * @param LLy x position of lower left corner of search zone
	 * @param lightLocalizer instance of LightLocalizer class
	 * @param navigator instance of Navigator class
	 * @throws OdometerExceptions
	 */
	public SearchZoneLocator(int SC, int LLx, int LLy, LightLocalizer lightLocalizer,
			Navigation navigator) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.SC = SC;	
		this.LLx = LLx;
		this.LLy = LLy;
		this.lightLocalizer = lightLocalizer;
		this.navigator= navigator;
	}
	
	/** 
	 * Method that allows the robot to make its way to the search zone
	 * The path the robot takes will depend on the starting corner (SC) parameter
	 */
	public void goToSearchZone(){
		
		switch(SC){
			case 0: 
				//current position is (1,1)
				odo.setXYT(1.0 * TILE_SIZE, 1.0 * TILE_SIZE, 0.0);
				//go to LLx
				//go to LLy
				navigator.travelTo(LLx,1);
				if (LLx != 1) {
					navigator.turnTo(-45);
					navigator.driveBack(5);
					lightLocalizer.lightLocalize(LLx, 1);
				}
				navigator.travelTo(LLx,LLy);
				navigator.turnTo(45);
				navigator.driveBack(5);
				lightLocalizer.lightLocalize(LLx, LLy);
				break;
			case 1: 
				//current position is (7,1)
				odo.setXYT(7.0 * TILE_SIZE, 1.0 * TILE_SIZE, 270.0);
				//go to LLx
				//go to LLy
				navigator.travelTo(LLx,1);
				if (LLx != 7) {
					navigator.turnTo(135);
					navigator.driveBack(5);
					lightLocalizer.lightLocalize(LLx, 1);
				}
				navigator.travelTo(LLx,LLy);
				navigator.turnTo(45);
				navigator.driveBack(5);
				lightLocalizer.lightLocalize(LLx, LLy);
				break;
			case 2: 
				//current position is (7,7)
				odo.setXYT(7.0 * TILE_SIZE, 7.0 * TILE_SIZE, 180.0);
				//go to LLx
				//go to LLy
				navigator.travelTo(LLx - 1,7);
				navigator.turnTo(135);
				navigator.driveBack(5);
				lightLocalizer.lightLocalize(LLx - 1, 7);
				
				navigator.travelTo(LLx - 1,LLy);
				navigator.turnTo(-135);
				navigator.driveBack(5);
				lightLocalizer.lightLocalize(LLx - 1, LLy);
				
				navigator.travelTo(LLx,LLy);
				navigator.turnTo(-45);
				navigator.driveBack(5);
				lightLocalizer.lightLocalize(LLx, LLy);
				break;
			case 3:	
				//current position is (1,7)
				odo.setXYT(1.0 * TILE_SIZE, 7.0 * TILE_SIZE, 90.0);
				//go to LLx
				//go to LLy
				navigator.travelTo(LLx - 1,7);
				navigator.turnTo(-45);
				navigator.driveBack(5);
				lightLocalizer.lightLocalize(LLx - 1, 7);
				
				navigator.travelTo(LLx - 1,LLy);
				navigator.turnTo(-135);
				navigator.driveBack(5);
				lightLocalizer.lightLocalize(LLx - 1, LLy);
				
				navigator.travelTo(LLx,LLy);
				navigator.turnTo(-45);
				navigator.driveBack(5);
				lightLocalizer.lightLocalize(LLx, LLy);
				break;
		    default:
		    	System.out.println("Error - invalid button"); // None of the above - abort
		        System.exit(-1);
		        break;
		}
		
	} 
  
}

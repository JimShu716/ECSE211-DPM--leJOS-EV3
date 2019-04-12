package ca.mcgill.ecse211.canlocator;

import ca.mcgill.ecse211.lab5.Navigation;
import ca.mcgill.ecse211.localization.LightLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class CanLocator {

	private Odometer odo;
	private Navigation navigator;
	private CanDetect canDetect;
	private SampleProvider usDistance;
	private float[] usData;
	private LightLocalizer lightLocalizer;
	
	private static int FORWARD_SPEED = 100;
	private static double OFFSET = 0.5;
	private static final double TILE_SIZE = 30.48;
	private static final double CAN_DISTANCE_ON_BORDER = 18.5;
	private static final double CAN_DISTANCE_FROM_OUT = 11.75;
	private static final double ANGLE_ERROR =10.0;
	private static final double DISTANCE_ERROR =4.0;
	private double ENDX = 0, ENDY = 0;
	private double Cx = 0,Cy = 0, Ct = 0;
	
	private int TR;  //this variable stores the integer defining the target can color.
	private int count = 0;
	private int LLx, LLy, URx, URy;
	private static boolean fromInsideDodge = false;
	private static boolean loopStop = false;
	
	public CanLocator(CanDetect canDetect, SampleProvider usDistance, float[] usData, 
			Navigation navigator, LightLocalizer lightLocalizer, int TR, int LLx, int LLy, int URx, int URy) throws OdometerExceptions {
		odo = Odometer.getOdometer();
		this.usDistance = usDistance;
		this.usData = usData;
		this.canDetect = canDetect;
		this.navigator = navigator;
		this.TR = TR;
		this.LLx = LLx;
		this.LLy = LLy;
		this.URx = URx;
		this.URy = URy;
		this.Cy = LLy;
		this.Cx = LLx;
		this.ENDX = LLx+1;
		this.ENDY = LLy;
		this.lightLocalizer = lightLocalizer; 
	}
	
	
	/**
	 * RunLocator() is the method that runs the algorithm for searching for the correct can.
	 * It drives the EV3 forward and in a square around teh search zone and looks for cans.
	 * If a can is detected, it calls for the searchProcess(), otherwise it calls goToNext().
	 * Once it has traveled around the whole zone without finding the correct can it then travels
	 * to the upper right corner.
	 */
	
	public void RunLocator(){
		
		while (true && !loopStop) {	
			
			//when EV3 goes full circle with the algorithm
			//and ends where it started, break the loop.
			if(Cx == ENDX && Cy == ENDY) {
				
				lightLocalizer.lightLocalize(Cx,Cy);
				
				//If no can was found once algorithm is finished, go to Upper Right
				navigator.travelTo(Cx,LLy-OFFSET);
				navigator.travelTo(URx+OFFSET,LLy-OFFSET);
				navigator.travelTo(URx+OFFSET,URy);
				navigator.travelTo(URx,URy);
				break;
			}			
			
			else if(!checkCan()){

				//if the EV3 is at one of the 4 corners of the search zone
				if((Cx*TILE_SIZE > (LLx*TILE_SIZE-DISTANCE_ERROR) && Cx*TILE_SIZE < (LLx*TILE_SIZE+DISTANCE_ERROR) && Cy*TILE_SIZE > (LLy*TILE_SIZE-DISTANCE_ERROR) && Cy*TILE_SIZE < (LLy*TILE_SIZE+DISTANCE_ERROR))
						|| (Cx*TILE_SIZE > (LLx*TILE_SIZE-DISTANCE_ERROR) && Cx*TILE_SIZE < (LLx*TILE_SIZE+DISTANCE_ERROR) && Cy*TILE_SIZE > (URy*TILE_SIZE-DISTANCE_ERROR) && Cy*TILE_SIZE < (URy*TILE_SIZE+DISTANCE_ERROR))
						|| (Cx*TILE_SIZE > (URx*TILE_SIZE-DISTANCE_ERROR) && Cx*TILE_SIZE < (URx*TILE_SIZE+DISTANCE_ERROR) && Cy*TILE_SIZE > (URy*TILE_SIZE-DISTANCE_ERROR) && Cy*TILE_SIZE < (URy*TILE_SIZE+DISTANCE_ERROR))
						 || (Cx*TILE_SIZE > (URx*TILE_SIZE-DISTANCE_ERROR) && Cx*TILE_SIZE < (URx*TILE_SIZE+DISTANCE_ERROR) && Cy*TILE_SIZE > (LLy*TILE_SIZE-DISTANCE_ERROR) && Cy*TILE_SIZE < (LLy*TILE_SIZE+DISTANCE_ERROR))){
					
					goToNext();
					
				}
				
				else {
					
					if(!fromInsideDodge) navigator.turnTo(-90);
					fromInsideDodge = false;
					if(checkCan()){
						
						searchProcess();
						
					}
					
					else goToNext();
					
				}
			}
			
			//checks a can in front of it
			else{
				
				searchProcess();
				
			}
			
		}
		
	}	
	
	/**
	 * searchProcess() runs when the EV3 detects a can. When detected, it drives to it and checks its color.
	 * If the color is correct, it beeps once and travels to the upper right corner. Otherwise it
	 * reverses and calls one of the dodge methods depending on where the can was spotted.
	 * For instance, If an incorrect colored can is placed on the border
	 * the EV3 dodges outwards, and then sets the distanceToCan to CAN_DISTANCE_FROM_OUT so that
	 * the EV3 knows how far to move towards a can if it spots one.
	 */
	
	private void searchProcess(){
		
		double distanceToCan = 0.0;
		boolean inside = false;
		
		Sound.beep();
		//outside the border
		if(Cx*TILE_SIZE < LLx*TILE_SIZE-DISTANCE_ERROR || Cy*TILE_SIZE > URy*TILE_SIZE+DISTANCE_ERROR 
				|| Cx*TILE_SIZE > URx*TILE_SIZE+DISTANCE_ERROR || Cy*TILE_SIZE < LLy*TILE_SIZE-DISTANCE_ERROR){

			distanceToCan = CAN_DISTANCE_FROM_OUT;
			inside = false;
		}
		
		//on the border
		else{
			distanceToCan = CAN_DISTANCE_ON_BORDER;
			
			if(Cx == LLx && (Ct > (90-ANGLE_ERROR) && Ct < (90+ANGLE_ERROR))){
				
				Cx = (Cx*TILE_SIZE+CAN_DISTANCE_ON_BORDER)/TILE_SIZE;
				
			}
			else if (Cy == URy && (Ct > (180-ANGLE_ERROR) && Ct < (180+ANGLE_ERROR))) {
				
				Cy = (Cy*TILE_SIZE-CAN_DISTANCE_ON_BORDER)/TILE_SIZE;
				
			}
			else if ( Cx == URx && Ct > (270-ANGLE_ERROR) && Ct < (270+ANGLE_ERROR)){
				
				Cx = (Cx*TILE_SIZE-CAN_DISTANCE_ON_BORDER)/TILE_SIZE;
				
			}
			else if( Cy == LLy && Ct > (360-ANGLE_ERROR) || Ct < (0+ANGLE_ERROR)){
				
				Cx = (Cx*TILE_SIZE+CAN_DISTANCE_ON_BORDER)/TILE_SIZE;
				
			}
			
			inside = true;
		}
	
		if(checkColor(distanceToCan)){
			
			if(inside){

				if((Cx*TILE_SIZE > (LLx*TILE_SIZE+10) && (Ct > (90-ANGLE_ERROR) && Ct < (90+ANGLE_ERROR)))
						|| (Cy*TILE_SIZE < (URy*TILE_SIZE-10) && (Ct > (180-ANGLE_ERROR) && Ct < (180+ANGLE_ERROR)))
						|| (Cx*TILE_SIZE < (URx*TILE_SIZE-10) && (Ct > (270-ANGLE_ERROR) && Ct < (270+ANGLE_ERROR)))
						 || Cy*TILE_SIZE > (LLy*TILE_SIZE+10) && (Ct > (360-ANGLE_ERROR) || Ct < (0+ANGLE_ERROR))){
					
					travelToURInside();
				}
				
				else{ 
					
					travelToURBorder();
				}
			}
		
			else{
				travelToUROutside();
			}
			
		}
		
		else{
			if(inside){
				
				if((Cx*TILE_SIZE > (LLx*TILE_SIZE-DISTANCE_ERROR) && Cx*TILE_SIZE < (LLx*TILE_SIZE+DISTANCE_ERROR) && (Ct > (360-ANGLE_ERROR) || Ct < (0+ANGLE_ERROR)))
						|| (Cy*TILE_SIZE < (URy*TILE_SIZE+DISTANCE_ERROR) && Cy*TILE_SIZE > (URy*TILE_SIZE-DISTANCE_ERROR) && (Ct > (90-ANGLE_ERROR) && Ct < (90+ANGLE_ERROR)))
						|| (Cx*TILE_SIZE < (URx*TILE_SIZE+DISTANCE_ERROR) && Cx*TILE_SIZE > (URx*TILE_SIZE-DISTANCE_ERROR) && (Ct > (180-ANGLE_ERROR) && Ct < (180+ANGLE_ERROR)))
						 || Cy*TILE_SIZE > (LLy*TILE_SIZE-DISTANCE_ERROR) && Cy*TILE_SIZE < (LLy*TILE_SIZE+DISTANCE_ERROR) && (Ct > (270-ANGLE_ERROR) && Ct < (270+ANGLE_ERROR))){//////
					borderDodge();
				}
				
				else{
					insideDodge();
				}
			}
			
			else{
				outsideDodge();
			}
		}
		
	}
	
	/**
	*checkCan() returns true if a can was spotten by the ultrasonic sensor within the
	*range of a tile. Otherwise, it returns false.
	*/
	
	//robot is facing the can
	private boolean checkCan(){
	
		//read sensor and see if a can is detected in range
		if(readUSDistance() <= TILE_SIZE+DISTANCE_ERROR) return true;
		else return false;
		
	}
	
	/**
	*checkColor() is a method that is called after checkCan(). It 
	*makes the EV3 beep once and return true if the can scanned is the target can
	*Otherwise, it will beep twice and return false.
	*@param distance
	*/
	
	private boolean checkColor(double distance){

		navigator.moveToCan(distance);
		
		//if the can color is the target color, beep once
		if (TR == canDetect.run()) {
			Sound.beep();
			return true;
		}
		
		//otherwise, beep twice
		else {
			Sound.beep(); 
			Sound.beep();
			return false;
		}
	} 
	
	/**
	*goToNext() moves the EV3 forward to the next position when no cans are detected.
	*/

	private void goToNext() { 

		
		navigator.moveToCan(TILE_SIZE);
		navigator.turnTo(90);

		//keeps coordinate values in check to update odo if needed
		if(Cy < URy && Cx==LLx) {
			Cy = Cy+1;
			odo.setY(Cy*TILE_SIZE);
		}
		else if(Cx < URx && Cy==URy) { 
			Cx = Cx+1;
			odo.setX(Cx*TILE_SIZE);
		}
		else if(Cy > LLy && Cx==URx) {
			Cy=Cy-1;
			odo.setY(Cy*TILE_SIZE);
		}
		
		//ENDX is the x coordinate of the final position of the EV3
		else if(Cx > ENDX && Cy==LLy) {
			Cx=Cx-1;
			odo.setX(Cx*TILE_SIZE);
		}
		
		
		if(Cy < URy && Cx==LLx) {
			Ct = 90.0;
			odo.setTheta(Ct);
		}
		else if(Cx < URx && Cy==URy) { 
			Ct = 180.0;
			odo.setTheta(Ct);
		}
		else if(Cy > LLy && Cx==URx) {
			Ct = 270.0;
			odo.setTheta(Ct);
		}
		
		//ENDX is the x coordinate of the final position of the EV3
		else if(Cx >= ENDX && Cy==LLy) {
			Ct = 0.0;
			odo.setTheta(Ct);
		}
		
	}
	
	/**
	*travelToURBorder() is called when the correct can is found on the edge of the search zone. This
	*method will use travelTo() from the Navigator class to get the EV3 to the upper right corner.
	*/
	
	private void travelToURBorder() {
		
		navigator.driveBack(CAN_DISTANCE_ON_BORDER);
		
		if ( (odo.getXYT()[2] >= 360-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 0+ANGLE_ERROR)){
			
			navigator.travelTo(LLx-OFFSET, Cy);
			navigator.travelTo(Cx,URy+OFFSET);
			navigator.travelTo(URx,Cy);
			navigator.travelTo(URx,URy);		
		}
		
		else if ( (odo.getXYT()[2] >= 90-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 90+ANGLE_ERROR) ){

			navigator.travelTo(Cx,URy+OFFSET);
			navigator.travelTo(Cy,URy+OFFSET);
			navigator.travelTo(URx,URy+OFFSET);
			navigator.travelTo(URx,URy);
		}
		
		else if ( (odo.getXYT()[2] >= 180-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 180+ANGLE_ERROR) ){
		
			navigator.travelTo(URx+OFFSET,Cy);
			navigator.travelTo(URx+OFFSET,URy);
			navigator.travelTo(URx,URy);
		}
		
		else if ( (odo.getXYT()[2] >= 270-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 270+ANGLE_ERROR) ){
		
			navigator.travelTo(Cx,LLy-OFFSET);
			navigator.travelTo(URx+OFFSET,LLy-OFFSET);
			navigator.travelTo(URx+OFFSET,URy);
			navigator.travelTo(URx,URy);
		}
		
		loopStop = true;
	}
	
		

	/**
	*travelToUROutside() is called when the correct can is found from the outside of
	*the search zone. This method will use travelTo() from the Navigator class
	*to get the EV3 to the upper right corner.
	*/
					   
	private void travelToUROutside() {
		
		navigator.driveBack(CAN_DISTANCE_FROM_OUT);
		//lightLocalizer.lightLocalize(Cx,Cy);
		
		if ( (odo.getXYT()[2] >= 360-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 0+ANGLE_ERROR)){
		
			navigator.travelTo(Cx, URy+OFFSET);
			navigator.travelTo(URx,URy+OFFSET);
			navigator.travelTo(URx,URy);	
		}
		
		else if ( (odo.getXYT()[2] >= 90-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 90+ANGLE_ERROR) ){
			
			navigator.travelTo(URx,URy+OFFSET);
			navigator.travelTo(URx,URy);
		}
		
		else if ( (odo.getXYT()[2] >= 180-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 180+ANGLE_ERROR) ){
		
			navigator.travelTo(URx+OFFSET,URy);
			navigator.travelTo(URx,URy);
		}
		
		else if ( (odo.getXYT()[2] >= 270-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 270+ANGLE_ERROR) ){
		
			navigator.travelTo(URx+OFFSET,Cy);
			navigator.travelTo(URx+OFFSET,URy);
			navigator.travelTo(URx,URy);
		}
		
		loopStop = true;
		
	}
	
	private void travelToURInside(){
		
		navigator.driveBack(CAN_DISTANCE_ON_BORDER+0.5*TILE_SIZE);
//		//lightLocalizer.lightLocalize(Cx,Cy);
//		
//		if(Cy == URy && Cx < URx){
//			navigator.turnTo(90);
//		}
//		
//		else if(Cx==URx && Cy > LLy){
//			navigator.turnTo(180);
//		}
//		
//		else if(Cy==LLy && Cx > LLx){
//			navigator.turnTo(270);
//		}
		
		//on the first edge
		if((odo.getXYT()[2] >= 90-ANGLE_ERROR) && 
		    	(odo.getXYT()[2] <= 90+ANGLE_ERROR)){
//			navigator.turnTo(-90);
//			navigator.driveForward(URy*TILE_SIZE-(Cy)*TILE_SIZE+CAN_DISTANCE_FROM_OUT);
//			navigator.turnTo(90);
//			navigator.driveForward(URx*TILE_SIZE-(LLx)*TILE_SIZE + CAN_DISTANCE_FROM_OUT);
//			navigator.turnTo(90);
//			navigator.driveForward(CAN_DISTANCE_FROM_OUT);
			
			navigator.travelTo(odo.getXYT()[0],URy+OFFSET);
			navigator.travelTo(URx,URy+OFFSET);
			navigator.travelTo(URx,URy);
			
		}
		
		//on the second edge
		else if(odo.getXYT()[2] > (180-ANGLE_ERROR) && odo.getXYT()[2] < (180+ANGLE_ERROR)){
//			navigator.turnTo(-90);
//			navigator.driveForward(URx*TILE_SIZE-Cx*TILE_SIZE);
//			navigator.turnTo(90);
//			navigator.driveForward(CAN_DISTANCE_FROM_OUT);	
			
			navigator.travelTo(URx,URy+OFFSET);
			navigator.travelTo(URx,URy);
			
		}
		
		//on the third edge
		else if(odo.getXYT()[2] > 270-ANGLE_ERROR && odo.getXYT()[2] < 270-+ANGLE_ERROR){
//			navigator.turnTo(90);
//			navigator.driveForward(URy*TILE_SIZE-Cy*TILE_SIZE);
//			navigator.turnTo(-90);
//			navigator.driveForward(CAN_DISTANCE_FROM_OUT);	
			
			navigator.travelTo(URx+OFFSET,URy);
			navigator.travelTo(URx,URy);
			
		}
		
		//on the fourth edge
		else if(odo.getXYT()[2] > (360-ANGLE_ERROR) || odo.getXYT()[2] < (0+ANGLE_ERROR)){
//			navigator.turnTo(90);
//			navigator.driveForward(URx*TILE_SIZE-Cx*TILE_SIZE+CAN_DISTANCE_FROM_OUT);
//			navigator.turnTo(-90);
//			navigator.driveForward(URy*TILE_SIZE-Cx*TILE_SIZE+CAN_DISTANCE_FROM_OUT);
//			navigator.turnTo(-90);
//			navigator.driveForward(CAN_DISTANCE_FROM_OUT);
			
			navigator.travelTo(URx+OFFSET,LLy-OFFSET);
			navigator.travelTo(URx+OFFSET,URy);
			navigator.travelTo(URx,URy);
			
		}
		
		loopStop = true;
		
	}
	
	/**
	*borderDodge() is called when an incorrect color of a can is detected. 
	*The EV3 will dodge the can and continue its trip to look for the correct one.
	*/
	
	private void borderDodge() {
		
		if ( (odo.getXYT()[2] >= 360-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 0+ANGLE_ERROR)){
		
			navigator.driveBack(CAN_DISTANCE_ON_BORDER);
			
			if(Cy*TILE_SIZE >= (URy-1)*TILE_SIZE-DISTANCE_ERROR &&
			   	Cy*TILE_SIZE <= (URy-1)*TILE_SIZE+DISTANCE_ERROR){
				
				//the 1.5 added is to make the EV3 dodge 1.5 times a tile
				navigator.travelTo(LLx-OFFSET, Cy);
				navigator.travelTo(LLx-OFFSET,(Cy) + 1.5);
				navigator.travelTo(LLx-OFFSET+1.5,(Cy) + 1.5);
				navigator.turnTo(90);
			}
			
			else{
				//the 2 added is to make the EV3 dodge 2 times a tile
				navigator.travelTo(LLx-OFFSET, Cy);
				navigator.travelTo(LLx-OFFSET,(Cy) + 2);
				navigator.turnTo(90);
			}
		}
		
		
		else if ( (odo.getXYT()[2] >= 90-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 90+ANGLE_ERROR) ){
			
			navigator.driveBack(CAN_DISTANCE_ON_BORDER);
			navigator.travelTo(Cx,URy+OFFSET);
			navigator.travelTo((Cx) + 2, URy+OFFSET);
			navigator.turnTo(90);
		}
		
		else if ( (odo.getXYT()[2] >= 180-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 180+ANGLE_ERROR) ){
		
			navigator.driveBack(CAN_DISTANCE_ON_BORDER);
			
			if(odo.getXYT()[1]/TILE_SIZE <= LLy+1+DISTANCE_ERROR ||
			   	odo.getXYT()[1]/TILE_SIZE >= LLy+1-DISTANCE_ERROR){
				
				navigator.travelTo(URx+OFFSET,Cy);
				navigator.travelTo(URx+OFFSET,(Cy) - 1.5);
				navigator.travelTo(URx+OFFSET-1.5,(Cy) - 1.5);
				navigator.turnTo(90);
			}
			
			else{
				
				navigator.travelTo(URx+OFFSET,Cy);
				navigator.travelTo(URx+OFFSET,(Cy) - 2);
				navigator.turnTo(90);
			}
		}
		
		else if ( (odo.getXYT()[2] >= 270-ANGLE_ERROR) || 
		    	(odo.getXYT()[2] <= 270+ANGLE_ERROR) ){
		
			navigator.driveBack(CAN_DISTANCE_ON_BORDER);
			navigator.travelTo(Cx,LLy-OFFSET);
			navigator.travelTo((Cx) - 2, LLy-OFFSET);
			navigator.turnTo(90);
		}
	}
	
	/**
	*outsideDodge() is called if the EV3 is outside the zonoe and it needs to
	*avoid an incorrect can.
	*/				   
	
	private void outsideDodge() {
	
		navigator.driveBack(CAN_DISTANCE_ON_BORDER);
		navigator.turnTo(-90);
		
		if(Cy*TILE_SIZE <= (URy*TILE_SIZE+DISTANCE_ERROR) ||
				Cy*TILE_SIZE >= (URy*TILE_SIZE-DISTANCE_ERROR) ||
				Cx*TILE_SIZE <= (URx*TILE_SIZE+DISTANCE_ERROR)
				|| Cx*TILE_SIZE >= (URx*TILE_SIZE-DISTANCE_ERROR)){
			navigator.driveForward(TILE_SIZE*0.5);
			navigator.turnTo(90);
			navigator.driveForward(TILE_SIZE*1.5);
			navigator.turnTo(90);
		}
		
		else{
			navigator.driveForward(TILE_SIZE);
			navigator.turnTo(90);
		}
			
	}
	
	//returns to the next coordinate on the square without turning 90 deg right 
	private void  insideDodge(){

			//on the first edge of square
			navigator.driveBack(CAN_DISTANCE_ON_BORDER);
			
			if((Ct > (90-ANGLE_ERROR) && Ct < (90+ANGLE_ERROR))){
				
				Cx = (int)(Cx*TILE_SIZE-CAN_DISTANCE_ON_BORDER)/TILE_SIZE;
				
			}
			else if (Cy == URy && (Ct > (180-ANGLE_ERROR) && Ct < (180+ANGLE_ERROR))) {
				
				Cy = (int)(Cy*TILE_SIZE+CAN_DISTANCE_ON_BORDER)/TILE_SIZE;
				
			}
			else if ( Cx == URx && Ct > (270-ANGLE_ERROR) && Ct < (270+ANGLE_ERROR)){
				
				Cx = (int)(Cx*TILE_SIZE+CAN_DISTANCE_ON_BORDER)/TILE_SIZE;
				
			}
			else if( Cy == LLy && Ct > (360-ANGLE_ERROR) || Ct < (0+ANGLE_ERROR)){
				
				Cx = (int)(Cx*TILE_SIZE-CAN_DISTANCE_ON_BORDER)/TILE_SIZE;
				
			}
			
		
			
			navigator.turnTo(-90);
			fromInsideDodge = true;
	}
	
	
		
	private int readUSDistance() {
		//this method returns the ultrasonic distance read.
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
		
	}
  
}

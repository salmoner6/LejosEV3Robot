package robot;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.navigation.MovePilot;

public class Motors extends MovePilot {

	static RegulatedMotor pinces;
	protected boolean areOpen;

	public Motors() {
		super(5.6, 12.1, new EV3LargeRegulatedMotor(MotorPort.C), new EV3LargeRegulatedMotor(MotorPort.A));
		pinces = new EV3MediumRegulatedMotor(MotorPort.B);
		areOpen = true;
	}

	public void openPinces(boolean b) {
		//if (areOpen) return;
		//if b is true it will open while moving;
		
		pinces.setSpeed(10000);
		pinces.rotate(4 * 360,b);
		areOpen = true;
	}

	public void closePinces(boolean b) {
		////if b is true it will closes pinces while moving;
		//if (! areOpen) return;
		
		pinces.setSpeed(10000);
		pinces.rotate(-4 * 360,b);
		areOpen = false;
	}
}
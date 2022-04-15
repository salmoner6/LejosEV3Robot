package robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.Color;
import lejos.robotics.FixedRangeScanner;
import lejos.robotics.RangeFinder;
import lejos.robotics.RangeFinderAdapter;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Pose;
import lejos.robotics.objectdetection.RangeFeatureDetector;
import lejos.utility.Delay;

public class Robot {

	Sensors sensors;
	Motors motors;
	int degATourner;
	PoseProvider position;
	Pose startXYH;
	Point butXY;

	public final static int INITIAL = 0;
	public final static int DETECTER = 1;
	public final static int ALLER = 2;
	public final static int DEPLACER = 3;
	public final static int RAMASSER = 4;
	// public float boussole;

	public Robot() {
		sensors = new Sensors();
		motors = new Motors();
		degATourner = 0;
		//boussole = 0;
		position = new OdometryPoseProvider(motors);
		startXYH = new Pose(240, 28, 180);
		butXY = new Point(0, 28);
		position.setPose(startXYH);
	}

	public void run() {
		System.out.println("Pressez un bouton pour lancer");
		Button.waitForAnyPress();
		boolean b = true;
		int etat = INITIAL;
		while (b) {
			switch (etat) {
			case INITIAL:
				ramasserPremier();
				degATourner = 80;
				etat = DETECTER;
				System.out.println("etat passe de initial a detecter");
				break;
			case DETECTER:
				search(degATourner);
				// search(40);
				etat = ALLER;
				System.out.println("etat passe de detecter a aller");
				break;
			case ALLER:
				// we move forward and test for wall
				if (allerValMin() < 29) {
					etat = DEPLACER;
					System.out.println("etat passe de aller a deplacer");
				} else {
					etat = RAMASSER;
					System.out.println("etat passe de aller a ramasser");
				}
				break;
			case DEPLACER:
				System.out.println("Deplacer");
				motors.travel(-20);
				rotate(90);
				degATourner = 100;
				etat = DETECTER;
				System.out.println("etat passe de deplacer a detecter");
				break;
			case RAMASSER:
				System.out.println("Ramasser");
				ramasserPalet();
				marquerBut();
				etat = DETECTER;
				System.out.println("etat passe de ramasser a detecter");
				break;
			}
		}

	}

	public void ramasserPremier() {
		System.out.println("ramasserPremier()");
		motors.travel(35);
		motors.openPinces(false);
		motors.travel(10.5);
		motors.closePinces(false);
		rotate(45);
		travelDodge(30, true);
		rotate(-45);
		motors.setLinearSpeed(60);
		forwardToGoal();
		motors.openPinces(false);
		motors.travel(-8);
		motors.closePinces(false);
		rotate(180);

	}

	public void search(int arcLength) {
		System.out.println("search()");
		int degPerSec = 60;
		motors.setAngularSpeed(degPerSec);
		TreeMap<Double, Float> liste = new TreeMap<Double, Float>();
		rotate(arcLength, true);

		while (motors.isMoving() == true) {
			double i = (double) sensors.getDistance();
			liste.put(i, currentH());
			Delay.msDelay(100 / degPerSec);
		}
		System.out.println(liste.get(liste.firstKey()));
		float angle = liste.get(liste.firstKey());
		rotate(angle - currentH() - 5);
		// System.out.println("measurements: " + liste.size() );
		// rotate(angle, false);
//		System.out.println(" measurements: " + liste.size() + " val : " + val + "Angle : " + angle + " Indice : "
//				+ indice + " backtrack :" + backTrack + " list : " + liste);

	}

	public float allerValMin() {
		System.out.println("allerValMin()");
		motors.forward();
		while (motors.isMoving()) {
			if (sensors.getDistance() < 34) {
				motors.stop();
			}
		}
		motors.setLinearSpeed(10);
		motors.travel(5);
		return (float) sensors.getDistance();
	}

	public void ramasserPalet() {
		System.out.println("ramasserPalet()");
		motors.openPinces(false);
		motors.setLinearSpeed(35);
		motors.travel(28);
		motors.closePinces(false);

	}

	public void marquerBut() {
		System.out.println("marquerBut()");
		rotate(180 - currentH());
		forwardToGoal();
		motors.openPinces(false);
		motors.travel(-8);
		motors.closePinces(false);
		recalibrer();
		rotate(150);
		degATourner = 100;
	}

	public void recalibrer() {
		System.out.println("recalibrer");
		// Delay.msDelay(2000);
		rotate(-20);
		search(50);
		position.getPose().setHeading(180);
	}

	public void rotate(float angle) {
		motors.setAngularSpeed(130);
		motors.rotate((double) angle);
		// majBoussole(angle);
	}

	public void rotate(float angle, boolean b) {
		// rotate for search, returns control immediately 
		motors.setAngularSpeed(60);
		motors.rotate((double) angle, b);
		// majBoussole(angle);
	}

	public void travelDodge(double distance, boolean immediateReturn) {
		motors.travel(distance, immediateReturn);
		while (motors.isMoving()) {
			if (sensors.getDistance() < 20) {
				dodge();
			}
		}
	}

	public void forwardToGoal() {
		motors.forward();
		while (motors.isMoving()) {
			if (sensors.getColor() == 6 && sensors.getDistance() < 32) {
				motors.stop();
				return;
			}
			if (sensors.getDistance() < 26) {
				dodge();
			}
		}
	}

	public void avoidWall() {
		if (position.getPose().getY() < 78) {
			if (position.getPose().getHeading() <= 90 && position.getPose().getHeading() > -90) {
				rotate(45);
			} else
				rotate(-45);
		} else if (position.getPose().getHeading() <= 90 && position.getPose().getHeading() > -90) {
			rotate(-45);
		} else
			rotate(45);
	}

	public void dodge() {
		if (position.getPose().getY() < 78) {
			if (position.getPose().getHeading() <= 90 && position.getPose().getHeading() > -90) {
				dodgeLeft();
			} else
				dodgeRight();
		} else if (position.getPose().getHeading() <= 90 && position.getPose().getHeading() > -90) {
			dodgeRight();
		} else
			dodgeLeft();
	}

	public void dodgeLeft() {
		System.out.println("Dodging");
		rotate(50);
		Delay.msDelay(500);
		motors.forward();
		Delay.msDelay(1000);
		rotate(-50);
		Delay.msDelay(500);
		motors.forward();
	}

	public void dodgeRight() {
		System.out.println("Dodging");
		rotate(-50);
		Delay.msDelay(500);
		motors.forward();
		Delay.msDelay(1000);
		rotate(50);
		Delay.msDelay(500);
		motors.forward();
	}

	public float currentX() {
		return position.getPose().getX();
	}

	public float currentY() {
		return position.getPose().getY();
	}

	public float currentH() {
		return position.getPose().getHeading();
	}

	public void run2() {
		System.out.println("Pressez un bouton pour lancer");
		Button.waitForAnyPress();
		boolean b = true;
		int etat = DETECTER;
		degATourner = 90;
		Pose start2XYH = new Pose(0, 78, 180);
		position.setPose(start2XYH);
		rotate(-45);

		while (b) {
			switch (etat) {
			case DETECTER:
				search(degATourner);
				// search(40);
				etat = ALLER;
				System.out.println("etat passe de detecter a aller");
				break;
			case ALLER:
				// we move forward and test for wall
				if (allerValMin() < 29) {
					etat = DEPLACER;
					System.out.println("etat passe de aller a deplacer");
				} else {
					etat = RAMASSER;
					System.out.println("etat passe de aller a ramasser");
				}
				break;
			case DEPLACER:
				System.out.println("Deplacer");
				motors.travel(-20);
				rotate(90);
				degATourner = 100;
				etat = DETECTER;
				System.out.println("etat passe de deplacer a detecter");
				break;
			case RAMASSER:
				System.out.println("Ramasser");
				ramasserPalet();
				marquerBut();
				etat = DETECTER;
				System.out.println("etat passe de ramasser a detecter");
				break;
			}
		}

	}

	public static void main(String[] args) {
		Robot robo = new Robot();
		// System.out.println(robo.motors.getLinearSpeed());
		// robo.rotate(360);
		robo.run();
		// robo.run2();

		System.out.println("X: " + robo.currentX() + " Y: " + robo.currentY() + " H: " + robo.currentH());
	}
}

package robot;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

public class Sensors {
	protected EV3UltrasonicSensor us;
	protected EV3ColorSensor cs;
	protected EV3TouchSensor ts;

	public Sensors() {
		us = new EV3UltrasonicSensor(SensorPort.S2);
		ts = new EV3TouchSensor(SensorPort.S3);
		cs = new EV3ColorSensor(SensorPort.S1);
	}

	public int getColor() {
		SampleProvider s = this.cs.getColorIDMode();
		float[] sample = new float [s.sampleSize()];
		s.fetchSample(sample, 0);
		return (int) sample[0];
	}

	public float getDistance() {
		SampleProvider sp = us.getDistanceMode();
		float distanceValue;
		float[] sample = new float[sp.sampleSize()];
		sp.fetchSample(sample, 0);
		distanceValue = (float) sample[0] * 100;
		return distanceValue;
	}

	public double getTouch() {
		SampleProvider sp = ts.getTouchMode();
		double touch;
		float[] sample = new float[sp.sampleSize()];
		sp.fetchSample(sample, 0);
		touch = (double) sample[0];
		return touch;
	}

	public boolean isTouching() {
		return getTouch() != 0;
	}

	public boolean listen() {
		SampleProvider sp = us.getListenMode();
		float[] sample = new float[sp.sampleSize()];
		sp.fetchSample(sample, 0);
		 return sample[0]==1;
		
	}
}
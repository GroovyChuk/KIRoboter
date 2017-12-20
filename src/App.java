import java.io.IOException;
import java.util.Iterator;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class App {

	public static void main(String[] args) throws IOException {
		Wheel wheel1 = WheeledChassis.modelWheel(Motor.A, 23).offset(-86);
		Wheel wheel2 = WheeledChassis.modelWheel(Motor.D, 23).offset(86);
		EV3UltrasonicSensor ultraSonic = new EV3UltrasonicSensor(SensorPort.S1);
		EV3ColorSensor color = new EV3ColorSensor(SensorPort.S4);
		GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
		
		Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);

		g.drawString("Gestartet...", 5, 0, 0);
		
		pilot.setLinearSpeed(150); // cm per second
		pilot.setAngularSpeed(100);

		SampleProvider distance = ultraSonic.getMode("Distance");
		SampleProvider light = color.getMode("RGB");
		float[] sampleD = new float[distance.sampleSize()];
		float[] sampleL = new float[light.sampleSize()];
		
		distance.fetchSample(sampleD, 0);
		distance.fetchSample(sampleL, 0);
		
		for (int i = 0; i < 5; i++) {
			Button.waitForAnyPress();
			distance.fetchSample(sampleD, 0);
			g.drawString("Distanz: " + sampleD[0], 5, 20*i + 20, 0);
		}


		
//		for (int i = 0; i < 5; i++) {
//			pilot.travel(100); // cm
//			pilot.rotate(-180); // degree clockwise
//		}
		
//		pilot.travel(-100, true);
		
		while (pilot.isMoving())
			Thread.yield();
		pilot.stop();
		
		Button.waitForAnyPress();
		if(Button.ESCAPE.isDown()) System.exit(0);
	}
}

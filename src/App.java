import java.io.IOException;
import java.util.Iterator;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
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

	private static EV3UltrasonicSensor ultraSonic = new EV3UltrasonicSensor(SensorPort.S1);
	private static EV3ColorSensor color = new EV3ColorSensor(SensorPort.S4); 
	private static EV3Console console = new EV3Console();
	private static SampleProvider distance = ultraSonic.getMode("Distance");
	private static SampleProvider light = color.getMode("RGB");
	private static float[] sampleD = new float[distance.sampleSize()];
	private static float[] sampleL = new float[light.sampleSize()];
	
	public static void main(String[] args) throws IOException {
		Wheel wheel1 = WheeledChassis.modelWheel(Motor.A, 23).offset(-86);
		Wheel wheel2 = WheeledChassis.modelWheel(Motor.D, 23).offset(86);
		EV3MediumRegulatedMotor motor = new EV3MediumRegulatedMotor(MotorPort.D);

		GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
		
		Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);

		g.drawString("Connect to Server...", 5, 0, 0);
		
		g.drawString("Connected Successfully...", 10, 0, 0);
		
		console.log("Connecion established");
		
		pilot.setLinearSpeed(150); // cm per second
		pilot.setAngularSpeed(100);
		
		for (int i = 0; i < 3; i++) {
			Button.waitForAnyPress();
			readSensors();
		}
		
		pilot.travel(50);         // cm              
        pilot.rotate(90);        // degree clockwise
        pilot.rotate(-270);
        pilot.travel(50);               
        pilot.rotate(180);
        motor.rotateTo(90);
        motor.rotateTo(-90);

		
		while (pilot.isMoving())
			Thread.yield();
//		pilot.stop();
		
		Button.waitForAnyPress();
		if(Button.ESCAPE.isDown()) System.exit(0);
	}
	
	public static void readSensors() {
		distance.fetchSample(sampleD, 0);
		console.log("Distance: " + sampleD[0]);
		distance.fetchSample(sampleL, 0);
		console.log("Light: " + sampleL[0]);
	}
}

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

	private static EV3Console console;
	private static SensorReader sensorReader;
	private static Chassis chassis;
	private static MovePilot pilot;
	
	public static void main(String[] args) throws IOException {

		GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
		g.drawString("Connect to Server...", 5, 0, 0);
		
		console = new EV3Console();
		sensorReader = new SensorReader(console);
		Wheel wheel1 = WheeledChassis.modelWheel(Motor.A, 56).offset(-58);
		Wheel wheel2 = WheeledChassis.modelWheel(Motor.D, 56).offset(58);
		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(chassis);
		
		pilot.setLinearSpeed(150); // cm per second
		pilot.setAngularSpeed(100);
		
		followLine(500);
		
	}

	public static void aufgabe3_2() {
		float distance[];
		
		pilot.travel(4000,true);
		while (pilot.isMoving()) {
			Thread.yield();
			distance = sensorReader.readDistance();
			sensorReader.logSensorData();
			if (distance[0] >= 0.1)
				pilot.stop();
		}
	}		
	
	public static void aufgabe3() {
		
//		readSensors();
		pilot.travel(500);         // mm              
//		readSensors();
		pilot.rotate(-90);        // degree clockwise
        pilot.rotate(270);
        pilot.travel(500);  
//		readSensors();
        pilot.rotate(-180);
//		readSensors();

		while (pilot.isMoving())
			Thread.yield();
		pilot.stop();
	}
	
	public static void followLine(float step) {

		float light[];
		float travelledDistance = 0;
		boolean correct = false;
		
		console.log("following Line");
		pilot.travel(step, true);
		
		while (pilot.isMoving()) {
			Thread.yield();
			light = sensorReader.readLight();
			sensorReader.logSensorData();
			if (light[0] < 0.1) {
				travelledDistance = pilot.getMovement().getDistanceTraveled();
				correct = true;
				pilot.stop();
			}
		}
		
		if (correct) {
			correctMove();
			followLine(step-travelledDistance);
		}
	}
	
	public static void correctMove() {
		boolean corrected = false;
		
		console.log("correcting");
		pilot.rotate(20);
		pilot.travel(20,true);
		while (pilot.isMoving()) {
			Thread.yield();
			if (sensorReader.readLight()[0] > 0.1) {
				corrected = true;
				pilot.stop();
			} 
		}
		if (!corrected) {
			pilot.travel(-20);
			pilot.rotate(-40);
		}
	}
}

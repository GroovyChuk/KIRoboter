import java.io.IOException;
import java.util.ArrayList;

import org.freedesktop.dbus.Tuple;
import org.json.JSONArray;
import org.json.JSONObject;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;

public class App {

	private static SensorReader sensorReader;
	private static Chassis chassis;
	private static MovePilot pilot;
	private static MQTTClient mqttClient;
	private static double rotationC = 1.5f, sensorRotation = 0;
	private static EV3MediumRegulatedMotor sensorMotor;
	private static boolean firstMove = true;
	public static void main(String[] args) throws IOException {

		sensorMotor = new EV3MediumRegulatedMotor(MotorPort.B);
		sensorMotor.setSpeed(90.0f); // degrees per second
		GraphicsLCD g = LocalEV3.get().getGraphicsLCD();
		g.drawString("Connect to Server...", 5, 0, 0);
		mqttClient = new MQTTClient();

		sensorReader = new SensorReader();
		Wheel wheel1 = WheeledChassis.modelWheel(new EV3LargeRegulatedMotor(MotorPort.A), 56).offset(-75);
		Wheel wheel2 = WheeledChassis.modelWheel(new EV3LargeRegulatedMotor(MotorPort.D), 56).offset(75);

		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(chassis);
		
		pilot.setLinearSpeed(150); // cm per second
		pilot.setAngularSpeed(100);
		
		g.clear();
		g.drawString("Connected", 5, 0, 0);
		
//		excercise2(4000);
		
		while(true) {
			excercise3();
		}
	}

	public static void excercise1() {
		sensorReader.logSensorData();
		pilot.travel(500);         // mm              
		sensorReader.logSensorData();
		pilot.rotate(-90);        // degree clockwise
        pilot.rotate(270);
        pilot.travel(500);  
        sensorReader.logSensorData();
        pilot.rotate(-180);
        sensorReader.logSensorData();

		while (pilot.isMoving())
			Thread.yield();
		pilot.stop();
	}	
	
	public static void excercise2 (float distanceToDrive)
	{

		final float DRIVE_STEPS = 40;
		int stepAmount = (int)(distanceToDrive / DRIVE_STEPS);
	
		rotateSensor(-90);
		
		for (int i=0;i<stepAmount;++i)
		{
			followLine(DRIVE_STEPS);
			mqttClient.publish(sensorReader.readDistance()[0] + "",MQTTClient.TOPIC_SONIC_DISTANCE);
		}
		
		mqttClient.publish("End",MQTTClient.TOPIC_LOG);
		rotateSensorToTare();
	}
	
	public static void excercise3() {
		int rotations = 4, maxIndex = 0;
		int degreeStep = 360/rotations;
		double degree = 0.0, highestDistance = 0.0;
		ScanTuple ultrasonicValues[] = new ScanTuple[rotations];
			
		//measure sensor data
		for (int i = 0; i < rotations; i++) {
			double temp_distance = sensorReader.readDistance()[0];

			if (degree == 0) {
				ultrasonicValues[i] = new ScanTuple(sensorReader.readDistance()[0], 360);				
			} else
				ultrasonicValues[i] = new ScanTuple(sensorReader.readDistance()[0], degree);

			if (temp_distance > highestDistance) {
				highestDistance = temp_distance;
				maxIndex = i;
			}
			
			rotateSensor(degreeStep);
			degree += degreeStep;
		}
		sendSensorData(ultrasonicValues);
		
		//move
		if(!firstMove && ultrasonicValues[0].getDistance() > 0.55) {
			pilot.travel(500);
		} else {
			pilot.rotate(ultrasonicValues[maxIndex].getDegree());	
			if (ultrasonicValues[maxIndex].getDistance() > 0.55) {
				pilot.travel(500);
			} else 
				pilot.travel(ultrasonicValues[maxIndex].getDistance()*90);
		
			if (firstMove) {
				firstMove = false;
			}
		}
	}
	
	public static void followLine(float step) {

		float light[];
		float travelledDistance = 0;
		boolean correction = false;
		
		pilot.travel(step, true);
		
		while (pilot.isMoving()) {
			Thread.yield();
			light = sensorReader.readLight();
			
			if (light[0] < 0.1) {
				travelledDistance = pilot.getMovement().getDistanceTraveled();
				correction = true;
				pilot.stop();
			}
		}
		
		if (correction) {
			correctMove();
			followLine(step-travelledDistance);
		}
		fixOrientation(step);
		
	}
	
	public static void correctMove() {
		boolean corrected = false;
		
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
	
	public static void fixOrientation(double step) {
		double distance = sensorReader.readDistance()[0];
		if(distance >= 0.80 || distance < 0.05 ) {
			pilot.rotate(180);
			pilot.travel(step*2);
		}
	}
	
	public static void rotateSensor(double rotation) {
		double temp_rotation = sensorRotation;
		sensorRotation += rotation;
		if(sensorRotation > 180)
			sensorRotation -= 360;
		else if(sensorRotation < -180) {
			sensorRotation += 360;
		}

		sensorMotor.rotate((int) Math.round((sensorRotation - temp_rotation)*rotationC));
	}
	
	public static void rotateSensorToTare() {
		sensorMotor.rotate((int) Math.round((0.0-sensorRotation)*rotationC));
	}
	
	public static void startUltrasonicScan (int stepSize) {
		int angle;
		UltrasonicScan newScan = new UltrasonicScan(stepSize);
		
		for(angle=0;angle<=360;angle += stepSize){
			newScan.addResult(sensorReader.readDistance()[0]);
			Motor.C.rotate(stepSize);
		}
		publishUltrasonicScan(newScan);
	}
	
	public static boolean publishUltrasonicScan (UltrasonicScan ultrasonicScan) {
		 JSONObject json = new JSONObject();
		 JSONArray array = new JSONArray();
		 
		 try {
			 ArrayList<Float> results = ultrasonicScan.getResults();
		     for (Float result: results)
	         {
	             JSONObject item = new JSONObject();
	             item.put("value", result);
	             array.put(item);
	         }
	             json.put("scan_results", array);
	             MQTTClient.publish(json.toString(),MQTTClient.TOPIC_ULTRASONIC_SCAN_RESULT);
	         }
	         catch(Exception e){
	             e.printStackTrace();
	             return false;
	         }
	         return true;
	}
	
	public static void sendSensorData(ScanTuple ultrasonicValues[]) {
		JSONObject jsonResponse = new JSONObject();

        try {
            for (int i = 0; i < 4; ++i) {
                jsonResponse.put("" + ultrasonicValues[i].getDegree(),ultrasonicValues[i].getDistance());
            }
            jsonResponse.put("x_axis", true);
        }
        catch(Exception e){e.printStackTrace();}
        if (!jsonResponse.toString().equals(""))
            mqttClient.publish(jsonResponse.toString(),MQTTClient.TOPIC_SONIC_DISTANCE);		
	}
	
}
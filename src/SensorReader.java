import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class SensorReader {
	private SampleProvider distanceProvider;
	private SampleProvider lightProvider;
	private EV3UltrasonicSensor ultraSonicSensor;
	private EV3ColorSensor colorSensor;
	private EV3Console console;
	
	private float[] sampleDistance;
	private float[] sampleLight;
	
	public SensorReader (EV3Console ev3console) {
		ultraSonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
		colorSensor = new EV3ColorSensor(SensorPort.S4); 
		
		distanceProvider = ultraSonicSensor.getMode("Distance");
		lightProvider = colorSensor.getRedMode();
		
		sampleDistance = new float[distanceProvider.sampleSize()];
		sampleLight = new float[lightProvider.sampleSize()];
		
		console = ev3console;
	}
	
	public float[] readLight () {
		lightProvider.fetchSample(sampleLight,0);
		return sampleLight;
	}
	
	public float[] readDistance () {
		distanceProvider.fetchSample(sampleDistance,0);
		return sampleDistance;
	}
	
	public EV3UltrasonicSensor getUltraSonicSensor() {
		return ultraSonicSensor;
	}
	
	public void logSensorData () {
		console.log("Distance: " + readDistance()[0] + " Meter\nLight: " + readLight()[0]);
	}
}

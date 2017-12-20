import org.fusesource.mqtt.client.BlockingConnection;
import org.fusesource.mqtt.client.MQTT;
import org.fusesource.mqtt.client.QoS;

public class EV3Console {
	private MQTT mqtt;
    private BlockingConnection connection;
    private final String SERVER_ADDRESS = "192.168.43.246";
    private final int SERVER_PORT = 1883;
    private final String LOG_TOPIC = "log";
    private Thread connectThread;
	
	public EV3Console () {
	
		mqtt = new MQTT();
		try {
            mqtt.setHost(SERVER_ADDRESS,SERVER_PORT);
            connection = mqtt.blockingConnection();
            connection.connect();
            log("Connected Successfully...");
        } catch (Exception e) {
            e.printStackTrace();
        }
	}
	
	public void log (String msg) {
	  try {
            connection.publish(LOG_TOPIC, msg.getBytes() ,QoS.EXACTLY_ONCE, false);
	    } catch (Exception e) {
	        e.printStackTrace();
	    }
	}
	
}

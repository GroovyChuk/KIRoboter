import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import org.fusesource.mqtt.client.BlockingConnection;
import org.fusesource.mqtt.client.MQTT;
import org.fusesource.mqtt.client.QoS;

public class EV3Console {
	private MQTT mqtt;
    private BlockingConnection connection;
    private final String SERVER_ADDRESS = "192.168.43.246";
    private final int SERVER_PORT = 1883;
    private final String LOG_TOPIC = "log";
	DateFormat dateFormat = new SimpleDateFormat("HH:mm:ss");
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
		  	Date date = new Date();
            connection.publish(LOG_TOPIC, ("[" + dateFormat.format(date) + "] - " + msg).getBytes() ,QoS.AT_LEAST_ONCE, false);
	    } catch (Exception e) {
	        e.printStackTrace();
	    }
	}
	
}
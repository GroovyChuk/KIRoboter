import org.fusesource.mqtt.client.*;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;

/**
 * Created by Kalaman on 09.01.18.
 */
public class MQTTClient {
    private MQTT mqtt;
    private static BlockingConnection connection;
    private Topic [] topic;

    private ArrayList<MQTTListener> mqttListener;
    private static final String SERVER_IP = "192.168.43.246";
    private static final int SERVER_PORT = 1883;

    public static final String TOPIC_LOG = "log";
    public static final String TOPIC_NODE = "node";
    public static final String TOPIC_DRIVE = "drive";
    public static final String TOPIC_ULTRASONIC_SCAN = "ultrascan";
    public static final String TOPIC_ULTRASONIC_SCAN_RESULT = "ultrascan_result";
    public static final String TOPIC_SONIC_DISTANCE = "distance";

    public MQTTClient() {
        mqtt = new MQTT();
        mqttListener = new ArrayList<MQTTListener>();
        topic = new Topic[] {new Topic(TOPIC_DRIVE, QoS.EXACTLY_ONCE),new Topic(TOPIC_ULTRASONIC_SCAN,QoS.EXACTLY_ONCE)};

        try {
            mqtt.setHost(SERVER_IP, SERVER_PORT);
            connection = mqtt.blockingConnection();
            connection.connect();
            publish("Robot is connected [" + SERVER_IP + ":" + SERVER_PORT + "] ...",TOPIC_LOG);
            connection.subscribe(topic);

        }
        catch (Exception e){
            e.printStackTrace();
        }
    }

    public interface MQTTListener {
        public void onDriveReceived(float distanceInCM);
        public void onUltrasonicScanReceived (int stepSize);
    }

    public boolean addMQTTListener(MQTTListener listener) {
        return mqttListener.add(listener);
    }

    public boolean removeMQTTListener(MQTTListener listener) {
        return mqttListener.remove(listener);
    }

    public void startListeningThread () {
        new Thread() {
            @Override
            public void run() {
                try {
                    while(true) {
                        Message message = connection.receive();
                        String payload = new String(message.getPayload(), StandardCharsets.UTF_8);

                        if (message.getTopic().equals(TOPIC_DRIVE))
                        {
                            float distance = Float.parseFloat(payload) * (float)10;

                            for (MQTTListener listener : mqttListener)
                                listener.onDriveReceived(distance);
                        }
                        else if (message.getTopic().equals(TOPIC_ULTRASONIC_SCAN)) {

                            for (MQTTListener listener : mqttListener)
                                listener.onUltrasonicScanReceived(Integer.parseInt(payload));
                        }

                        message.ack();
                    }
                }
                catch (Exception e) {
                    e.printStackTrace();
                }
            }

        }.start();
    }

    public static void publish (String message, String topic) {
        try {
            connection.publish(topic, message.getBytes() ,QoS.EXACTLY_ONCE, false);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void publishLog (String msg) {
        publish(msg,TOPIC_LOG);
    }

}
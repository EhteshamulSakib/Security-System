package com.example.waltonpc.test;

import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.net.Uri;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.telephony.SmsManager;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import org.eclipse.paho.android.service.MqttAndroidClient;
import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
//import org.w3c.dom.Text;

public class MainActivity extends AppCompatActivity {
    static String MQTTHOST = "tcp://test.mosquitto.org:1883" ;


    MqttAndroidClient client;
    TextView hum  ;
    TextView temp  ;
    TextView pir  ;
    TextView smoke  ;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        hum = findViewById(R.id.hum_val) ;
        temp = findViewById(R.id.temp_val) ;
        pir = findViewById(R.id.pir_val) ;
        smoke = findViewById(R.id.smoke_val) ;

        String clientId = MqttClient.generateClientId();
        client = new MqttAndroidClient(this.getApplicationContext(), MQTTHOST, clientId);


        try {
            IMqttToken token = client.connect();
            token.setActionCallback(new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    Toast.makeText(MainActivity.this, "connected to server",Toast.LENGTH_LONG).show();
                    setSubscribe() ;

                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    Toast.makeText(MainActivity.this, "failed to connect to server !",Toast.LENGTH_LONG).show();

                }
            });
        } catch (MqttException e) {
            e.printStackTrace();
        }

        client.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {

            }

            @Override
            public void messageArrived(String topic, MqttMessage message) throws Exception {
                if ( topic.equals("sakib_pi/hum") )
                    hum.setText( new String( message.getPayload()  )  );
                if ( topic.equals("sakib_pi/temp") )
                    temp.setText( new String( message.getPayload() )  );
                if ( topic.equals("sakib_pi/pir") )
                    pir.setText( new String( message.getPayload()  )  );
                if ( topic.equals("sakib_pi/smoke") )
                    smoke.setText( new String( message.getPayload() ) ) ;


            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {

            }
        });
    }

    public void switch_on(View v) {

        try {
            client.publish("sakib_pi/switch", "1".getBytes(), 0, false);
        }
        catch (MqttException e) {
            e.printStackTrace();
        }
    }

    public void switch_off(View v) {

        try {
            client.publish("sakib_pi/switch", "0".getBytes(), 0, false);
        }
        catch (MqttException e) {
            e.printStackTrace();
        }
    }

    public void police(View v) {
        Intent callNow = new Intent(Intent.ACTION_CALL) ;
        callNow.setData( Uri.parse("tel:"+"01521223056")  ) ;
        startActivity(callNow) ;
    }

    public void fire_brigade(View v) {
        Intent callNow = new Intent(Intent.ACTION_CALL) ;
        callNow.setData( Uri.parse("tel:"+"999")  ) ;
        startActivity(callNow) ;
    }

    public void live_vid(View v) {
        Intent my_vid = new Intent(Intent.ACTION_VIEW, Uri.parse("http://192.168.192.1:8081") ) ;
        startActivity(my_vid) ;
    }
    public void message(View v){
        SmsManager smsManager = SmsManager.getDefault() ;
        smsManager.sendTextMessage("01521223056",null,"There is an Emergency !",null,null ) ;
    }


    private void setSubscribe(){
        try{
            client.subscribe("sakib_pi/hum", 0) ;
            client.subscribe("sakib_pi/temp", 0) ;
            client.subscribe("sakib_pi/pir", 0) ;
            client.subscribe("sakib_pi/smoke", 0) ;

        }
        catch(MqttException e){
            e.printStackTrace();
        }
    }


}

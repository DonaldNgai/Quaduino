package com.example.donald.quaduinoflightcontrol;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/**
 * An example full-screen activity that shows and hides the system UI (i.e.
 * status bar and navigation/system bar) with user interaction.
 */
public class FullscreenActivity extends AppCompatActivity {
    /**
     * Whether or not the system UI should be auto-hidden after
     * {@link #AUTO_HIDE_DELAY_MILLIS} milliseconds.
     */
    private static final boolean AUTO_HIDE = true;

    /**
     * If {@link #AUTO_HIDE} is set, the number of milliseconds to wait after
     * user interaction before hiding the system UI.
     */
    private static final int AUTO_HIDE_DELAY_MILLIS = 3000;

    /**
     * Some older devices needs a small delay between UI widget updates
     * and a change of the status and navigation bar.
     */
    private static final int UI_ANIMATION_DELAY = 300;

    private View mContentView;
    private View mControlsView;
    private boolean mVisible;
    private BluetoothAdapter mBluetoothAdapter;
    private Set<BluetoothDevice> pairedDevices;
    public static final String EXTRA_ADDRESS = "address";
    ListView devicelist;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_fullscreen);


        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        devicelist = (ListView)findViewById(R.id.bluetooth_list);
    }

    public void on(View v){
        if (!mBluetoothAdapter.isEnabled()) {
            Intent turnOn = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(turnOn, 0);
            Toast.makeText(getApplicationContext(), "Turned on", Toast.LENGTH_LONG).show();
        }
        else
        {
            Toast.makeText(getApplicationContext(),"Already on", Toast.LENGTH_LONG).show();
        }
    }

    public void findBluetooth(View v){
        List<String> list = new ArrayList<String>();

        if (!mBluetoothAdapter.isEnabled()) {
            int REQUEST_ENABLE_BT = 0;
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }

        pairedDevices = mBluetoothAdapter.getBondedDevices();
        // If there are paired devices
        if (pairedDevices.size() > 0) {
            // Loop through paired devices
            for (BluetoothDevice device : pairedDevices) {
                // Add the name and address to an array adapter to show in a ListView
                list.add(device.getName() + "\n" + device.getAddress());
            }
        }
        else
        {
            Toast.makeText(getApplicationContext(), "No Paired Bluetooth Devices Found.", Toast.LENGTH_LONG).show();
        }

        final ArrayAdapter<String> adapter = new ArrayAdapter<String>(this,android.R.layout.simple_list_item_1, list);
        devicelist.setAdapter(adapter);
        devicelist.setOnItemClickListener(myListClickListener); //Method called when the device from the list is clicked
    }

    private AdapterView.OnItemClickListener myListClickListener = new AdapterView.OnItemClickListener()
    {
        public void onItemClick (AdapterView av, View v, int arg2, long arg3)
        {
            // Get the device MAC address, the last 17 chars in the View
            String info = ((TextView) v).getText().toString();
            String address = info.substring(info.length() - 17);

            // Make an intent to start next activity.
            Intent i = new Intent(FullscreenActivity.this, Flight_Controller.class);
            //Change the activity.
            i.putExtra(EXTRA_ADDRESS, address); //this will be received at Flight Controller (class) Activity
            startActivityForResult(i,0);
        }
    };

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {

        if (requestCode == 0 && resultCode == RESULT_OK){
            try{
                double RP_P = data.getDoubleExtra("RPP", -1);
                double RP_I = data.getDoubleExtra("RPI", -1);
                double RP_D = data.getDoubleExtra("RPD", -1);
                double Y_P = data.getDoubleExtra("YawP", -1);
                double Y_I = data.getDoubleExtra("YawI", -1);
                double Y_D = data.getDoubleExtra("YawD", -1);

                StringBuilder sb = new StringBuilder(150);

                sb.append("Yaw P: " + Y_P + "\n" +
                        "Yaw I: " + Y_I + "\n" +
                        "Yaw D: " + Y_D + "\n" +
                        "RP P: " + RP_P + "\n" +
                        "RP I: " + RP_I + "\n" +
                        "RP D: " + RP_D + "\n");

                ((TextView) findViewById(R.id.PID_Results)).setText(sb.toString());
            }
            catch(Exception e){
                Toast.makeText(getApplicationContext(), "Failed to retrieve previous PID values.", Toast.LENGTH_LONG).show();
            }
        }
    }

}

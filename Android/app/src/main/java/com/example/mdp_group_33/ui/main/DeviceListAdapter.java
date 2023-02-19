package com.example.mdp_group_33.ui.main;

import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.TextView;

import com.example.mdp_group_33.R;

import java.util.ArrayList;

public class DeviceListAdapter extends ArrayAdapter<BluetoothDevice> {

    private final LayoutInflater myLayoutInflater;
    private final ArrayList<BluetoothDevice> BTDev;
    private final int myViewResourceId;

    public DeviceListAdapter(Context context, int ResourceId, ArrayList<BluetoothDevice> devices) {
        super(context, ResourceId, devices);
        this.BTDev = devices;
        myLayoutInflater = (LayoutInflater) context.getSystemService(Context.LAYOUT_INFLATER_SERVICE);
        myViewResourceId = ResourceId;
    }

    public View getView(int position, View view, ViewGroup parent) {
        view = myLayoutInflater.inflate(myViewResourceId, null);

        BluetoothDevice device = BTDev.get(position);

        if (device != null) {
            TextView deviceName = view.findViewById(R.id.deviceName);
            TextView deviceAddress = view.findViewById(R.id.deviceAddress);

            if (deviceName != null) {
                deviceName.setText(device.getName());
            }
            if (deviceAddress != null) {
                deviceAddress.setText(device.getAddress());
            }
        }

        return view;
    }
}
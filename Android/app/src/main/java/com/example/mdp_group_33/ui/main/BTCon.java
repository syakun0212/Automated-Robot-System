package com.example.mdp_group_33.ui.main;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.util.Log;
import android.widget.Toast;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.Charset;
import java.util.UUID;

public class BTCon {

    private static final String myAppName = "MDP_Group_33";
    private static final UUID myUUID = UUID.fromString("fe4a31ef-98ec-459c-bb5e-d67c67d580f1");

    private final BluetoothAdapter myBluetoothAdapter;
    Context mContext;

    private AcceptThread AcceptThread;

    private ConnectThread myConnectThread;
    private BluetoothDevice myDevice;
    private UUID myDeviceUUID;
    ProgressDialog myProgressDialog;
    Intent connectionStatus;

    public static boolean BluetoothConnectionStatus=false;
    private static ConnectedThread myConnectedThread;

    public BTCon(Context context) {
        this.myBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        this.mContext = context;
        startAcceptThread();
    }

    private class AcceptThread extends Thread{
        private final BluetoothServerSocket ServerSocket;

        public AcceptThread() {
            BluetoothServerSocket temp = null;

            try {
                temp = myBluetoothAdapter.listenUsingInsecureRfcommWithServiceRecord(myAppName, myUUID);
            }catch(IOException e){
            }
            ServerSocket = temp;
        }
        public void run(){
            BluetoothSocket socket =null;
            try {
                socket = ServerSocket.accept();
            }catch (IOException e){
            }
            if(socket!=null){
                connected(socket, socket.getRemoteDevice());
            }
        }
        public void cancel(){
            try{
                ServerSocket.close();
            } catch(IOException e){
            }
        }
    }

    private class ConnectThread extends Thread{
        private BluetoothSocket mySocket;

        public ConnectThread(BluetoothDevice device, UUID u){
            myDevice = device;
            myDeviceUUID = u;
        }

        public void run(){
            BluetoothSocket temp = null;

            try {
                temp = myDevice.createRfcommSocketToServiceRecord(myDeviceUUID);
            } catch (IOException e) {
            }
            mySocket= temp;
            myBluetoothAdapter.cancelDiscovery();

            try {
                mySocket.connect();


                connected(mySocket,myDevice);

            } catch (IOException e) {
                try {
                    mySocket.close();
                } catch (IOException e1) {
                }
                try {
                    BTActivity mBluetoothPopUpActivity = (BTActivity) mContext;
                    mBluetoothPopUpActivity.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            Toast.makeText(mContext, "Failed to connect to the Device.", Toast.LENGTH_LONG).show();
                        }
                    });
                } catch (Exception z) {
                    z.printStackTrace();
                }

            }
            try {
                myProgressDialog.dismiss();
            } catch(NullPointerException e){
                e.printStackTrace();
            }
        }

        public void cancel(){
            try{
                mySocket.close();
            } catch(IOException e){
            }
        }
    }

    public synchronized void startAcceptThread(){

        if(myConnectThread!=null){
            myConnectThread.cancel();
            myConnectThread=null;
        }
        if(AcceptThread == null){
            AcceptThread = new AcceptThread();
            AcceptThread.start();
        }
    }

    public void startClientThread(BluetoothDevice device, UUID uuid){

        try {
            myProgressDialog = ProgressDialog.show(mContext, "Connecting Bluetooth", "Please Wait...", true);
        } catch (Exception e) {
        }


        myConnectThread = new ConnectThread(device, uuid);
        myConnectThread.start();
    }

    private class ConnectedThread extends Thread{
        private final BluetoothSocket mSocket;
        private final InputStream inStream;
        private final OutputStream outStream;

        public ConnectedThread(BluetoothSocket socket) {

            connectionStatus = new Intent("ConnectionStatus");
            connectionStatus.putExtra("Status", "connected");
            connectionStatus.putExtra("Device", myDevice);
            LocalBroadcastManager.getInstance(mContext).sendBroadcast(connectionStatus);
            BluetoothConnectionStatus = true;

            this.mSocket = socket;
            InputStream tempIn = null;
            OutputStream tempOut = null;

            try {
                tempIn = mSocket.getInputStream();
                tempOut = mSocket.getOutputStream();
            } catch (IOException e) {
                e.printStackTrace();
            }

            inStream = tempIn;
            outStream = tempOut;
        }

        public void run(){
            byte[] buffer = new byte[1024];
            int bytes;

            while(true){
                try {
                    bytes = inStream.read(buffer);
                    String incomingmessage = new String(buffer, 0, bytes);

                    Intent incomingMessageIntent = new Intent("incomingMessage");
                    incomingMessageIntent.putExtra("receivedMessage", incomingmessage);

                    LocalBroadcastManager.getInstance(mContext).sendBroadcast(incomingMessageIntent);
                } catch (IOException e) {

                    connectionStatus = new Intent("ConnectionStatus");
                    connectionStatus.putExtra("Status", "disconnected");
                    connectionStatus.putExtra("Device", myDevice);
                    LocalBroadcastManager.getInstance(mContext).sendBroadcast(connectionStatus);
                    BluetoothConnectionStatus = false;

                    break;
                }
            }
        }

        public void write(byte[] bytes){
            String text = new String(bytes, Charset.defaultCharset());
            try {
                outStream.write(bytes);
            } catch (IOException e) {
            }
        }

        public void cancel(){
            try{
                mSocket.close();
            } catch(IOException e){
            }
        }
    }

    private void connected(BluetoothSocket mSocket, BluetoothDevice device) {
        myDevice =  device;
        if (AcceptThread != null) {
            AcceptThread.cancel();
            AcceptThread = null;
        }

        myConnectedThread = new ConnectedThread(mSocket);
        myConnectedThread.start();
    }

    public static void write(byte[] out){
        myConnectedThread.write(out);
    }
}

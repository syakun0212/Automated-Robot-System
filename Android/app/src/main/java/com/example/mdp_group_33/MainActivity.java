package com.example.mdp_group_33;

import android.app.Activity;
import android.app.Dialog;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.ClipData;
import android.content.ClipDescription;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.DragEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import androidx.viewpager.widget.ViewPager;

import com.example.mdp_group_33.ui.main.BTCon;
import com.example.mdp_group_33.ui.main.BTActivity;
import com.example.mdp_group_33.ui.main.CommsFragment;
import com.example.mdp_group_33.ui.main.MapControlFragment;
import com.example.mdp_group_33.ui.main.Map;
import com.example.mdp_group_33.ui.main.SectionsPagerAdapter;
import com.google.android.material.tabs.TabLayout;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    private static SharedPreferences sp;
    private static SharedPreferences.Editor editor;
    private static Context context;
    private static Map Map;

    static TextView xAxisTextView, yAxisTextView, directionAxisTextView;
    static TextView robotStatusTextView;

    boolean obs_goneFlag = false;
    boolean robot_goneFlag = false;

    ImageView obsPlain, obsTop, obsBtm, obsRight, obsLeft, Up, Down;
    Button cancelBtn, confirmBtn;

    BluetoothDevice mBTDevice;
    private static UUID myUUID;
    ProgressDialog myDialog;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        // Initialization
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        SectionsPagerAdapter sectionsPagerAdapter = new SectionsPagerAdapter(this, getSupportFragmentManager());
        ViewPager viewPager = findViewById(R.id.view_pager);
        viewPager.setAdapter(sectionsPagerAdapter);
        viewPager.setOffscreenPageLimit(9999);
        TabLayout tabs = findViewById(R.id.tabs);
        tabs.setupWithViewPager(viewPager);
        LocalBroadcastManager.getInstance(this).registerReceiver(messageReceiver, new IntentFilter("incomingMessage"));

        // Set up sharedPreferences
        MainActivity.context = getApplicationContext();
        sharedPreferences();
        editor.putString("message", "");
        editor.putString("direction", "None");
        editor.putString("connStatus", "Disconnected");
        editor.commit();

        // Toolbar
        Button bluetoothButton = findViewById(R.id.bluetoothButton);
        bluetoothButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent popup = new Intent(MainActivity.this, BTActivity.class);
                startActivity(popup);
            }
        });


        // Map
        Map = new Map(this);
        Map = findViewById(R.id.mapView);
        xAxisTextView = findViewById(R.id.xAxisTextView);
        yAxisTextView = findViewById(R.id.yAxisTextView);
        directionAxisTextView = findViewById(R.id.directionAxisTextView);

        // Robot Status
        robotStatusTextView = findViewById(R.id.robotStatusTextView);

        myDialog = new ProgressDialog(MainActivity.this);
        myDialog.setMessage("Waiting for other device to reconnect...");
        myDialog.setCancelable(false);
        myDialog.setButton(DialogInterface.BUTTON_NEGATIVE, "Cancel", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });

    }



    public static Map getGridMap() {
        return Map;
    }

    public static TextView getRobotStatusTextView() {  return robotStatusTextView; }

    public static void sharedPreferences() {
        sp = MainActivity.getSharedPreferences(MainActivity.context);
        editor = sp.edit();
    }

    public static void printMessage(String message) {
        editor = sp.edit();

        if (BTCon.BluetoothConnectionStatus == true) {
            byte[] bytes = message.getBytes(Charset.defaultCharset());
            BTCon.write(bytes);
        }
        editor.putString("message", CommsFragment.getMessageReceived().getText() + "\n" + message);
        editor.commit();
        refreshMessageReceived();
    }

    public static void printMessage(String name, int x, int y) throws JSONException {
        sharedPreferences();

        JSONObject jsonObject = new JSONObject();
        String message;

        switch(name) {
//            case "starting":
            case "waypoint":
                jsonObject.put(name, name);
                jsonObject.put("x", x);
                jsonObject.put("y", y);
                message = name + " (" + x + "," + y + ")";
                break;
            default:
                message = "Unexpected default for printMessage: " + name;
                break;
        }
        editor.putString("message", CommsFragment.getMessageReceived().getText() + "\n" + message);
        editor.commit();
        if (BTCon.BluetoothConnectionStatus == true) {
            byte[] bytes = message.getBytes(Charset.defaultCharset());
            BTCon.write(bytes);
        }
    }

    public static void refreshMessageReceived() {
        CommsFragment.getMessageReceived().setText(sp.getString("message", ""));
    }

    public static void refreshLabel() {
        xAxisTextView.setText(String.valueOf(Map.getCurCoord()[0]-1));
        yAxisTextView.setText(String.valueOf(Map.getCurCoord()[1]-1));
        directionAxisTextView.setText(sp.getString("direction",""));
    }

    public static void receiveMessage(String message) {
        sharedPreferences();
        editor.putString("message", sp.getString("message", "") + "\n" + message);
        editor.commit();
    }

    private static SharedPreferences getSharedPreferences(Context context) {
        return context.getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
    }

    private final BroadcastReceiver mBroadcastReceiver5 = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            BluetoothDevice mDevice = intent.getParcelableExtra("Device");
            String status = intent.getStringExtra("Status");
            sharedPreferences();

            if(status.equals("connected")){
                try {
                    myDialog.dismiss();
                } catch(NullPointerException e){
                    e.printStackTrace();
                }

                Toast.makeText(MainActivity.this, "Device now connected to "+mDevice.getName(), Toast.LENGTH_LONG).show();
                editor.putString("connStatus", "Connected to " + mDevice.getName());

            }
            else if(status.equals("disconnected")){
                Toast.makeText(MainActivity.this, "Disconnected from "+mDevice.getName(), Toast.LENGTH_LONG).show();
                editor.putString("connStatus", "Disconnected");
                myDialog.show();
            }
            editor.commit();
        }
    };

    BroadcastReceiver messageReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String message = intent.getStringExtra("receivedMessage");
            if (message.startsWith("ROBOT")){
                String[] msg = message.split(",");
                boolean setflag = true;
                ArrayList<int[]> ObsCoord = Map.getObsCoord();
                int ObsListSize = Map.getArraySize();
                for (int size = 0; size< ObsListSize; size++) {
                    int[] temp = ObsCoord.get(size);
                    if ((Integer.parseInt(msg[1]) + 1 == temp[0] || Integer.parseInt(msg[1]) - 1 == temp[0] || Integer.parseInt(msg[1]) == temp[0]) &&
                            (Integer.parseInt(msg[2]) + 1 == temp[1] || Integer.parseInt(msg[2]) - 1 == temp[1] || Integer.parseInt(msg[2]) == temp[1]))
                        setflag = false;
                }
                if (Integer.parseInt(msg[1]) <2 || Integer.parseInt(msg[1]) >19 || Integer.parseInt(msg[2]) <2 || Integer.parseInt(msg[2])>19)
                    setflag = false;
                if (MapControlFragment.getRobotStatus() && setflag)
                    Map.flyRobot(Integer.parseInt(msg[1]), Integer.parseInt(msg[2]), msg[3]);
            }
            else if (message.startsWith("TARGET")){
                String[] msg = message.split(",");
                int setArray = 1;
                ArrayList<int[]> ObsCoord = Map.getObsCoord();
                int[] templist = ObsCoord.get(Integer.parseInt(msg[1])-1);

                Map.setObstacle(templist[1], templist[0], Integer.parseInt(msg[1])-1,templist[3], Integer.parseInt(msg[2]), 0, setArray);
            }
            else if (message.startsWith("STATUS")){
                String[] msg = message.split(",");
                Map.printRobotStatus(msg[1]);
            }

            try {
                if (message.length() > 8 && message.startsWith("image", 2)) {
                    JSONObject jsonObject = new JSONObject(message);
                    JSONArray jsonArray = jsonObject.getJSONArray("image");
                    Map.drawImageNumberCell(jsonArray.getInt(0),jsonArray.getInt(1),jsonArray.getInt(2));
                }
            } catch (JSONException e) {
            }

            sharedPreferences();
            String receivedText = sp.getString("message", "") + "\n" + message;
            editor.putString("message", receivedText);
            editor.commit();
            refreshMessageReceived();
        }
    };

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data){
        super.onActivityResult(requestCode, resultCode, data);

        switch (requestCode){
            case 1:
                if(resultCode == Activity.RESULT_OK){
                    mBTDevice = data.getExtras().getParcelable("mBTDevice");
                    myUUID = (UUID) data.getSerializableExtra("myUUID");
                }
        }
    }

    @Override
    protected void onDestroy(){
        super.onDestroy();
        try{
            LocalBroadcastManager.getInstance(this).unregisterReceiver(messageReceiver);
            LocalBroadcastManager.getInstance(this).unregisterReceiver(mBroadcastReceiver5);
        } catch(IllegalArgumentException e){
            e.printStackTrace();
        }
    }

    @Override
    protected void onPause(){
        super.onPause();
        try{
            LocalBroadcastManager.getInstance(this).unregisterReceiver(mBroadcastReceiver5);
        } catch(IllegalArgumentException e){
            e.printStackTrace();
        }
    }

    @Override
    protected void onResume(){
        super.onResume();
        try{
            IntentFilter filter2 = new IntentFilter("ConnectionStatus");
            LocalBroadcastManager.getInstance(this).registerReceiver(mBroadcastReceiver5, filter2);
        } catch(IllegalArgumentException e){
            e.printStackTrace();
        }
    }

    @Override
    public void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public boolean onTouchEvent(MotionEvent event) {
        int row = Map.convertRow((int) (event.getY() / com.example.mdp_group_33.ui.main.Map.cellSize))+4;
        int column = (int) (event.getX() / com.example.mdp_group_33.ui.main.Map.cellSize)-1;

        if (MapControlFragment.getEditMapStatus()){
            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                    if (MapControlFragment.getSetStatus().equals("Set robot")){
                        if (MapControlFragment.getRobotStatus()) {
                            int robotcolumn = Integer.parseInt((String) xAxisTextView.getText()) + 1;
                            int robotrow = Integer.parseInt((String) yAxisTextView.getText()) + 1;

                            if (row > robotrow - 3 && row <= robotrow && column > robotcolumn - 3 && column <= robotcolumn) {
                                robot_goneFlag=false;
                                robot_handler.postDelayed(robot_mLongPressed, 500);
                            }
                        }
                    }
                    else{
                        ArrayList<int[]> ObsCoord = Map.getObsCoord();
                        int ObsListSize = Map.getArraySize();
                        for (int size = 0; size< ObsListSize; size++){
                            int[] temp = ObsCoord.get(size);
                            Log.d("Loop", temp[1]+"_"+temp[0]+"   "+row+"_"+column);
                            if (row == temp[1] && column == temp[0]){
                                temp[5] = 1;
                                Map.setArrayVar(size, temp);
                                MapControlFragment.setBtnMode(temp[3]+1);
                                obs_goneFlag=false;
                                obs_handler.postDelayed(obs_mLongPressed,500);
                            }
                        }
                    }
                    break;

                case MotionEvent.ACTION_MOVE:
                    break;

                case MotionEvent.ACTION_UP:
                    robot_handler.removeCallbacks(robot_mLongPressed);
                    obs_handler.removeCallbacks(obs_mLongPressed);

                    if (!MapControlFragment.getSetStatus().equals("Set robot") && !obs_goneFlag){
                        ArrayList<int[]> ObsCoord = Map.getObsCoord();
                        int ObsListSize = Map.getArraySize();
                        for (int size = 0; size< ObsListSize; size++){
                            int[] temp = ObsCoord.get(size);
                            if (temp[5] == 1){
                                ChangeObsDialog(size, temp);

                            }
                        }
                    }
                    break;

                default:
                    break;
            }
            return true;
        }
        else return false;
    }

    final Handler robot_handler = new Handler();
    Runnable robot_mLongPressed = new Runnable() {
        public void run() {
            robot_goneFlag = true;
            ImageView tempImage = MapControlFragment.RobotImage();
            ClipData.Item item = new ClipData.Item((CharSequence) tempImage.getTag());
            String[] mimeTypes = {ClipDescription.MIMETYPE_TEXT_PLAIN};
            ClipData data = new ClipData(tempImage.getTag().toString(), mimeTypes, item);
            View.DragShadowBuilder dragshadow = new View.DragShadowBuilder(tempImage);
            tempImage.startDrag(data, dragshadow, tempImage, 0);
        }
    };

    final Handler obs_handler = new Handler();
    Runnable obs_mLongPressed = new Runnable() {
        public void run() {
            obs_goneFlag = true;
            ImageView tempImage = MapControlFragment.setMode();
            ClipData.Item item = new ClipData.Item((CharSequence) tempImage.getTag());
            String[] mimeTypes = {ClipDescription.MIMETYPE_TEXT_PLAIN};
            ClipData data = new ClipData(tempImage.getTag().toString(), mimeTypes, item);
            View.DragShadowBuilder dragshadow = new View.DragShadowBuilder(tempImage);
            tempImage.startDrag(data, dragshadow, tempImage, 0);
        }
    };

    public void ChangeObsDialog(int index, int[] list){
        Dialog dialogBuilder = new Dialog (this);
        dialogBuilder.setContentView(R.layout.obs_pop);
        dialogBuilder.getWindow().setBackgroundDrawable(new ColorDrawable(Color.WHITE));
        dialogBuilder.setCancelable(false);
        dialogBuilder.show();

        obsPlain = dialogBuilder.findViewById(R.id.obsPlain);
        obsTop = dialogBuilder.findViewById(R.id.obsTop);
        obsBtm = dialogBuilder.findViewById(R.id.obsBtm);
        obsRight = dialogBuilder.findViewById(R.id.obsRight);
        obsLeft = dialogBuilder.findViewById(R.id.obsLeft);
        cancelBtn = dialogBuilder.findViewById(R.id.CancalBtn);
        confirmBtn = dialogBuilder.findViewById(R.id.ConfirmBtn);
        Up = dialogBuilder.findViewById(R.id.UpBtn);
        Down = dialogBuilder.findViewById(R.id.downBtn);

        Up.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v){
                if (obsPlain.getVisibility() == View.VISIBLE){
                    obsPlain.setVisibility(View.INVISIBLE);
                    obsTop.setVisibility(View.VISIBLE);
                }
                else if (obsTop.getVisibility() == View.VISIBLE){
                    obsTop.setVisibility(View.INVISIBLE);
                    obsRight.setVisibility(View.VISIBLE);
                }
                else if (obsRight.getVisibility() == View.VISIBLE){
                    obsRight.setVisibility(View.INVISIBLE);
                    obsBtm.setVisibility(View.VISIBLE);
                }
                else if (obsBtm.getVisibility() == View.VISIBLE){
                    obsBtm.setVisibility(View.INVISIBLE);
                    obsLeft.setVisibility(View.VISIBLE);
                }
                else {
                    obsLeft.setVisibility(View.INVISIBLE);
                    obsPlain.setVisibility(View.VISIBLE);
                }
            }
        });

        Down.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v){
                if (obsPlain.getVisibility() == View.VISIBLE){
                    obsPlain.setVisibility(View.INVISIBLE);
                    obsLeft.setVisibility(View.VISIBLE);
                }
                else if (obsTop.getVisibility() == View.VISIBLE){
                    obsTop.setVisibility(View.INVISIBLE);
                    obsPlain.setVisibility(View.VISIBLE);
                }
                else if (obsRight.getVisibility() == View.VISIBLE){
                    obsRight.setVisibility(View.INVISIBLE);
                    obsTop.setVisibility(View.VISIBLE);
                }
                else if (obsBtm.getVisibility() == View.VISIBLE){
                    obsBtm.setVisibility(View.INVISIBLE);
                    obsRight.setVisibility(View.VISIBLE);
                }
                else {
                    obsLeft.setVisibility(View.INVISIBLE);
                    obsBtm.setVisibility(View.VISIBLE);
                }
            }
        });

        cancelBtn.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v){
                Map.setObstacle(list[1] , list[0] , index, list[3] , list[4] ,0, 1);
                dialogBuilder.cancel();
            }
        });

        confirmBtn.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v){
                if (obsPlain.getVisibility() == View.VISIBLE) list[3] = 0;
                else if (obsTop.getVisibility() == View.VISIBLE) list[3] = 1;
                else if (obsRight.getVisibility() == View.VISIBLE) list[3] = 2;
                else if (obsBtm.getVisibility() == View.VISIBLE) list[3] = 3;
                else list[3] = 4;

                Map.setObstacle(list[1] , list[0] , index, list[3] , list[4] ,0, 1);
                dialogBuilder.cancel();
            }
        });
    }
}
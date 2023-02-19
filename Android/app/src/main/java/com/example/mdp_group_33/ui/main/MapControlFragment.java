package com.example.mdp_group_33.ui.main;

import android.content.ClipData;
import android.content.ClipDescription;
import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.util.Log;
import android.view.DragEvent;
import android.view.Gravity;
import android.view.LayoutInflater;
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

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProviders;

import com.example.mdp_group_33.MainActivity;
import com.example.mdp_group_33.R;

import java.nio.charset.Charset;
import java.util.ArrayList;

public class MapControlFragment extends Fragment implements View.OnLongClickListener, View.OnDragListener {

    private static final String ARG_SECTION_NUMBER = "section_number";
    private PageViewModel pageViewModel;
    private static int btnMode = 1;
    public static boolean robotSet = false;
    private static ImageView obstacleImage, obsImagetop, obsImagebtm, obsImageleft, obsImageright;

    ImageButton fwdBtn, rightBtn, bckBtn, leftBtn;
    TextView robotStatusTextView;
    static Button calibrateButton, startFastestButton, startImageButton, rstBtn;
    private static Map Map;
    static Switch Edit_Map;

    ImageButton incBtn,decBtn;
    static ImageView robotImage;
    static ToggleButton setMap;


    // Fragment Constructor
    public static MapControlFragment newInstance(int index) {
        MapControlFragment fragment = new MapControlFragment();
        Bundle bundle = new Bundle();
        bundle.putInt(ARG_SECTION_NUMBER, index);
        fragment.setArguments(bundle);
        return fragment;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        pageViewModel = ViewModelProviders.of(this).get(PageViewModel.class);
        int index = 1;
        if (getArguments() != null) {
            index = getArguments().getInt(ARG_SECTION_NUMBER);
        }
        pageViewModel.setIndex(index);
    }

    @Override
    public View onCreateView(
            @NonNull LayoutInflater inflater, ViewGroup container,
            Bundle savedInstanceState) {
        View root = inflater.inflate(R.layout.activity_mapcontrol, container, false);



        fwdBtn = root.findViewById(R.id.forwardImageBtn);
        rightBtn = root.findViewById(R.id.rightImageBtn);
        bckBtn = root.findViewById(R.id.backImageBtn);
        leftBtn = root.findViewById(R.id.leftImageBtn);
        calibrateButton = root.findViewById(R.id.calibrateButton);
        startFastestButton = root.findViewById(R.id.startFastestBtn);
        startImageButton = root.findViewById(R.id.startImageBtn);

        robotStatusTextView = MainActivity.getRobotStatusTextView();

        Map = MainActivity.getGridMap();

        robotImage = root.findViewById(R.id.robotImage);
        robotImage.setTag("Robot");
        robotImage.setOnLongClickListener(this);
        robotImage.setOnDragListener(this);

        obstacleImage = root.findViewById(R.id.obsImage);
        obstacleImage.setTag("Obs");
        obstacleImage.setOnLongClickListener(this);
        obstacleImage.setOnDragListener(this);

        obsImagetop = root.findViewById(R.id.obsImagetop);
        obsImagetop.setTag("ObsTop");
        obsImagetop.setOnLongClickListener(this);
        obsImagetop.setOnDragListener(this);


        obsImageright = root.findViewById(R.id.obsImageright);
        obsImageright.setTag("ObsRight");
        obsImageright.setOnLongClickListener(this);
        obsImageright.setOnDragListener(this);


        obsImagebtm = root.findViewById(R.id.obsImagebtm);
        obsImagebtm.setTag("ObsBtm");
        obsImagebtm.setOnLongClickListener(this);
        obsImagebtm.setOnDragListener(this);

        obsImageleft = root.findViewById(R.id.obsImageleft);
        obsImageleft.setTag("ObsLeft");
        obsImageleft.setOnLongClickListener(this);
        obsImageleft.setOnDragListener(this);

        rstBtn = root.findViewById(R.id.rstBtn);
        rstBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Map.resetMap();
            }
        });

        setMap = root.findViewById(R.id.setBtn);
        setMap.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (setMap.getText().equals("Set robot")) {
                    robotImage.setVisibility(View.VISIBLE);
                    obstacleImage.setVisibility(View.INVISIBLE);
                    obsImagetop.setVisibility(View.INVISIBLE);
                    obsImageright.setVisibility(View.INVISIBLE);
                    obsImagebtm.setVisibility(View.INVISIBLE);
                    obsImageleft.setVisibility(View.INVISIBLE);
                    incBtn.setVisibility(View.INVISIBLE);
                    decBtn.setVisibility(View.INVISIBLE);
                }
                else{
                    robotImage.setVisibility(View.INVISIBLE);
                    btnMode = 1;
                    setMode();
                    incBtn.setVisibility(View.VISIBLE);
                    decBtn.setVisibility(View.VISIBLE);
                }
            }
        });

        Edit_Map = root.findViewById(R.id.editMapBtn);
        Edit_Map.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Edit_Map.setChecked(Edit_Map.isChecked());

            }
        });

        incBtn = root.findViewById(R.id.incBtn);
        incBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (btnMode < 5) btnMode+=1;
                else btnMode = 1;
                setMode();
            }
        });

        decBtn = root.findViewById(R.id.decBtn);
        decBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (btnMode > 1) btnMode-=1;
                else btnMode = 5;
                setMode();
            }
        });

        fwdBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (Map.getCanDrawRobot()) {
                    Map.moveRobot("forward");
                    MainActivity.refreshLabel();
                    if (!Map.getValidPosition())
                        updateStatus("Unable to move forward");
                    MainActivity.printMessage("Forward");
                }
                else
                    updateStatus("Indicate starting point");
            }
        });

        rightBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (Map.getCanDrawRobot()) {
                    Map.moveRobot("right");
                    MainActivity.refreshLabel();
                    MainActivity.printMessage("Right");
                }
                else
                    updateStatus("Indicate starting point");
            }
        });

        bckBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (Map.getCanDrawRobot()) {
                    Map.moveRobot("back");
                    MainActivity.refreshLabel();
                    if (!Map.getValidPosition())
                        updateStatus("Unable to move backward");
                    MainActivity.printMessage("Back");
                }
                else
                    updateStatus("Indicate starting point");
            }
        });

        leftBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (Map.getCanDrawRobot()) {
                    Map.moveRobot("left");
                    MainActivity.refreshLabel();
                    MainActivity.printMessage("Left");
                }
                else
                    updateStatus("Indicate starting point");
            }
        });

        calibrateButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (BTCon.BluetoothConnectionStatus == true) {
                    int totalObs = Map.getArraySize();
                    String sentText = "Obstacle count," + totalObs;

                    ArrayList<int[]> ObsCoord = Map.getObsCoord();
                    for (int x = 0; x < totalObs; x++){
                        int[] temp = ObsCoord.get(x);
                        String dir = "0";
                        switch (temp[3]){
                            case(1):
                                dir = "N";
                                break;
                            case(2):
                                dir = "E";
                                break;
                            case(3):
                                dir = "S";
                                break;
                            case(4):
                                dir = "W";
                                break;
                            default:
                                break;
                        }

                        sentText = sentText + ",Obstacle,"+ (temp[2]+1) + ","+(temp[0]-1)+","+(temp[1]-1)+","+dir;
                    }
                    byte[] bytes = sentText.getBytes(Charset.defaultCharset());
                    BTCon.write(bytes);
                }
            }
        });

        startFastestButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (BTCon.BluetoothConnectionStatus == true) {
                    String sentText = "Start,Speed";
                    byte[] bytes = sentText.getBytes(Charset.defaultCharset());
                    BTCon.write(bytes);
                }
            }
        });

        startImageButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (BTCon.BluetoothConnectionStatus == true) {
                    String sentText = "Start,Image";
                    byte[] bytes = sentText.getBytes(Charset.defaultCharset());
                    BTCon.write(bytes);
                }
            }
        });

        return root;
    }

    public static ImageView setMode(){
        switch (btnMode){
            case(1):
                obstacleImage.setVisibility(View.VISIBLE);
                obsImagetop.setVisibility(View.INVISIBLE);
                obsImageright.setVisibility(View.INVISIBLE);
                obsImagebtm.setVisibility(View.INVISIBLE);
                obsImageleft.setVisibility(View.INVISIBLE);
                return obstacleImage;

            case(2):
                obstacleImage.setVisibility(View.INVISIBLE);
                obsImagetop.setVisibility(View.VISIBLE);
                obsImageright.setVisibility(View.INVISIBLE);
                obsImagebtm.setVisibility(View.INVISIBLE);
                obsImageleft.setVisibility(View.INVISIBLE);
                return obsImagetop;

            case(3):
                obstacleImage.setVisibility(View.INVISIBLE);
                obsImagetop.setVisibility(View.INVISIBLE);
                obsImageright.setVisibility(View.VISIBLE);
                obsImagebtm.setVisibility(View.INVISIBLE);
                obsImageleft.setVisibility(View.INVISIBLE);
                return obsImageright;

            case(4):
                obstacleImage.setVisibility(View.INVISIBLE);
                obsImagetop.setVisibility(View.INVISIBLE);
                obsImageright.setVisibility(View.INVISIBLE);
                obsImagebtm.setVisibility(View.VISIBLE);
                obsImageleft.setVisibility(View.INVISIBLE);
                return obsImagebtm;

            case(5):
                obstacleImage.setVisibility(View.INVISIBLE);
                obsImagetop.setVisibility(View.INVISIBLE);
                obsImageright.setVisibility(View.INVISIBLE);
                obsImagebtm.setVisibility(View.INVISIBLE);
                obsImageleft.setVisibility(View.VISIBLE);
                return obsImageleft;

        }
        return obstacleImage;
    }

    @Override
    public boolean onLongClick(View v) {
        Log.d("on long click", v.getTag().toString());
        if (Edit_Map.isChecked()){
            if (v.getTag().toString().equals("Robot")) {
                if (robotImage.getVisibility() == View.VISIBLE) {
                    ClipData.Item item = new ClipData.Item((CharSequence) v.getTag());
                    String[] mimeTypes = {ClipDescription.MIMETYPE_TEXT_PLAIN};
                    ClipData data = new ClipData(v.getTag().toString(), mimeTypes, item);
                    View.DragShadowBuilder dragshadow = new View.DragShadowBuilder(v);
                    v.startDrag(data, dragshadow, v, 0);
                }
                return true;
            }
            else{
                ClipData.Item item = new ClipData.Item((CharSequence) v.getTag());
                String[] mimeTypes = {ClipDescription.MIMETYPE_TEXT_PLAIN};
                ClipData data = new ClipData(v.getTag().toString(), mimeTypes, item);
                View.DragShadowBuilder dragshadow = new View.DragShadowBuilder(v);
                v.startDrag(data, dragshadow, v, 0);
            }
        }
        return false;
    }

    @Override
    public boolean onDrag(View v, DragEvent event) {
        int action = event.getAction();

        int column = (int) (event.getX() / com.example.mdp_group_33.ui.main.Map.cellSize);
        int row = Map.convertRow((int) (event.getY() / com.example.mdp_group_33.ui.main.Map.cellSize));

        Log.d("DRAGGING NOW", v.getTag().toString());

        switch (action) {

            case DragEvent.ACTION_DRAG_STARTED:
                return event.getClipDescription().hasMimeType(ClipDescription.MIMETYPE_TEXT_PLAIN);

            case DragEvent.ACTION_DRAG_ENTERED:
                return true;

            case DragEvent.ACTION_DRAG_LOCATION:
                return true;

            case DragEvent.ACTION_DRAG_EXITED:
                return true;

            case DragEvent.ACTION_DROP:

                ClipData.Item item = event.getClipData().getItemAt(0);
                String dragData = item.getText().toString();
                v.getBackground().clearColorFilter();
                v.invalidate();
                View vw = (View) event.getLocalState();
                ViewGroup owner = (ViewGroup) vw.getParent();
                owner.removeView(vw);
                LinearLayout container = (LinearLayout) v;
                container.addView(vw);
                vw.setVisibility(View.VISIBLE);
                return true;


            case DragEvent.ACTION_DRAG_ENDED:
                if (v.getTag().toString().equals("Robot")) {
                    if ((column <= 20 && column >= 3) && (row >= -2 && row <= 15)) {
                        Map.setRobot(row +4, column-1);
                        setRobotStatus(true);
                    } else {
                        Map.removerobot();
                        setRobotStatus(false);
                    }
                }

                else{
                    int setArray = 0;
                    int temp = Map.getArraySize();
                    ArrayList<int[]> ObsCoord = Map.getObsCoord();
                    int ObsListSize = Map.getArraySize();


                    if ((column <= 21 && column >= 2) && (row >= -3 && row <= 16)) {

                        for (int size = 0; size< ObsListSize; size++) {
                            int[] templist = ObsCoord.get(size);
                            if (templist[5] == 1) {
                                temp = templist[2];
                                setArray = 1;
                            }
                        }
                        switch (v.getTag().toString()){
                            case("Obs"):
                                Map.setObstacle(row + 4, column - 1, temp,0, 0, 0, setArray);
                                break;
                            case("ObsTop"):
                                Map.setObstacle(row + 4, column - 1, temp,1, 0,0, setArray);
                                break;
                            case("ObsRight"):
                                Map.setObstacle(row + 4, column - 1, temp,2, 0,0, setArray);
                                break;
                            case("ObsBtm"):
                                Map.setObstacle(row + 4, column - 1, temp,3, 0,0, setArray);
                                break;
                            case("ObsLeft"):
                                Map.setObstacle(row + 4, column - 1, temp,4, 0,0, setArray);
                                break;
                        }
                    }
                    else{
                        if (temp != 0){
                            for (int size = 0; size< ObsListSize; size++) {
                                int[] templist = ObsCoord.get(size);
                                if (templist[5] == 1) {
                                    temp = templist[2];
                                    setArray = 2;
                                    Map.setObstacle(0,0,temp,0,0,0,setArray);
                                    break;
                                }
                            }
                        }

                    }
                    return true;


                }
                return true;

            default:
                break;
        }
        return false;
    }



    private void updateStatus(String message) {
        Toast toast = Toast.makeText(getContext(), message, Toast.LENGTH_SHORT);
        toast.setGravity(Gravity.TOP,0, 0);
        toast.show();
    }

    public static boolean getEditMapStatus(){
        return Edit_Map.isChecked();
    }

    public static void setBtnMode(int mode){
        btnMode = mode;
    }

    public static String getSetStatus(){
        return (String) setMap.getText();
    }

    public static void setRobotStatus(boolean status){
        robotSet = status;
    }

    public static boolean getRobotStatus(){
        return robotSet;
    }

    public static ImageView RobotImage() {
        return robotImage;
    }
}

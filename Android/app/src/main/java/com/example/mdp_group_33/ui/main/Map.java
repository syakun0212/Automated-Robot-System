package com.example.mdp_group_33.ui.main;

import android.app.Activity;
import android.content.ClipData;
import android.content.ClipDescription;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.util.Log;
import android.view.DragEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import androidx.core.view.MotionEventCompat;

import com.example.mdp_group_33.MainActivity;
import com.example.mdp_group_33.R;

import org.jetbrains.annotations.Nullable;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.math.BigInteger;
import java.util.ArrayList;
import java.util.List;

public class Map extends View{

    public Map(Context c) {
        super(c);
        initMap();
    }

    SharedPreferences sp;

    private final Paint blackPaint = new Paint();
    private final Paint whitePaint = new Paint();
    private final Paint whiteObs = new Paint();
    private final Paint obstacleColor = new Paint();
    private final Paint robotColor = new Paint();
    private final Paint startColor = new Paint();
    private final Paint ImageColor = new Paint();
    private final Paint unexploredColor = new Paint();
    private final Paint exploredColor = new Paint();

    private static String robotDirection = "None";
    private static int[] startCoord = new int[]{-1, -1};
    private static int[] curCoord = new int[]{-1, -1};
    private static int[] oldCoord = new int[]{-1, -1};
    private static ArrayList<String[]> arrowCoord = new ArrayList<>();
    private static ArrayList<int[]> obstacleCoord = new ArrayList<>();
    private static boolean canDrawRobot = false;
    private static boolean validPosition = false;

    private static final String TAG = "GridMap";
    private static final int COL = 20;
    private static final int ROW = 20;
    public static float cellSize;
    private static Cell[][] cells;

    private boolean mapDrawn = false;


    public Map(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        initMap();
        blackPaint.setStyle(Paint.Style.FILL_AND_STROKE);
        whitePaint.setStyle(Paint.Style.FILL_AND_STROKE);
        whitePaint.setColor(Color.WHITE);
        whiteObs.setStyle(Paint.Style.FILL_AND_STROKE);
        whiteObs.setColor(Color.WHITE);
        whiteObs.setTextSize(18f);
        obstacleColor.setColor(Color.BLACK);
        robotColor.setColor(Color.GREEN);
        startColor.setColor(Color.CYAN);
        ImageColor.setColor(Color.YELLOW);
        unexploredColor.setColor(Color.LTGRAY);
        exploredColor.setColor(Color.WHITE);

        sp = getContext().getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
    }

    private void initMap() {
        setWillNotDraw(false);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        if (!mapDrawn) {
            String[] dummyArrowCoord = new String[3];
            dummyArrowCoord[0] = "1";
            dummyArrowCoord[1] = "1";
            dummyArrowCoord[2] = "dummy";
            arrowCoord.add(dummyArrowCoord);
            this.createCell();
            this.setEndCoord(19, 19);
            mapDrawn = true;
        }

        drawIndividualCell(canvas);
        drawHorizontalLines(canvas);
        drawVerticalLines(canvas);
        drawGridNumber(canvas);
        drawObstacles(canvas);
        if (getCanDrawRobot())
            drawRobot(canvas, curCoord);
        showLog("Exiting onDraw");
    }

    private void drawIndividualCell(Canvas canvas) {
        showLog("Entering drawIndividualCell");
        for (int x = 1; x <= COL; x++)
            for (int y = 0; y < ROW; y++)
                for (int i = 0; i < this.getArrowCoord().size(); i++)
                    if (!cells[x][y].type.equals("image") && cells[x][y].getId() == -1) {
                        canvas.drawRect(cells[x][y].startX, cells[x][y].startY, cells[x][y].endX, cells[x][y].endY, cells[x][y].paint);
                    } else {
                        Paint textPaint = new Paint();
                        textPaint.setTextSize(20);
                        textPaint.setColor(Color.WHITE);
                        textPaint.setTextAlign(Paint.Align.CENTER);
                        canvas.drawRect(cells[x][y].startX, cells[x][y].startY, cells[x][y].endX, cells[x][y].endY, cells[x][y].paint);
                        canvas.drawText(String.valueOf(cells[x][y].getId()),(cells[x][y].startX+cells[x][y].endX)/2, cells[x][y].endY + (cells[x][y].startY-cells[x][y].endY)/4, textPaint);
                    }
    }

    public void drawImageNumberCell(int x, int y, int id) {
        cells[x+1][19-y].setType("image");
        cells[x+1][19-y].setId(id);
        this.invalidate();
    }

    private void drawHorizontalLines(Canvas canvas) {
        for (int y = 0; y <= ROW; y++)
            canvas.drawLine(cells[1][y].startX, cells[1][y].startY - (cellSize / 30), cells[20][y].endX, cells[20][y].startY - (cellSize / 30), blackPaint);
    }

    private void drawVerticalLines(Canvas canvas) {
        for (int x = 0; x <= COL; x++)
            canvas.drawLine(cells[x][0].startX - (cellSize / 30) + cellSize, cells[x][0].startY - (cellSize / 30), cells[x][0].startX - (cellSize / 30) + cellSize, cells[x][19].endY + (cellSize / 30), blackPaint);
    }

    private void drawGridNumber(Canvas canvas) {
        for (int x = 1; x <= COL; x++) {
            if (x > 9)
                canvas.drawText(Integer.toString(x-1), cells[x][20].startX + (cellSize / 5), cells[x][20].startY + (cellSize / 2), blackPaint);
            else
                canvas.drawText(Integer.toString(x-1), cells[x][20].startX + (cellSize / 3), cells[x][20].startY + (cellSize / 2), blackPaint);
        }
        for (int y = 0; y < ROW; y++) {
            if ((20 - y) > 10)
                canvas.drawText(Integer.toString(19 - y), cells[0][y].startX + (cellSize / 3), cells[0][y].startY + (cellSize / 1.5f), blackPaint);
            else
                canvas.drawText(Integer.toString(19 - y), cells[0][y].startX + (cellSize / 2), cells[0][y].startY + (cellSize / 1.5f), blackPaint);
        }
    }

    private void drawObstacles(Canvas canvas){
        for (int[] temp :obstacleCoord){
            switch (temp[3]){
                case 1:
                    //top
                    canvas.drawRect(cells[temp[0]][this.convertRow(temp[1])].startX, cells[temp[0]][this.convertRow(temp[1])].startY, cells[temp[0]][this.convertRow(temp[1])].startX +(cellSize-cellSize/30), cells[temp[0]][this.convertRow(temp[1])].startY+(cellSize/5), ImageColor);
                    break;
                case 2:
                    //right
                    canvas.drawRect(cells[temp[0]][this.convertRow(temp[1])].endX, cells[temp[0]][this.convertRow(temp[1])].startY, cells[temp[0]][this.convertRow(temp[1])].endX - (cellSize/5), cells[temp[0]][this.convertRow(temp[1])].endY, ImageColor);
                    break;
                case 3:
                    //btm
                    canvas.drawRect(cells[temp[0]][this.convertRow(temp[1])].startX, cells[temp[0]][this.convertRow(temp[1])].endY, cells[temp[0]][this.convertRow(temp[1])].startX +(cellSize-cellSize/30), cells[temp[0]][this.convertRow(temp[1])].endY-(cellSize/5), ImageColor);
                    break;
                case 4:
                    //left
                    canvas.drawRect(cells[temp[0]][this.convertRow(temp[1])].startX, cells[temp[0]][this.convertRow(temp[1])].startY, cells[temp[0]][this.convertRow(temp[1])].startX +(cellSize/5), cells[temp[0]][this.convertRow(temp[1])].endY, ImageColor);
                    break;
            }

            if (temp[4] == 0) canvas.drawText(Integer.toString(temp[2]+1), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 3), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize / (float)1.5), whitePaint);
            else {
                switch (temp[3]){
                    case 1:
                        if (temp[4] <10) canvas.drawText(Integer.toString(temp[4]), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 3.5f), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize / 1.2f), whiteObs);
                        else canvas.drawText(Integer.toString(temp[4]), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 10), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize / 1.2f), whiteObs);
                        break;
                    case 2:
                        if (temp[4] <10) canvas.drawText(Integer.toString(temp[4]), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 5f), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize/1.5f), whiteObs);
                        else canvas.drawText(Integer.toString(temp[4]), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 35), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize / 1.5f), whiteObs);
                        break;
                    case 3:
                        if (temp[4] <10) canvas.drawText(Integer.toString(temp[4]), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 3.5f), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize/1.5f), whiteObs);
                        else canvas.drawText(Integer.toString(temp[4]), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 10), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize / 1.5f), whiteObs);
                        break;
                    case 4:
                        if (temp[4] <10) canvas.drawText(Integer.toString(temp[4]), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 3f), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize/1.5f), whiteObs);
                        else canvas.drawText(Integer.toString(temp[4]), cells[temp[0]][this.convertRow(temp[1])].startX + (cellSize / 5), cells[temp[0]][this.convertRow(temp[1])].startY + (cellSize / 1.5f), whiteObs);
                        break;
                }

            }
        }

    }

    private void drawRobot(Canvas canvas, int[] curCoord) {
        int androidRowCoord = this.convertRow(curCoord[1]);
        for (int y = androidRowCoord; y <= androidRowCoord + 1; y++)
            canvas.drawLine(cells[curCoord[0] - 1][y].startX, cells[curCoord[0] - 1][y].startY - (cellSize / 30), cells[curCoord[0] + 1][y].endX, cells[curCoord[0] + 1][y].startY - (cellSize / 30), robotColor);
        for (int x = curCoord[0] - 1; x < curCoord[0] + 1; x++)
            canvas.drawLine(cells[x][androidRowCoord - 1].startX - (cellSize / 30) + cellSize, cells[x][androidRowCoord - 1].startY, cells[x][androidRowCoord + 1].startX - (cellSize / 30) + cellSize, cells[x][androidRowCoord + 1].endY, robotColor);

        switch (this.getRobotDirection()) {
            case "N":
                canvas.drawLine(cells[curCoord[0] - 1][androidRowCoord + 1].startX, cells[curCoord[0] - 1][androidRowCoord + 1].endY, (cells[curCoord[0]][androidRowCoord - 1].startX + cells[curCoord[0]][androidRowCoord - 1].endX) / 2, cells[curCoord[0]][androidRowCoord - 1].startY, blackPaint);
                canvas.drawLine((cells[curCoord[0]][androidRowCoord - 1].startX + cells[curCoord[0]][androidRowCoord - 1].endX) / 2, cells[curCoord[0]][androidRowCoord - 1].startY, cells[curCoord[0] + 1][androidRowCoord + 1].endX, cells[curCoord[0] + 1][androidRowCoord + 1].endY, blackPaint);
                break;
            case "S":
                canvas.drawLine(cells[curCoord[0] - 1][androidRowCoord - 1].startX, cells[curCoord[0] - 1][androidRowCoord - 1].startY, (cells[curCoord[0]][androidRowCoord + 1].startX + cells[curCoord[0]][androidRowCoord + 1].endX) / 2, cells[curCoord[0]][androidRowCoord + 1].endY, blackPaint);
                canvas.drawLine((cells[curCoord[0]][androidRowCoord + 1].startX + cells[curCoord[0]][androidRowCoord + 1].endX) / 2, cells[curCoord[0]][androidRowCoord + 1].endY, cells[curCoord[0] + 1][androidRowCoord - 1].endX, cells[curCoord[0] + 1][androidRowCoord - 1].startY, blackPaint);
                break;
            case "E":
                canvas.drawLine(cells[curCoord[0] - 1][androidRowCoord - 1].startX, cells[curCoord[0] - 1][androidRowCoord - 1].startY, cells[curCoord[0] + 1][androidRowCoord].endX, cells[curCoord[0] + 1][androidRowCoord - 1].endY + (cells[curCoord[0] + 1][androidRowCoord].endY - cells[curCoord[0] + 1][androidRowCoord - 1].endY) / 2, blackPaint);
                canvas.drawLine(cells[curCoord[0] + 1][androidRowCoord].endX, cells[curCoord[0] + 1][androidRowCoord - 1].endY + (cells[curCoord[0] + 1][androidRowCoord].endY - cells[curCoord[0] + 1][androidRowCoord - 1].endY) / 2, cells[curCoord[0] - 1][androidRowCoord + 1].startX, cells[curCoord[0] - 1][androidRowCoord + 1].endY, blackPaint);
                break;
            case "W":
                canvas.drawLine(cells[curCoord[0] + 1][androidRowCoord - 1].endX, cells[curCoord[0] + 1][androidRowCoord - 1].startY, cells[curCoord[0] - 1][androidRowCoord].startX, cells[curCoord[0] - 1][androidRowCoord - 1].endY + (cells[curCoord[0] - 1][androidRowCoord].endY - cells[curCoord[0] - 1][androidRowCoord - 1].endY) / 2, blackPaint);
                canvas.drawLine(cells[curCoord[0] - 1][androidRowCoord].startX, cells[curCoord[0] - 1][androidRowCoord - 1].endY + (cells[curCoord[0] - 1][androidRowCoord].endY - cells[curCoord[0] - 1][androidRowCoord - 1].endY) / 2, cells[curCoord[0] + 1][androidRowCoord + 1].endX, cells[curCoord[0] + 1][androidRowCoord + 1].endY, blackPaint);
                break;
            default:
                break;
        }
    }

    private ArrayList<String[]> getArrowCoord() {
        return arrowCoord;
    }

    public String getRobotDirection() {
        return robotDirection;
    }

    public ArrayList<int[]> getObsCoord(){
        return obstacleCoord;
    }


    public int getArraySize(){
        return obstacleCoord.size();
    }

    public void setArrayVar(int ArrayID, int[] Changes){
        obstacleCoord.set(ArrayID, Changes);
    }


    private void setValidPosition(boolean status) {
        validPosition = status;
    }

    public boolean getValidPosition() {
        return validPosition;
    }

    public boolean getCanDrawRobot() {
        return canDrawRobot;
    }

    private void createCell() {
        showLog("Entering cellCreate");
        cells = new Cell[COL + 1][ROW + 1];
        this.calculateDimension();
        cellSize = this.getCellSize();

        for (int x = 0; x <= COL; x++)
            for (int y = 0; y <= ROW; y++)
                cells[x][y] = new Cell(x * cellSize + (cellSize / 30), y * cellSize + (cellSize / 30), (x + 1) * cellSize, (y + 1) * cellSize, unexploredColor, "unexplored");
        showLog("Exiting createCell");
    }

    public void setEndCoord(int col, int row) {
        showLog("Entering setEndCoord");
        row = this.convertRow(row);
        for (int x = col - 1; x <= col + 1; x++)
            for (int y = row - 1; y <= row + 1; y++)
                cells[x][y].setType("end");
        showLog("Exiting setEndCoord");
    }

    public void setStartCoord(int col, int row) {
        showLog("Entering setStartCoord");
        startCoord[0] = col;
        startCoord[1] = row;
        String direction = getRobotDirection();
        if(direction.equals("None")) {
            direction = "N";
        }
        if (MapControlFragment.getEditMapStatus())
            this.setCurCoord(col, row, direction);
        showLog("Exiting setStartCoord");
    }

    private int[] getStartCoord() {
        return startCoord;
    }

    public void setCurCoord(int col, int row, String direction) {
        showLog("Entering setCurCoord");
        curCoord[0] = col;
        curCoord[1] = row;
        this.setRobotDirection(direction);
        this.updateRobotAxis(col, row, direction);

        row = this.convertRow(row);

        for (int x = col-1; x <= col + 1; x++)
            for (int y = row-1; y <= row + 1; y++)
                cells[x][y].setType("robot");
        showLog("Exiting setCurCoord");
    }

    public int[] getCurCoord() {
        return curCoord;
    }

    private void calculateDimension() {
        this.setCellSize(getWidth()/(COL+1));
    }

    public int convertRow(int row) {
        return (20 - row);
    }

    private void setCellSize(float cellSize) {
        Map.cellSize = cellSize;
    }

    private float getCellSize() {
        return cellSize;
    }

    private void setOldRobotCoord(int oldCol, int oldRow) {
        showLog("Entering setOldRobotCoord");
        oldCoord[0] = oldCol;
        oldCoord[1] = oldRow;
        oldRow = this.convertRow(oldRow);
        for (int x = oldCol - 1; x <= oldCol + 1; x++)
            for (int y = oldRow - 1; y <= oldRow + 1; y++)
                cells[x][y].setType("explored");
        showLog("Exiting setOldRobotCoord");
    }

    private int[] getOldRobotCoord() {
        return oldCoord;
    }

    public void setRobotDirection(String direction) {
        sp = getContext().getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
        SharedPreferences.Editor editor = sp.edit();
        robotDirection = direction;
        editor.putString("direction", direction);
        editor.commit();
        this.invalidate();
    }

    public void updateRobotAxis(int col, int row, String direction) {
        TextView xAxisTextView =  ((Activity)this.getContext()).findViewById(R.id.xAxisTextView);
        TextView yAxisTextView =  ((Activity)this.getContext()).findViewById(R.id.yAxisTextView);
        TextView directionAxisTextView =  ((Activity)this.getContext()).findViewById(R.id.directionAxisTextView);

        xAxisTextView.setText(String.valueOf(col-1));
        yAxisTextView.setText(String.valueOf(row-1));
        directionAxisTextView.setText(direction);
    }

    public void setObstacleCoord(int col, int row, int ObsID, int dir, int TarID, int ChangeFlag) {
        int[] obstacleCoord = new int[]{col, row, ObsID, dir, TarID, ChangeFlag};
        Map.obstacleCoord.add(obstacleCoord);
        row = this.convertRow(row);
        cells[col][row].setType("obstacle");
    }

    private ArrayList<int[]> getObstacleCoord() {
        return obstacleCoord;
    }

    private void showLog(String message) {
        Log.d(TAG, message);
    }

    private class Cell {
        float startX, startY, endX, endY;
        Paint paint;
        String type;
        int id = -1;

        private Cell(float startX, float startY, float endX, float endY, Paint paint, String type) {
            this.startX = startX;
            this.startY = startY;
            this.endX = endX;
            this.endY = endY;
            this.paint = paint;
            this.type = type;
        }

        public void setType(String type) {
            this.type = type;
            switch (type) {
                case "obstacle":
                    this.paint = obstacleColor;
                    break;
                case "robot":
                    this.paint = robotColor;
                    break;
                case "start":
                    this.paint = startColor;
                    break;
                case "unexplored":
                    this.paint = unexploredColor;
                    break;
                case "explored":
                    this.paint = exploredColor;
                    break;
                case "image":
                    this.paint = obstacleColor;
                default:
                    showLog("setTtype default: " + type);
                    break;
            }
        }

        public void setId(int id) {
            this.id = id;
        }

        public int getId() {
            return this.id;
        }
    }

    public void resetMap() {
        showLog("Entering resetMap");
        TextView robotStatusTextView =  ((Activity)this.getContext()).findViewById(R.id.robotStatusTextView);
        ImageView robotImage = ((Activity)this.getContext()).findViewById(R.id.robotImage);

        updateRobotAxis(1, 1, "None");
        robotStatusTextView.setText("Not Available");
        SharedPreferences.Editor editor = sp.edit();

        ToggleButton setMap = ((Activity)this.getContext()).findViewById(R.id.setBtn);

        if (setMap.getText().equals("Set robot")){
            robotImage.setVisibility(VISIBLE);
        }

        startCoord = new int[]{-1, -1};
        curCoord = new int[]{-1, -1};
        oldCoord = new int[]{-1, -1};
        robotDirection = "None";
        obstacleCoord = new ArrayList<>();
        mapDrawn = false;
        canDrawRobot = false;
        validPosition = false;

        MapControlFragment.setRobotStatus(false);

        showLog("Exiting resetMap");
        this.invalidate();
    }



    public void flyRobot(int row, int col, String direction){
        int[] curCoord = this.getCurCoord();
        this.setOldRobotCoord(curCoord[0], curCoord[1]);
        this.setCurCoord(row, col, direction);
        this.invalidate();
    }

    public void moveRobot(String direction) {
        setValidPosition(false);
        int[] curCoord = this.getCurCoord();
        ArrayList<int[]> obstacleCoord = this.getObstacleCoord();
        this.setOldRobotCoord(curCoord[0], curCoord[1]);
        int[] oldCoord = this.getOldRobotCoord();
        String robotDirection = getRobotDirection();
        String backupDirection = robotDirection;

        switch (robotDirection) {
            case "N":
                switch (direction) {
                    case "forward":
                        if (curCoord[1] != 19) {
                            curCoord[1] += 1;
                            validPosition = true;
                        }
                        break;
                    case "right":
                        robotDirection = "E";
                        break;
                    case "back":
                        if (curCoord[1] != 2) {
                            curCoord[1] -= 1;
                            validPosition = true;
                        }
                        break;
                    case "left":
                        robotDirection = "W";
                        break;
                    default:
                        robotDirection = "error up";
                        break;
                }
                break;
            case "E":
                switch (direction) {
                    case "forward":
                        if (curCoord[0] != 19) {
                            curCoord[0] += 1;
                            validPosition = true;
                        }
                        break;
                    case "right":
                        robotDirection = "S";
                        break;
                    case "back":
                        if (curCoord[0] != 2) {
                            curCoord[0] -= 1;
                            validPosition = true;
                        }
                        break;
                    case "left":
                        robotDirection = "N";
                        break;
                    default:
                        robotDirection = "error right";
                }
                break;
            case "S":
                switch (direction) {
                    case "forward":
                        if (curCoord[1] != 2) {
                            curCoord[1] -= 1;
                            validPosition = true;
                        }
                        break;
                    case "right":
                        robotDirection = "W";
                        break;
                    case "back":
                        if (curCoord[1] != 19) {
                            curCoord[1] += 1;
                            validPosition = true;
                        }
                        break;
                    case "left":
                        robotDirection = "E";
                        break;
                    default:
                        robotDirection = "error down";
                }
                break;
            case "W":
                switch (direction) {
                    case "forward":
                        if (curCoord[0] != 2) {
                            curCoord[0] -= 1;
                            validPosition = true;
                        }
                        break;
                    case "right":
                        robotDirection = "N";
                        break;
                    case "back":
                        if (curCoord[0] != 19) {
                            curCoord[0] += 1;
                            validPosition = true;
                        }
                        break;
                    case "left":
                        robotDirection = "S";
                        break;
                    default:
                        robotDirection = "error left";
                }
                break;
            default:
                robotDirection = "error moveCurCoord";
                break;
        }
        if (getValidPosition())
            for (int x = curCoord[0] - 1; x <= curCoord[0] + 1; x++) {
                for (int y = curCoord[1] - 1; y <= curCoord[1] + 1; y++) {
                    for (int i = 0; i < obstacleCoord.size(); i++) {
                        if (obstacleCoord.get(i)[0] != x || obstacleCoord.get(i)[1] != y)
                            setValidPosition(true);
                        else {
                            setValidPosition(false);
                            break;
                        }
                    }
                    if (!getValidPosition())
                        break;
                }
                if (!getValidPosition())
                    break;
            }
        if (getValidPosition())
            this.setCurCoord(curCoord[0], curCoord[1], robotDirection);
        else {
            if (direction.equals("forward") || direction.equals("back"))
                robotDirection = backupDirection;
            this.setCurCoord(oldCoord[0], oldCoord[1], robotDirection);
        }
        this.invalidate();
    }

    public void printRobotStatus(String message) {
        TextView robotStatusTextView = ((Activity)this.getContext()).findViewById(R.id.robotStatusTextView);
        robotStatusTextView.setText(message);
    }

    public void setRobot(int row, int column){
        if (MapControlFragment.getEditMapStatus()) {
            if (canDrawRobot) {
                int[] startCoord = this.getStartCoord();
                if (startCoord[0] >= 2 && startCoord[1] >= 2) {
                    startCoord[1] = this.convertRow(startCoord[1]);
                    for (int x = startCoord[0] - 1; x <= startCoord[0]+1; x++)
                        for (int y = startCoord[1] - 1; y <= startCoord[1]+1; y++)
                            cells[x][y].setType("unexplored");
                }
            }
            else
                canDrawRobot = true;
            this.setStartCoord(column, row);

            String direction = getRobotDirection();
            if(direction.equals("None")) {
                direction = "N";
            }
            try {
                int directionInt = 0;
                if(direction.equals("N")){
                    directionInt = 0;
                } else if(direction.equals("W")) {
                    directionInt = 3;
                } else if(direction.equals("E")) {
                    directionInt = 1;
                } else if(direction.equals("S")) {
                    directionInt = 2;
                }
                MainActivity.printMessage("starting " + "(" + (row - 1) + "," + (column - 1) + "," + directionInt + ")");
            } catch (Exception e) {
                e.printStackTrace();
            }
            updateRobotAxis(column, row, direction);
            this.invalidate();
        }
    }

    public void removerobot(){
        int[] startCoord = this.getStartCoord();
        if (startCoord[0] >= 2 && startCoord[1] >= 2) {
            startCoord[1] = this.convertRow(startCoord[1]);
            for (int x = startCoord[0] - 1; x <= startCoord[0] + 1; x++)
                for (int y = startCoord[1] - 1; y <= startCoord[1] + 1; y++)
                    cells[x][y].setType("unexplored");
        }
        canDrawRobot = false;
        setRobotDirection("None");
        updateRobotAxis(1, 1, getRobotDirection());
        this.invalidate();
    }

    public void setObstacle(int row, int column, int ObsID, int dir, int TarID, int Changeflag, int setFlag){
        int[] intArray = new int[]{column, row, ObsID, dir, TarID, Changeflag};

        if (setFlag == 0) this.setObstacleCoord(column, row, ObsID, dir, TarID, Changeflag);

        else if (setFlag == 1) {
            int[] temp = obstacleCoord.get(ObsID);
            cells[temp[0]][this.convertRow(temp[1])].setType("unexplored");
            cells[column][this.convertRow(row)].setType("obstacle");
            setArrayVar(ObsID, intArray);
        }

        else {
            int size = getArraySize();
            for (int x = ObsID+1; x < size; x++){
                int[] temp = obstacleCoord.get(x);
                temp[2] = x-1;
                setArrayVar(x, temp);
            }
            int[] temp = obstacleCoord.get(ObsID);
            cells[temp[0]][this.convertRow(temp[1])].setType("unexplored");
            obstacleCoord.remove(ObsID);

        }
        Log.d("setobs", row+"_"+column);
        this.invalidate();

    }

}

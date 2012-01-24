/*
  DriveView.java is part of the RoseWheel project.
  Copyright (C) 2011 RoseWheel Team <rosewheel@googlegroups.com>
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

package fr.rosewheel.RoseWheel;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Point;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.AttributeSet;
import android.view.Display;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;

public class DriveView extends View implements SensorEventListener {
	private Sensor mAccelerometer;
    private SensorManager mSensorManager;
    private WindowManager mWindowManager;
    private Display mDisplay;
    private Paint mPaint;
    private RoseWheelApp mRemote;
    private boolean mSensorsStarted = false;
    private boolean mEnableValues = false;
    private boolean mUseSensors = true;
    private int mFilteredSensorX;
    private int mFilteredSensorY;
    private int mGivenX;
    private int mGivenY;
    private static final float mAlpha = 0.8f;
    private Path mPath = null;
    private boolean mStopToZero = false;
    
    public DriveView(Context context, AttributeSet attributes) {
    	super(context, attributes);
        // Get an instance of the SensorManager
        mSensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        // Get an instance of the WindowManager
        mWindowManager = (WindowManager) context.getSystemService(Context.WINDOW_SERVICE);
        mDisplay = mWindowManager.getDefaultDisplay();
        mPaint = new Paint();
        mFilteredSensorX = 0;
        mFilteredSensorY = 0;
        
        mPath = new Path();
        mPath.setFillType(Path.FillType.EVEN_ODD);
        mPath.moveTo(-50, -50);
        mPath.lineTo(-50, 50);
        mPath.lineTo(50, 0);
        mPath.close();        
    }
    
    public void setRemote(RoseWheelApp r) {
    	mRemote = r;
    }
    
    public void startSensors() {
    	if (!mSensorsStarted) {
    		mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_UI);
    		mSensorsStarted = true;
    	}
    }

    public void stopSensors() {
    	if (mSensorsStarted) {
    		mSensorsStarted = false;
    		mSensorManager.unregisterListener(this);
    	}
    }
    
    public void stopToZero() {
    	mStopToZero = true;
    }
     
    public void enableValues() {
    	mEnableValues = true;
    }

    public void disableValues() {
    	mEnableValues = false;
    }
    
    public synchronized void useSensors(boolean state) {
    	mUseSensors = state;
        invalidate();
    }
    
    public synchronized void setGivenXY(int x, int y) {
    	this.mGivenX = x;
    	this.mGivenY = y;
    	invalidate();
    }
    
    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
    		this.setMeasuredDimension(w, h);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() != Sensor.TYPE_ACCELEROMETER)
            return;
        
        mFilteredSensorX *= mAlpha;
        mFilteredSensorY *= mAlpha;
        event.values[0] *= 50 * (1.0 - mAlpha);
        event.values[1] *= 50 * (1.0 - mAlpha);
        
        switch (mDisplay.getOrientation()) {
            case Surface.ROTATION_0:
            	mFilteredSensorX -= (int)event.values[0];
            	mFilteredSensorY += (int)event.values[1];
                break;
            case Surface.ROTATION_90:
            	mFilteredSensorX -= (int)-event.values[1];
            	mFilteredSensorY += (int)event.values[0];
                break;
            case Surface.ROTATION_180:
            	mFilteredSensorX -= (int)-event.values[0];
            	mFilteredSensorY += (int)-event.values[1];
                break;
            case Surface.ROTATION_270:
            	mFilteredSensorX -= (int)event.values[1];
            	mFilteredSensorY += (int)-event.values[0];
                break;
        }
                
        if (mFilteredSensorX > 100)
        	mFilteredSensorX = 100;
        if (mFilteredSensorY > 100)
        	mFilteredSensorY = 100;
        if (mFilteredSensorX < -100)
        	mFilteredSensorX = -100;
        if (mFilteredSensorY < -100)
        	mFilteredSensorY = -100;
        
        if (mStopToZero)
        {
        	stopSensors();
        	mStopToZero = false;
        	mRemote.sendSensors(0, 0);
        	return;
        }
        
        if (mEnableValues)
        	mRemote.sendSensors(mFilteredSensorX, mFilteredSensorY);
        else
        	mRemote.sendSensors(0, 0);
    }
    
    @Override
    protected synchronized void onDraw(Canvas canvas) {
    	mPaint.setColor(Color.BLACK);
    	canvas.drawRect(0, 0, canvas.getWidth(), canvas.getHeight(), mPaint);

    	int x;
    	int y;
    	
    	if (mUseSensors) {
    		x = this.mFilteredSensorX;
    		y = this.mFilteredSensorY;
    		this.mGivenX = x;
    		this.mGivenY = y;
    	}
    	else {
    		x = this.mGivenX;
    		y = this.mGivenY;
    	}
    	
    	if (!mEnableValues) {
        	x = 0;
        	y = 0;    		
    	}
    	
    	Point center = new Point(canvas.getWidth()  / 2,
    							 canvas.getHeight() / 2);
    	int min = Math.min(center.x, center.y);
    	Point moving = new Point(center.x + x * center.x / 100,
				                 min + y * min / 100);

    	mPaint.setAlpha(255);
    	mPaint.setStyle(Paint.Style.STROKE);
    	mPaint.setColor(Color.WHITE);
    	canvas.drawLine(center.x - min, min, center.x + min, min, mPaint);
    	canvas.drawLine(center.x, 0, center.x, 2 * min, mPaint);
    	
    	if (mUseSensors)
    		mPaint.setColor(Color.GREEN);
    	else
    		mPaint.setColor(Color.RED);
    	
    	mPaint.setAlpha(127);
    	mPaint.setStyle(Paint.Style.FILL);
    	canvas.translate(moving.x, moving.y);
    	canvas.drawPath(mPath, mPaint);
        // and make sure to redraw asap
    	canvas.translate(-moving.x, -moving.y);
        invalidate();
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
	
}

/*
  RoseWheelApp.java is part of the RoseWheel project.
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

/*
 * Copyright (C) 2009 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package fr.rosewheel.RoseWheel;

import android.content.res.Resources;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnTouchListener;
import android.widget.Button;
import android.widget.TabHost;
import android.widget.TabHost.TabContentFactory;
import android.widget.TextView;

/**
 * This is the main Activity that displays the current chat session.
 */
public class RoseWheelApp extends BluetoothActivity {
	// Debugging
	private static final String TAG = "RoseWheelApp";
	private static final boolean D = true;
	private static final float VADVICED_KMH = 12.0f;
	private static final float VMAX_KMH = 26.0f;

	private TextView mAngle;
	private TextView mBattery;
	private TextView mSpeed;

	private TextView mAngleLabel;
	private TextView mBatteryLabel;
	private TextView mSpeedLabel;
	private TextView mTurnLabel;

	private Button mKlaxon;
	private Button mLock;
	private Button mWakeUp;
	private Button mDanger;
	
	private GaugeView mBatteryGauge;
	private GaugeView mSpeedGauge;
	private TurnView mTurnView;

	enum State {CONTROL_STATE, SONARS};
	enum ControlState {HUMAN, ASLEEP, REMOTE};
	
	private boolean mLocked = true;
	private ControlState mControlState = ControlState.REMOTE;
	private boolean mAsleep = false;
	private boolean mSonarsEnabled = false;

	private StringBuilder buffer = null;

	private DriveView mDriveView = null;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		if (D)
			Log.e(TAG, "+++ ON CREATE +++");

		buffer = new StringBuilder();

		// Set up the window layout
		setContentView(R.layout.main);

		mTitle = (TextView) findViewById(R.id.title_left_text);
		mTitle.setText(R.string.app_name);
		mTitle = (TextView) findViewById(R.id.title_right_text);

		mDebug = (TextView) findViewById(R.id.debug);

		final View remote = (View) findViewById(R.id.remote);
		final View gui = (View) findViewById(R.id.gui);

		Resources res = getResources(); // Resource object to get Drawables
		TabHost tabHost = getTabHost(); // The activity TabHost
		TabHost.TabSpec spec; // Resusable TabSpec for each tab

		// Initialize a TabSpec for each tab and add it to the TabHost
		spec = tabHost.newTabSpec("remote")
				.setIndicator("", res.getDrawable(R.drawable.remote))
				.setContent(new TabContentFactory() {
					public View createTabContent(String arg0) {
						return remote;
					}
				}

				);
		tabHost.addTab(spec);

		spec = tabHost.newTabSpec("gui")
				.setIndicator("", res.getDrawable(R.drawable.odometer))
				.setContent(new TabContentFactory() {
					public View createTabContent(String arg0) {
						return gui;
					}
				}

				);
		tabHost.addTab(spec);

		tabHost.setCurrentTab(1);

		mDriveView = (DriveView) findViewById(R.id.driveview);
		mDriveView.enableValues();
		mDriveView.setRemote(this);

		mKlaxon = (Button) findViewById(R.id.button_klaxon);
		mKlaxon.setOnTouchListener(mKlaxonListener);
		mLock = (Button) findViewById(R.id.button_lock);
		mLock.setOnTouchListener(mLockListener);
		mWakeUp = (Button) findViewById(R.id.button_wakeup);
		mWakeUp.setOnClickListener(mWakeUpListener);
		mWakeUp.setText("Sleep");
		mDanger = (Button) findViewById(R.id.button_danger);
		mDanger.setOnClickListener(mDangerListener);
		mDanger.setText("Sonars ON");

		mAngleLabel = (TextView) findViewById(R.id.angle_label);
		mAngleLabel.setText("Bend Angle");
		mBatteryLabel = (TextView) findViewById(R.id.battery_label);
		mBatteryLabel.setText("Battery");
		mSpeedLabel = (TextView) findViewById(R.id.speed_label);
		mSpeedLabel.setText("Command");
		mTurnLabel = (TextView) findViewById(R.id.turn_label);
		mTurnLabel.setText("Turn Angle");

		mAngle = (TextView) findViewById(R.id.angle);
		mAngle.setText("");
		mBattery = (TextView) findViewById(R.id.battery);
		mBattery.setText("");
		mSpeed = (TextView) findViewById(R.id.speed);
		mSpeed.setText("");
		
		mSpeedGauge = (GaugeView) findViewById(R.id.speed_gauge);
		mBatteryGauge = (GaugeView) findViewById(R.id.battery_gauge);
		int[] colors = new int[2];
		colors[0] = Color.rgb(164, 198, 57);
		colors[1] = Color.rgb(164, 198, 57);
		mBatteryGauge.setColors(colors);
		
		mTurnView = (TurnView) findViewById(R.id.turn_view);
		
		setControlState(ControlState.REMOTE);
	}

	public void setLocked(boolean b) {
		if (b) {
			mLocked = true;
			mLock.setText("Unlock");
		}
		else {
			mLocked = false;
			mLock.setText("Lock");
		}

	}
	
	private void setControlState(ControlState state) {
		mControlState = state;
		mAsleep = state == ControlState.ASLEEP;
		
		if (mAsleep)
			mWakeUp.setText("Wake Up");
		else
			mWakeUp.setText("Sleep");
		
		boolean human = state == ControlState.HUMAN;
		mLock.setClickable(!human);
		mDriveView.useSensors(!human);
		mTurnView.setUsed(!human);
	}

	private void setSonarsState(boolean state) {
		mSonarsEnabled = state;
		if (mSonarsEnabled)
			mDanger.setText("Sonars OFF");
		else
			mDanger.setText("Sonars ON");
	}
	
	private OnTouchListener mKlaxonListener = new OnTouchListener() {
		@Override
		public boolean onTouch(View v, MotionEvent event) {
			String command = "k";

			if (event.getAction() == MotionEvent.ACTION_UP)
				sendCommand(command + "0");
			else if (event.getAction() == MotionEvent.ACTION_DOWN)
				sendCommand(command + "1");

			return false;
		}
	};

	private OnTouchListener mLockListener = new OnTouchListener() {
		@Override
		public boolean onTouch(View v, MotionEvent event) {
			if (mAsleep)
				return false;

			if (!mLocked && event.getAction() == MotionEvent.ACTION_UP) {
				mDriveView.disableValues();
				mDriveView.stopToZero();
				setLocked(true);
			} else if (mLocked && event.getAction() == MotionEvent.ACTION_DOWN) {
				mDriveView.enableValues();
				mDriveView.startSensors();
				setLocked(false);
			}
			
			return false;
		}
	};

	private OnClickListener mWakeUpListener = new OnClickListener() {
		@Override
		public void onClick(View v) {
			if (mAsleep) {
				setControlState(ControlState.REMOTE);
				sendCommand("w");
			}
			else {
				setControlState(ControlState.ASLEEP);
				sendCommand("S");
			}
		}
	};

	private OnClickListener mDangerListener = new OnClickListener() {
		@Override
		public void onClick(View v) {			
			setSonarsState(!mSonarsEnabled);
			if (mSonarsEnabled)
				sendCommand("d1");
			else
				sendCommand("d0");
		}
	};

	@Override
	public void onStart() {
		super.onStart();
		if (D)
			Log.e(TAG, "++ ON START ++");
	}

	@Override
	public synchronized void onResume() {
		super.onResume();
		if (D)
			Log.e(TAG, "+ ON RESUME +");
	}

	@Override
	public synchronized void onPause() {
		super.onPause();
		if (D)
			Log.e(TAG, "- ON PAUSE -");
	}

	@Override
	public void onStop() {
		super.onStop();
		mDriveView.stopSensors();
		if (D)
			Log.e(TAG, "-- ON STOP --");
	}

	@Override
	public void onDestroy() {
		if (D)
			Log.e(TAG, "--- ON DESTROY ---");
		super.onDestroy();
		mDriveView.stopSensors();
	}

	public void sendSensors(int x, int y) {
		sendCommand("c" + x + " " + y);
	}

	public void sendCommand(String message) {
		mDebug.setText(message);

		if (mBluetoothService.getState() == BluetoothService.STATE_CONNECTED) {
			sendMessage("#" + message + "\r\n");
		}
	}

	@Override
	protected void readMessage(String message) {
		buffer.append(message);
		for (;;) {
			int index = buffer.indexOf("\r\n");
			if (index == -1)
				break;
			interpreter(buffer.substring(0, index));
			buffer.delete(0, index + 2);
		}
	}

	protected void interpreter(String message) {
		if (message.length() == 0)
			return;

		if (message.charAt(0) != '#')
			return;

		if (message.length() == 1)
			return;

		char command = message.charAt(1);

		if (message.length() == 2)
			return;

		String args_string = message.substring(2).trim();
		String[] args = args_string.split(" ");

		/*
		 * if (args_string.indexOf('#') != -1) return;
		 */

		try {
			Integer a = null, b = null;
			switch (command) {
			case 'S':
				if (args.length != 2)
					return;
				a = Integer.parseInt(args[0]);
				b = Integer.parseInt(args[1]);
				setSonarsState(b != 0);
				switch (a) {
				case 0:
					setControlState(ControlState.REMOTE);
					break;
				case 1:
					setControlState(ControlState.HUMAN);
					break;
				case 2:
					setControlState(ControlState.ASLEEP);
					break;
				}
				break;
			case 'm':
				if (args.length != 2)
					return;
				
				a = Integer.parseInt(args[0]);
				b = Integer.parseInt(args[1]);
				Float af = a * VMAX_KMH / 100.0f;
				Float bf = b * VMAX_KMH / 100.0f;
				Float speedf = (Math.abs(af) + Math.abs(bf)) / 2.0f;
				Float turnf = (af - bf) / 2.0f;
				Integer speed_int = (int)((Math.abs(a) + Math.abs(b)) / 2.0f);
				mSpeedGauge.setValue(speedf / VADVICED_KMH);
				mTurnView.setValue(turnf / (2 * VADVICED_KMH) + 0.5f);
				mSpeed.setText(speed_int.toString());
				
				if (mControlState == ControlState.HUMAN) {
					Integer speed = (int)((a + b) / 2);
					Integer turn =  (int)((a - b) / 2);
					mDriveView.setGivenXY(speed, turn);
				}
				break;
			case 'b':
				if (args.length != 1)
					return;
				a = Integer.parseInt(args[0]);
				mBattery.setText(a.toString());			
				mBatteryGauge.setValue(a / 100.0f);
				break;
			case 'a':
				if (args.length != 1)
					return;
				a = Integer.parseInt(args[0]);
				mAngle.setText(a.toString());
				break;
			case 'h':
				break;
			case 'o':
				break;
			case 'p':
				sendMessage("ping");
				break;
			}
		} catch (Exception e) {
		}
	}
}

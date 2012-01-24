/*
  GaugeView.java is part of the RoseWheel project.
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
import android.graphics.Rect;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.GradientDrawable.Orientation;
import android.util.AttributeSet;

public class GaugeView extends ValueView {
	private GradientDrawable mGradient;
	private int[] mColors;
	private final int mStrokeWidth = 2;
	
	public GaugeView(Context context, AttributeSet attributes) {
		super(context, attributes);
		
		int[] default_colors = new int[2];
		default_colors[0] = Color.BLUE;
		default_colors[1] = Color.RED;
		setColors(default_colors);		
	}
	
	synchronized private void updateGradient() {
		mGradient = new GradientDrawable(Orientation.BOTTOM_TOP, mColors);
		mGradient.setGradientType(GradientDrawable.LINEAR_GRADIENT);
		mGradient.setDither(true);
		mGradient.setStroke(mStrokeWidth, Color.WHITE);
	}
	
	synchronized public void setColors(int[] colors) {
		mColors = colors;
		updateGradient();
	}
		
    @Override
    synchronized protected void onDraw(Canvas canvas) {
		mGradient.setBounds(canvas.getClipBounds());
		mGradient.draw(canvas);
    	
    	Rect r = canvas.getClipBounds();
    	
		r.top += mStrokeWidth;
    	r.left += mStrokeWidth;
    	r.right -= mStrokeWidth;
    	r.bottom -= mStrokeWidth;
    	
    	r.bottom *= (1.0f - mValue);
    	
    	mPaint.setStyle(Paint.Style.FILL);
		mPaint.setColor(Color.BLACK);
    	canvas.drawRect(r, mPaint);    	
    	
        invalidate();
    }
}

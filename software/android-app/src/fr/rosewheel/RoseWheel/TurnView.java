/*
  TurnView.java is part of the RoseWheel project.
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
import android.graphics.Rect;
import android.util.AttributeSet;

public class TurnView extends ValueView {
	private Path mPath;
	private Boolean mUsed = false;
		
	public TurnView(Context context, AttributeSet attributes) {
		super(context, attributes);
		
        mPath = new Path();
        mPath.setFillType(Path.FillType.EVEN_ODD);
        mValue = 0.5f;
	}
	
	public void setUsed(boolean b) {
		mUsed = b;
	}
	
    @Override
    protected void onDraw(Canvas canvas) {
    	Rect r = canvas.getClipBounds();
    	int width = Math.abs(r.right - r.left);
    	int height = Math.abs(r.top - r.bottom);
    	int radius = Math.min(width, height) / 2 - 5;
        int edge = radius / 2;

        mPath.reset();
        mPath.moveTo(-edge, edge);
        mPath.lineTo(edge, edge);
        mPath.lineTo(0, -edge);
        mPath.close();
    	
    	mPaint.setColor(Color.WHITE);
    	mPaint.setStyle(Paint.Style.FILL);
    	mPaint.setStrokeWidth(5.0f);
    	mPaint.setAlpha(125);
    	canvas.drawCircle(width / 2, height / 2, radius, mPaint);

    	mPaint.setAlpha(255);
    	mPaint.setStyle(Paint.Style.STROKE);
    	canvas.drawCircle(width / 2, height / 2, radius, mPaint);
    	
    	canvas.translate(width / 2, height / 2);
    	canvas.rotate(90 * (mValue - 0.5f) / 0.5f);
    	canvas.drawLine(0, -height / 2 + 5, 0, height / 2 - 5, mPaint);
    	
    	if (mUsed)
    		mPaint.setColor(Color.rgb(164, 198, 57));
    	else
    		mPaint.setColor(Color.RED);
    	
    	mPaint.setStyle(Paint.Style.FILL);
        mPaint.setAlpha(200);
    	canvas.drawPath(mPath, mPaint);
    	
        invalidate();
    }
}

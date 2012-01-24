/*
  ValueView.java is part of the RoseWheel project.
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
import android.graphics.Paint;
import android.util.AttributeSet;
import android.view.View;

abstract class ValueView extends View {
	protected float mValue = 0.0f;
	protected Paint mPaint;

	public ValueView(Context context, AttributeSet attributes) {
		super(context, attributes);
		mPaint = new Paint();
	}
	
	public void setValue(float value) {
		if (value > 1.0f)
			value = 1.0f;
		if (value < 0.0f)
			value = 1.0f;
		mValue = value;
	}
	
	public float getValue() {
		return mValue;
	}	
	
    @Override
    protected abstract void onDraw(Canvas canvas);
}

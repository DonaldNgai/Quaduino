package com.example.donald.quaduinoflightcontrol;

import android.content.Context;
import android.view.View;
import android.widget.TextView;

/**
 * Created by donald on 15/03/16.
 */
public class AddOrSubOnClickListener implements View.OnClickListener{

    double amount = 0;
    double oldValue = 0;
    TextView label = null;
    Context context = null;

    public AddOrSubOnClickListener(Context context,double amount,double oldValue,TextView label){
        this.amount = amount;
        this.oldValue = oldValue;
        this.context = context;
        this.label = label;
    }

    @Override public void onClick(View v) {
//        Toast.makeText(context, label.getText().toString(), Toast.LENGTH_SHORT).show();
        double newValue =  oldValue + amount;
        oldValue = newValue;
//        Toast.makeText(context, newValue, Toast.LENGTH_SHORT).show();
        label.setText(String.valueOf(newValue));
    }

}

// ======================================================================
// ===                    DATA SMOOTHING FUNCTION                     ===
// ======================================================================

int digitalSmooth(int rawIn, int *sensSmoothArray, int *sortedArray, int *inIndex, int bufferSize){     // "int *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  int j, k, top, bottom;
  int temp;
  long total;
  
  boolean done;

  sensSmoothArray[*inIndex] = rawIn;                 // input new data into the oldest slot
  *inIndex = (*inIndex + 1) % bufferSize;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable

  for (j=0; j<bufferSize; j++)// transfer data array into anther array for sorting and averaging
  {     
    sortedArray[j] = sensSmoothArray[j];
  }

  done = 0; // flag to know when we're done sorting              
  while(done != 1) // simple swap sort, sorts numbers from lowest to highest
  {        
    done = 1;
    for (j = 0; j < (bufferSize - 1); j++)
    {
      if (sortedArray[j] > sortedArray[j + 1])  // numbers are out of order - swap
      {     
        temp = sortedArray[j + 1];
        sortedArray [j+1] =  sortedArray[j] ;
        sortedArray [j] = temp;
        done = 0;
      }
    }
  }


//  for (j = 0; j < (bufferSize); j++){    // print the array to debug
//    Serial.print(sortedArray[j]); 
//    Serial.print("   "); 
//  }
//  Serial.println();


  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((bufferSize * 15)  / 100), 1); 
  top = min((((bufferSize * 85) / 100) + 1  ), (bufferSize - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++)
  {
    total += sortedArray[j];  // total remaining indices
    k++; 
//     Serial.print(sortedArray[j]); 
//     Serial.print("   "); 
  }

//  Serial.println();
//  Serial.print("total = " + String(total) + " ");
//  Serial.println(total/k);
  return total / k;    // divide by number of samples
}

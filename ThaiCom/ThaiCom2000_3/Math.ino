void unitVec(float u[3]){
  float sizeVec = sqrt(long(u[0])*long(u[0]) + long(u[1])*long(u[1]) + long(u[2])*long(u[2]));
  for(int i=0; i<3; i++){
    u[i] /= sizeVec;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void crossVector(float u[3], float v[3], float out[3]){
  out[0] = u[1]*v[2] - u[2]*v[1];
  out[1] = u[2]*v[0] - u[0]*v[2];
  out[2] = u[0]*v[1] - u[1]*v[0];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void generate3x3(float u[3], float v[3], float w[3], float out[3][3]){
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      switch(j){
        case 0:  out[i][j] = u[i]; break;
        case 1:  out[i][j] = v[i]; break;
        case 2:  out[i][j] = w[i]; break;
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void inverse3x3(float u[3][3], float out[3][3]){
  float det = u[0][0]*u[1][1]*u[2][2] - u[0][0]*u[1][2]*u[2][1] - u[0][1]*u[1][0]*u[2][2] + u[0][1]*u[1][2]*u[2][0] + u[0][2]*u[1][0]*u[2][1] - u[0][2]*u[1][1]*u[2][0];
  
  out[0][0] = (u[1][1]*u[2][2] - u[1][2]*u[2][1])  / det;
  out[0][1] = -(u[0][1]*u[2][2] - u[0][2]*u[2][1]) / det;
  out[0][2] = (u[0][1]*u[1][2] - u[0][2]*u[1][1])  / det;
  
  out[1][0] = -(u[1][0]*u[2][2] - u[1][2]*u[2][0]) / det;
  out[1][1] = (u[0][0]*u[2][2] - u[0][2]*u[2][0])  / det;
  out[1][2] = -(u[0][0]*u[1][2] - u[0][2]*u[1][0]) / det;
  
  out[2][0] = (u[1][0]*u[2][1] - u[1][1]*u[2][0])  / det;
  out[2][1] = -(u[0][0]*u[2][1] - u[0][1]*u[2][0]) / det;
  out[2][2] = (u[0][0]*u[1][1] - u[0][1]*u[1][0])  / det;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void MultipleMatrix33x33(float u[3][3], float v[3][3], float out[3][3]){
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      out[i][j] = 0;
    }
  }
  
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      for(int k=0; k<3; k++){
        out[i][j] += u[i][k] * v[k][j];
      }  
    }
  } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void MultipleMatrix33x31(float u[3][3], float v[3], float out[3]){
  for(int i=0; i<3; i++){
    out[i] = 0;
  }
  
  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++){
      out[i] += u[i][j] * v[j];  
    }
  } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

  /* for Test
  crossVector(xx,yy,zz);
  generate3x3(xx,yy,zz,vec);
  inverse3x3(vec,invvec);
  MultipleMatrix(invvec,zz,multi);
  Serial.print(invvec[0][0]);
  Serial.print("  ");
  Serial.print(invvec[0][1]);
  Serial.print("  ");
  Serial.println(invvec[0][2]);
  Serial.print(invvec[1][0]);
  Serial.print("  ");
  Serial.print(invvec[1][1]);
  Serial.print("  ");
  Serial.println(invvec[1][2]);
  Serial.print(invvec[2][0]);
  Serial.print("  ");
  Serial.print(invvec[2][1]);
  Serial.print("  ");
  Serial.println(invvec[2][2]);
  Serial.println();
  Serial.print(multi[0]);
  Serial.print("  ");
  Serial.print(multi[1]);
  Serial.print("  ");
  Serial.println(multi[2]);
  delay(100);
  
  crossVector(xx,yy,zz);
  generate3x3(xx,yy,zz,vec);
  crossVector(xxx,yyy,zzz);
  generate3x3(xxx,yyy,zzz,vvec);
  MultipleMatrix33x33(vec,vvec,multi3);
  Serial.print(idealVec[0][0]);
  Serial.print("  ");
  Serial.print(idealVec[0][1]);
  Serial.print("  ");
  Serial.println(idealVec[0][2]);
  Serial.print(idealVec[1][0]);
  Serial.print("  ");
  Serial.print(idealVec[1][1]);
  Serial.print("  ");
  Serial.println(idealVec[1][2]);
  Serial.print(idealVec[2][0]);
  Serial.print("  ");
  Serial.print(idealVec[2][1]);
  Serial.print("  ");
  Serial.println(idealVec[2][2]);
  Serial.println();
  delay(100);
  */

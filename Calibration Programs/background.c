#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

const string filename1 ="libCalib.dat";
const string filename2 ="lightLog.txt";
TFileIOResult nIoResult1, nIoResult2;
TFileHandle filehandle1, filehandle2;
int fileSize = 1000;

task main(){
  Delete(filename1, nIoResult1);
  Delete(filename2, nIoResult2);
  OpenWrite(filehandle1, nIoResult1, filename1, fileSize);
  OpenWrite(filehandle2, nIoResult2, filename2, fileSize);
  motor[Left]=10;
  motor[Right]=10;
  nMotorEncoder[Left]=0;
  nMotorEncoder[Right]=0;
  short max = 0, min = 1023;

  int timer1 = 0;

  while(timer1 < 10){
    short lightValue = SensorValue[S3];
    string str;
    StringFormat(str, "%d %d %d\n", lightValue, nMotorEncoder[Left], nMotorEncoder[Right]);
    WriteString(filehandle2, nIoResult2, str);
    nxtScrollText(str);
    if(lightValue>max){
      max = lightValue;
    }
    if(lightValue < min){
      min = lightValue;
    }

    wait1Msec(200);
    timer1++;
  }

  WriteShort(filehandle1, nIoResult1, min);
  WriteShort(filehandle1, nIoResult1, max);
  Close(filehandle1,nIoResult1);
  Close(filehandle2,nIoResult2);
  eraseDisplay();
  //nxtEraseRect(0,0,63,63);
  nxtDisplayBigStringAt(0,31,"%d %d", min, max);
  motor[Left]=0;
  motor[Right]=0;
  wait1Msec(10000);


  return;
}

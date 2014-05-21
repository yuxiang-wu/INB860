#pragma config(Sensor, S3,     Light,          sensorLightActive)
#pragma config(Motor,  motorA,          Right,         tmotorNormal, PIDControl, encoder)
#pragma config(Motor,  motorB,          Left,          tmotorNormal, PIDControl, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

const string filename1 ="libCalib.dat";
const string filename2 ="compassCalib.dat";
TFileIOResult nIoResult;
int fileSize=1000;
TFileHandle filehandle;

task main(){
  //read the max and min from file "libCalib.dat"
  OpenRead(filehandle, nIoResult, filename1, fileSize);
  short min, max;
  ReadShort(filehandle, nIoResult, min);
  ReadShort(filehandle, nIoResult, max);
  Close(filehandle, nIoResult);
  short threshold = (min + max) / 2;

  float k;
  motor[Left]=15;
  motor[Right]=30;

  //observe a transition from white to dark and reset motor encoder
  while(SensorValue[Light] > threshold){}
  nMotorEncoder[Left]=0;
  nMotorEncoder[Right]=0;
  wait1Msec(1500);

  //observe white-to-dark transition again and record the motor encoder value, stop the motor
  while(SensorValue[Light] > threshold){}
  wait1Msec(1500);
  while(SensorValue[Light] > threshold){}
  wait1Msec(1500);
  while(SensorValue[Light] > threshold){}

  int leftMotorEncode = nMotorEncoder[Left];
  int rightMotorEncode = nMotorEncoder[Right];
  motor[Left]=0;
  motor[Right]=0;

  k = 3*360.0/(rightMotorEncode - leftMotorEncode);

  Delete(filename2, nIoResult);
  OpenWrite(filehandle, nIoResult, filename2, fileSize);
  WriteFloat(filehandle, nIoResult, k);
  Close(filehandle, nIoResult);

  string str;
  StringFormat(str, "%f", k);
  nxtDisplayCenteredTextLine(7, str);
  wait1Msec(10000);
  return;
}

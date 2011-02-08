  
void requestEvent(){
   // Send roll-pitch-yaw over I2C when requested 
   
   	byte IMU_buffer[6];
	int tempint;
	long templong;

      tempint=ToDeg(roll)*100;  //Roll (degrees) * 100 in 2 bytes
	IMU_buffer[0]=tempint&0xff;
	IMU_buffer[1]=(tempint>>8)&0xff;
      
	tempint=ToDeg(pitch)*100;   //Pitch (degrees) * 100 in 2 bytes
	IMU_buffer[2]=tempint&0xff;
	IMU_buffer[3]=(tempint>>8)&0xff;
      
	templong=ToDeg(yaw)*100;  //Yaw (degrees) * 100 in 2 bytes
	if(templong>18000) templong -=36000;
	if(templong<-18000) templong +=36000;
	tempint = templong;
	IMU_buffer[4]=tempint&0xff;
	IMU_buffer[5]=(tempint>>8)&0xff;
      
      byte* pointer = &IMU_buffer[0];
      Wire.send(pointer, 6);
  }
  
void receiveEvent(int howMany){
   
   byte GPS_buffer[10];
   for(int i=0; i < howMany; i++){
     GPS_buffer[i]=Wire.receive();
   }
   
   GPS.fill_buffer(GPS_buffer);

}


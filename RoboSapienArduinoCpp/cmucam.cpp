#include "cmucam.h"


// constructor
CMUCAM::CMUCAM(HardwareSerial* commport) {
   
   // NULLing scan line
   ScanLineRedChannel = NULL;  
   ScanLineGreenChannel = NULL;
   ScanLineBlueChannel = NULL;
   
   
   // setting up serial port
   CommPort = commport;
   
   // assigning initial state
   state = stIdle;
   
   ScannedLines = 0;   
   // clearing frame ready flag
   flagScanLineReady = false;
   
   CurrentFrameHeight = CurrentFrameWidth = 200;
   ScannedLines = 0;
            
   // clearing abstraction ready flag
   flagAbstractionDataReady = false;
   
   // zeroing current abstraction byte
   current_abst_byte = 0;
            
   // opening serial port 
   CommPort->begin(57600);
            
}



void CMUCAM::getAbstractionData() {
   
   CommPort->print("A");
   CommPort->write(13);
   
   
   AbstractionRequestTime = millis();
   
   state = stWaitingAbstraction;
   
}


// requesting dimensions of frame for streaming
void CMUCAM::getStreamingDims() {
   
   CommPort->print("D");
   CommPort->write(13);
   
   
   DimensionsRequestTime = millis();
   state = stWaitingStreamingDims;
}

// retrieve dimensions of the frame for an upcoming abstraction
void CMUCAM::getAbstDims() {
   CommPort->print("D");
   CommPort->write(13);
   
   DimensionsRequestTime = millis();
   state = stWaitingAbstDims;
}
        
void CMUCAM::resetCamera() {
   CommPort->print("R");
   CommPort->write(13);
}

void CMUCAM::getLine() {
  // disposing scanline channels first
  disposeScanLine();
  // marking start of Transmission
  LineRequestTime = millis();
  flagScanLineReady = false;
  
  if ((ScannedLines==CurrentFrameHeight)||(ScannedLines==0)) {// previous frame finished
    ScannedLines=0;
    getStreamingDims();
  } else {
    CommPort->print("S");
    CommPort->write(13);

  
    state = stWaitingScanLine;
  }
}



void CMUCAM::transitionAction() {

  int i, j, imminent_bytes;
 // Serial.print("Bytes available for serial3: ");
 // Serial.println(CommPort->available(), DEC);
  switch (state) {
      case stIdle:
           break;
      case stWaitingStreamingDims:
           if (CommPort->available() >= 4) {
                 
                
                 
                 byte* dimbuf = (byte*)malloc(4);
                 for (i=0; i<4; i++)
                     dimbuf[i] = CommPort->read();
                 CurrentFrameWidth = dimbuf[0] + dimbuf[1] * 256;
                 CurrentFrameHeight = dimbuf[2] + dimbuf[3] * 256;

                
                 
                 // going for the lines now...
                 CommPort->flush();
                 ScannedLines = 0;
                 Serial.println("Requesting Line following dimension request");
                 CommPort->print("S");
                 CommPort->write(13);
                 
                 state = stWaitingScanLine;
                }
                 else { // timeout checking....
                   unsigned long dimresponsetime = millis() - DimensionsRequestTime;
                   unsigned long linetime = millis() - LineRequestTime;
                   if (linetime > 3000) // 3 seconds timeout for line
                   {// frame timeout in 10 seconds
                       CommPort->flush();
                       // raise a failure flag here!!!    
                       resetCamera();
                       state = stIdle;
                   }
                    else if (dimresponsetime > 2000) // 2 second timeout for dimensions
                        {
                            CommPort->flush();
                            getStreamingDims(); // request dimensions again
                        }
                        
                 }
                 break;
      case stWaitingScanLine:
          
           if (CommPort->available() >= 3*CurrentFrameWidth) {
              
               
              // allocating the new scanned line channel arrays
              ScanLineRedChannel = (byte*)malloc(CurrentFrameHeight);
              ScanLineGreenChannel = (byte*)malloc(CurrentFrameHeight);
              ScanLineBlueChannel = (byte*)malloc(CurrentFrameHeight);
               
              ScannedLines++;

              // now reading the scan line
               
              byte* RGBBuf = (byte*)malloc(CurrentFrameWidth * 3);
              for (i=0; i<CurrentFrameWidth * 3; i++)
                  RGBBuf[i] = CommPort->read();
              // assigning RGB values
             
                  
              for (i = 0; i < CurrentFrameWidth; i++) {
                   ScanLineRedChannel[i] = RGBBuf[i * 3];
                   ScanLineGreenChannel[i] = RGBBuf[i * 3 + 1];
                   ScanLineBlueChannel[i] = RGBBuf[i * 3 + 2];
              }
             
              // disposing the RGB buffer
              free(RGBBuf);

              // raise flag
              flagScanLineReady = true;
              // change state
              state = stIdle;
                     
                           
           }
             else { // checking timeouts
                unsigned long lineresponsetime = millis() - LineRequestTime;
                if (lineresponsetime > 2000) // 2 second timeout for dimensions
                {
                     CommPort->flush();
                     //getLine() ; // request the line again again
                     // raise an error flag
                     resetCamera();
                     state = stIdle;
                }
            }



            break;
      
      case stScanLineRead: // nothing
                        break;

                case stWaitingAbstDims: // waiting for the abstracted frame's dimensions
                    if (CommPort->available() >= 4)
                    {
                        byte* dimbuf = (byte*)malloc(4);
                        for (i=0; i<4; i++) 
                            dimbuf[i] = CommPort->read();
                            
                        AbstractedFrameWidth = dimbuf[0] + dimbuf[1] * 256;
                        AbstractedFrameHeight = dimbuf[2] + dimbuf[3] * 256;

                        // going for the lines now...
                        CommPort->flush();
                        
                        getAbstractionData();
                    }
                    else
                    { // timeout checking....
                        unsigned long dimresponsetime = millis() - DimensionsRequestTime;
                        unsigned long abstractiontime = millis() - AbstractionRequestTime;
                        if (abstractiontime > 10000) // 10 seconds timeout for frame
                        {// frame timeout in 10 seconds
                            CommPort->flush();
                            // raise a failure flag here!!!    
                            resetCamera();
                            state = stIdle;
                        }
                        else if (dimresponsetime > 1000) // 1 second timeout for dimensions
                        {
                            CommPort->flush();
                            getAbstDims(); // request dimensions again
                        }

                    }
                    break;  
                case stWaitingAbstraction: // expecting the abstracted frame
                    if (ABST_FRAME_WIDTH*ABST_FRAME_HEIGHT*3-current_abst_byte<CHUNK_SIZE)
                        imminent_bytes = ABST_FRAME_WIDTH*ABST_FRAME_HEIGHT*3-current_abst_byte;
                    else 
                        imminent_bytes = CHUNK_SIZE;
                      
                      
                      
                        
                    if (CommPort->available() >= imminent_bytes)
                    {
                      

                        // now reading the data
                        for (i=0; i<imminent_bytes; i++)
                            RawbytesBuffer[i] = CommPort->read();
                        RawbytesLength = imminent_bytes;
                        
                        
                        current_abst_byte += imminent_bytes;

                        // changing state
                        state = stAbstractionRead;
                        

                    }
                    else
                    { // checking timeouts
                        unsigned long abstractiontime = millis() - AbstractionRequestTime;
                        
                        if (abstractiontime >5000) // 5 seconds timeout for frame
                        {// frame timeout in 10 seconds
                            CommPort->flush();
                            // raise a failure flag here!!!
                            Serial.println("Camera Timedout!");
                            resetCamera();
                            state = stIdle;
                        }
                    }



                    break;

            }
        }


// processEvents checkes the internal state and performs additional actions
void CMUCAM::processEvents() {
     if (state == stScanLineRead)
      { // frame available
        state = stIdle;
        flagScanLineReady = true;
      }
       else if (state == stAbstractionRead)
            { // abstracted frame chunk available
                if (current_abst_byte==ABST_FRAME_WIDTH*ABST_FRAME_HEIGHT*3)
                    current_abst_byte = 0;
                state = stIdle;
                flagAbstractionDataReady = true;
            }
                
}

// dispose a CMUCAM frame
void CMUCAM::disposeCMUCAMFrame(CMUCAMFrame* frame) {
  int i;
  if (frame!=NULL) {
    if (frame->RedChannel!=NULL) {
      for (i=0; i<frame->Height; i++)
          free(frame->RedChannel[i]);
      free(frame->RedChannel);
    }
    if (frame->GreenChannel!=NULL) {
      for (i=0; i<frame->Height; i++)
          free(frame->GreenChannel[i]);
      free(frame->GreenChannel);
    }
    if (frame->BlueChannel!=NULL) {
      for (i=0; i<frame->Height; i++)
        free(frame->BlueChannel[i]);
      free(frame->BlueChannel);
    }
    free(frame);
    frame = NULL;
  }
}



void CMUCAM::disposeScanLine() {
  if (ScanLineRedChannel!=NULL) {
      free(ScanLineRedChannel);
      ScanLineRedChannel = NULL;
    }
    if (ScanLineGreenChannel!=NULL) {
      free(ScanLineGreenChannel);
      ScanLineGreenChannel = NULL;
    }
    if (ScanLineBlueChannel!=NULL) {
      free(ScanLineBlueChannel);
      ScanLineBlueChannel = NULL;
    }
}


EZPacket* CMUCAM::createAbstractionChunkPacket(byte nodeid, byte datareceiver) {
        int i;
        uint16_t paramlen;
        // Creating an EZPacket
        EZPacket* pack = (EZPacket*)malloc(sizeof(EZPacket));
        pack->SenderID = nodeid;
        pack->ReceiverID = datareceiver;
        pack->SenderNodeType = t_Node_Robot;
        
        // Message length should be 4 bytes (command) +  CHUNKSIZE (or remaining bytes) ()
        
        pack->MessageLength = 4+RawbytesLength;
        // allocating the message
        pack->Message = (byte*)malloc(pack->MessageLength);
        
        // now filling the message
        pack->Message[0] = (byte)t_RobosapienV2; // robot type
        pack->Message[1] = (byte)rc_CMUCAMAbstractionData; // command code
        
        paramlen = RawbytesLength;
        pack->Message[2] = lowByte(paramlen); // low byte of the command's parameters length
        pack->Message[3] = highByte(paramlen); // high byte of the length of the parameters
        
        Serial.print("Parameter Length: ");
        Serial.println(paramlen, DEC);
        // copying the bytes now
        for (i=0; i<paramlen; i++)
          pack->Message[4+i] = RawbytesBuffer[i];
          
        
        
        
        return pack;
}


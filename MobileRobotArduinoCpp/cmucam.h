#ifndef CMUCAM_H
#define CMUCAM_H

#include "WProgram.h"
#include "ezrobonet.h"
#include "ezrobonetcmds.h"


typedef struct
{
    int Width, Height;

    byte** RedChannel;
    byte** GreenChannel;
    byte** BlueChannel;
} CMUCAMFrame;



class CMUCAM {
        
        // some timing variables 
        
        public: 
                unsigned long LineRequestTime;
                unsigned long DimensionsRequestTime;
                unsigned long AbstractionRequestTime;
        

        // serial port
        public: HardwareSerial* CommPort;
        public: static const int UART_NOMINAL_BAUD_RATE = 57600;
                static const int UART_HIGH_BAUD_RATE = 115200;
                static const int CHUNK_SIZE = 100;


   
        // a scan line from the frame
        public: byte* ScanLineRedChannel;
                byte* ScanLineGreenChannel;
                byte* ScanLineBlueChannel;
                byte RawbytesBuffer[CHUNK_SIZE];
                byte RawbytesLength;
                
                
        
        public: static const int ABST_FRAME_HEIGHT = 8;
                static const int ABST_FRAME_WIDTH = 8;


        // state constants
        public: static const int stIdle = 0;
                static const int stWaitingStreamingDims = 1;
                static const int stWaitingScanLine = 2;
                static const int stScanLineRead = 3;
                static const int stWaitingAbstDims = 4;
                static const int stWaitingAbstraction = 5;
                static const int stAbstractionRead = 6;
        

        // internal state
        public: int state;
        // Frame ready flag
        public: boolean flagScanLineReady;
                boolean flagAbstractionDataReady;

        // frame upload related variables
        public: int ScannedLines;
                int CurrentFrameWidth, CurrentFrameHeight;
                int AbstractedFrameWidth, AbstractedFrameHeight;
                int current_abst_byte;

        // constructor
        public: CMUCAM(HardwareSerial* commport);
        
        
        

        // requesting dimensions of frame for sreaming
        public: void getStreamingDims();

        // retrieve dimensions of the frame for an upcoming abstraction
        public: void getAbstDims();
        
        public: void resetCamera();

        public: void getLine();

        public: void getAbstractionData();

        public: void transitionAction();

        // processEvents checkes the internal state and performs additional actions
        public: void processEvents();
        
        // dispose a CMUCAMFrame structure
        public: void disposeCMUCAMFrame(CMUCAMFrame* frame);
          
        // dispose a scan line
        public: void disposeScanLine();

        public: EZPacket* createAbstractionChunkPacket(byte nodeid, byte datareceiver);
};



#endif

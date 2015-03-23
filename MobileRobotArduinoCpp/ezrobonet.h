// *************************************************************
// *                   EZRoboNet Protocol                      *
// *             for R/F transmissions using an                *
// *            easyRadio transceiver with Arduino             *
// *                                                           *
// *                 George Terzakis (Tsambikos) 2010          *
// *                                                           *
// *************************************************************

#ifndef EZROBONET_H
#define EZROBONET_H

#include "WProgram.h"


// frame starter and terminator characters 
// (CRC checksum follows terminator)
const byte framestarter = (byte)'*';
const byte frameterminator = 0xA;
// maximum size for a frame payload
const uint16_t MAX_FRAME_PAYLOAD_LENGTH = 50;
// maximum size of a message in a packet
const uint16_t MAX_PACKET_MESSAGE_LENGTH = 1024;

// Transmission Time out Constants
// packet send/receive timeout
const uint16_t PACKET_SENDING_TIMEOUT = 5000; 
const uint16_t FRAME_ACKNOWLEDGE_TIMEOUT = 500;

// types of nodes
typedef enum {
        t_Node_Station,
        t_Node_Robot }EZNode_t;

// types of eZroboNet packet contents
typedef  enum {
          t_Ping,
          t_Data,
          t_Acknowledge }EZFrame_t;

// link level frame (may contain part of, or entire eZroboNet packet
typedef struct {
    byte SenderID;               // 1-255. 
    byte ReceiverID;             // 1-255. 0  broadcast
    EZFrame_t Ftype;
    uint16_t PayloadLength;  // length of the payload
    byte *Payload;               // payload 
}EZFrame;

// eZroboNet packet
typedef struct {
  byte SenderID;     // 1-255. 
  byte ReceiverID;   // 1-255. 0 broadcast
  EZNode_t SenderNodeType;    // type of sender node (robot or station) 
  uint16_t MessageLength;
  byte *Message;     // maximum 1024 bytes
}EZPacket;
  
// CRC calculation   
uint16_t calculateCRC(byte* pData, int numBytes);
// CRC check
bool checkCRC(byte* pData, int numBytes, uint16_t acrc);

class EZRoboNetDevice {
  // state constants
  public: static const byte stIdle = 0;
          static const byte stSending = 1;
          static const byte stReceiving = 2;
          static const byte stWaitingAcknowledge = 3; 
          static const byte stPacketReady = 4;
          static const byte MAX_OUTBOUND_PACKETS_NUM = 4;
          
  // members
  public: byte NodeID; 
           EZNode_t DevNodeType; // type of node Robot or Station
           HardwareSerial* CommPort;
           byte state;
           unsigned long TransmissionStartTime, ReceptionStartTime;
           unsigned long FrameTransmissionStartTime, FrameReceptionStartTime; 
           uint16_t FramesReceived, FramesSent, TotalFrames;
           uint16_t CurrentPacketLength;
           byte* CurrentPacket;
           byte PacketReceiverID;
           byte PacketSenderID;
           uint16_t remainingBytes;
           byte FrameHead[6];
           
  // some flags...
  public: bool PacketSendTimeout;
          bool PacketReceiveTimeout;
          bool PacketQueueOverflow;
  // list of packets received. Maximum is 2 
  public: byte ReceivedPacketsNum;
          // received packets queue
          EZPacket* ReceivedPackets[2];
          // outbound packets queue
          EZPacket* OutboundPackets[MAX_OUTBOUND_PACKETS_NUM];
          byte OutboundPacketsNum;
           
  // constructor
  public: EZRoboNetDevice(byte node_id, EZNode_t node_type, HardwareSerial* commport);
  
  // constructing a byte array out of a packet
  public: static byte* packet2Bytes(EZPacket* pack);
  
  // constructing a byte array out of a frame 
  // (including CRC checksum as two bytes in the end and starter-first and terminator prior to CRC)
  public: static byte* frame2Bytes(EZFrame* frame);
  
  // retrieves the length of a raw frame (which probably just arrived)
  public: static uint16_t getRawFrameLength(byte* frame);
  
  // retrieves the length of a packet from a raw byte stream
  public: static uint16_t getRawPacketLength(byte* rawpacket);
  
  // convert raw bytes (newly arrived frame apparently) to a frame structure 
  public: static EZFrame* bytes2Frame(byte* rawframe);

  // Convert raw bytes to a packet structure
  public: static EZPacket* bytes2Packet(byte* rawpacket);
  
  // dispose packet structure
  public: static void disposePacket(EZPacket* pack);
  
  // dispose frame structure
  public: static void disposeFrame(EZFrame* frame);
  
  
  // initiate transmission of a packet
  public: void sendPacket(EZPacket* pack);
  
  
  // send the next frame in a transmission sequence
  private: void sendNextFrame();
  
  // call this method upon first data reception
  private: void receivedNewPacket(EZFrame* frame);
  
  // creating an acknowledgement frame
  public: EZFrame* makeAcknowledge(byte receiver_id);
  
  // send an acknowledgment
  private: void sendAcknowledge(byte receiver_id);
  
  // handling a newly arrived frame (following initial packet reception)
  private: void receivedNewFrame(EZFrame* frame);
    
  // change of internal state and firing of consequent actions upon frame arrival
  public: void transitionAction(EZFrame* frame);
  
  // clear serial data
  public: void flushSerialBuffer();
  
  // processData processes incoming data, assembles frames and calls to transitionAction
  public: void processData();
  
  // processing of events (ready packets, timeouts, etc..)
  public: void processEvents();
  
  
  
  // available packets
  byte packetsAvailable();
  
  // flags
  bool getPacketQueueOverflow();

  bool getPacketReceiveTimeout();

  bool getPackeSendTimeout();
  
  // clear the flags
  public: void clearFlags();
  
  // clear PacketSendTimeout flag
  public: void clearPacketSendTimeoutFlag();
  
  // clear PacketreceiveTimeout flag
  public: void clearPacketReceiveTimeoutFlag();
  
  // clear PacketQueueOverflow flag
  public: void clearPacketQueueOverflowFlag();
  
  public: EZPacket* getNextAvailablePacket();
  // flags ends
  
  // Clear the Packet Queue, reset the flags
  public: void resetPacketTransceiver();
  
  // examines if the outbound packet queue is fulll
  public: boolean OutboundQueueFull(); 
  
  // pushes a new packet into the outbound queue
  public: boolean pushOutboundPacket(EZPacket* pack); 
};
           
  


#endif



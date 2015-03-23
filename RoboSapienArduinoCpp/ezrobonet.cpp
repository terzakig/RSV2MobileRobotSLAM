/////////////////// EZRobotNet ///////////////////////////////////
//
//			Arduino .cpp file: ezrobonet.cpp

// A Network Protocol for Wireless/Wired Serial Communications for Cost Effective Robotic applications
// 
// Copyright (C) 2010 George Terzakis

//	This program is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.

//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//	GNU General Public License for more details.
//	You should have received a copy of the GNU General Public License along
//	with this program; if not, write to the Free Software Foundation, Inc.,
//	51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. 

#include "ezrobonet.h"

uint16_t calculateCRC( byte* pData, int numBytes )
{
    int index = 0;
    uint16_t crc = 0;

    while( index < numBytes )
    {
        crc =  (byte)(crc >> 8) | (crc << 8);
        crc ^= pData[ index++ ];
        crc ^= (byte)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }
    
    return crc;
}

bool checkCRC(byte* pData, int numBytes, uint16_t acrc) {
  uint16_t ccrc = calculateCRC(pData, numBytes);
  
  return acrc == ccrc;
}

EZRoboNetDevice::EZRoboNetDevice(byte node_id, EZNode_t node_type, HardwareSerial* commport) {
    NodeID = node_id;
    DevNodeType = node_type;
    
    // assigning serial port
    CommPort = commport;
    CommPort->begin(19200);
    
    // initializing state
    state = stIdle;
    // flags
    PacketSendTimeout = false;
    PacketReceiveTimeout = false;
    PacketQueueOverflow = false;
    // Available Packets Queue
    ReceivedPacketsNum = 0;
    ReceivedPackets[0] = NULL;
    ReceivedPackets[1] = NULL;
    // initializing the outbound packets queue
    int i;
    for (i=0; i<MAX_OUTBOUND_PACKETS_NUM; i++)
        OutboundPackets[i] = NULL;
    OutboundPacketsNum = 0;
    
    // incoming data processing globals
    remainingBytes = 0;
}
  
// returns an euivalent array of bytes to a packet
byte* EZRoboNetDevice::packet2Bytes(EZPacket* pack) {
  int packlen = 5 + pack->MessageLength; // 5 bytes in the header plus the message length
  byte* rawmsg = (byte*)malloc(packlen);
  rawmsg[0] = pack->SenderID;
  rawmsg[1] = pack->ReceiverID;
  rawmsg[2] = (byte)pack->SenderNodeType;
  rawmsg[3] = lowByte(pack->MessageLength);
  rawmsg[4] = highByte(pack->MessageLength);
  int i;
  for (i=0; i<pack->MessageLength; i++) 
    rawmsg[5+i] = pack->Message[i];
  
  return rawmsg;
}

// returns the eqiv. array of bytes (with CRC checksum added) of a frame structure
byte* EZRoboNetDevice::frame2Bytes(EZFrame* frame) {
  int framelen = 5 + frame->PayloadLength + 4; // 5 header bytes plus payload, plus starter, terminator and CRC checksum
  byte* rawframe = (byte*)malloc(framelen);
  
  rawframe[0] = framestarter;
  rawframe[1] = frame->SenderID;
  rawframe[2] = frame->ReceiverID;
  rawframe[3] = (byte)frame->Ftype;
  rawframe[4] = lowByte(frame->PayloadLength);
  rawframe[5] = highByte(frame->PayloadLength);
  int i;
  for (i=0; i<frame->PayloadLength; i++) 
    rawframe[6+i] = frame->Payload[i];
  rawframe[framelen-3] = frameterminator;
  // now adding CRC
  uint16_t crc = calculateCRC(rawframe, framelen-2);
  rawframe[framelen-2] = lowByte(crc);
  rawframe[framelen-1] = highByte(crc);
  
  return rawframe;
}
  
// length of a raw frame
uint16_t EZRoboNetDevice::getRawFrameLength(byte* rawframe) {
  
  return 9+rawframe[4]+256*rawframe[5];
}

// length of a raw packet
uint16_t EZRoboNetDevice::getRawPacketLength(byte* rawpacket) {
  
  return 5 + rawpacket[3] + 256*rawpacket[4];
}

// Convert bytes to a Frame structure
EZFrame* EZRoboNetDevice::bytes2Frame(byte* rawframe) {
   EZFrame* frame = (EZFrame*)malloc(sizeof(EZFrame));
   uint16_t payloadlen = rawframe[4]+256*rawframe[5];
   
   frame->SenderID = rawframe[1];
   frame->ReceiverID = rawframe[2];
   frame->Ftype = (EZFrame_t)rawframe[3];
   frame->PayloadLength = payloadlen;
   
   frame->Payload = (byte*)malloc(payloadlen);
   int i;
   for (i=0; i<payloadlen; i++)
     frame->Payload[i] = rawframe[6+i];
   
   return frame;
}

// Convert bytes to a packet structure
EZPacket* EZRoboNetDevice::bytes2Packet(byte* rawpacket) {
  EZPacket* packet = (EZPacket*)malloc(sizeof(EZPacket));
  uint16_t messagelen = rawpacket[3]+256*rawpacket[4];
  
  packet->SenderID = rawpacket[0];
  packet->ReceiverID = rawpacket[1];
  packet->SenderNodeType = (EZNode_t)rawpacket[2];
  packet->MessageLength = messagelen;
  packet->Message = (byte*)malloc(messagelen);
  int i;
  for (i=0; i<messagelen; i++)
    packet->Message[i] = rawpacket[5+i];
  
  
  return packet;
}

// dispose packet
void EZRoboNetDevice::disposePacket(EZPacket* pack) {
  if (pack!=NULL) {
    free(pack->Message);
    free(pack);
  }
  pack = NULL;
}

//dispose frame
void EZRoboNetDevice::disposeFrame(EZFrame* frame) {
  if (frame!=NULL) {
    free(frame->Payload);
    free(frame);
  }
  frame = NULL;
}


// initiate transmission of a packet
void EZRoboNetDevice::sendPacket(EZPacket* pack) {
    byte theReceiver = pack->ReceiverID;
    CurrentPacket = packet2Bytes(pack);
    // disposing pack now...
    disposePacket(pack);
    // retrieving length of raw packet data
    CurrentPacketLength = getRawPacketLength(CurrentPacket);
    
    // computing number of frames required
    TotalFrames = CurrentPacketLength /  MAX_FRAME_PAYLOAD_LENGTH +1;
    
    // zeroing number of frames sent
    FramesSent = 0;
    // Clearing the timeout flag
    PacketSendTimeout = false;
    // marking the starting time for possible timeouts
    TransmissionStartTime = millis();
    // Setting Packet receiver Global
    PacketReceiverID = theReceiver;
    // setting state to sending...
    state = stSending;
    // now sending first frame
    sendNextFrame();

}
void EZRoboNetDevice::sendNextFrame() {
    // creating the first frame
    EZFrame* frame = (EZFrame*)malloc(sizeof(EZFrame));
    frame->SenderID = NodeID;
    frame->ReceiverID = PacketReceiverID;
    frame->Ftype = t_Data;
    int payloadlen, index;
    
    index = FramesSent*MAX_FRAME_PAYLOAD_LENGTH;
    // computing size of the current frame's payload
    if (FramesSent==TotalFrames-1) 
       payloadlen = CurrentPacketLength % MAX_FRAME_PAYLOAD_LENGTH;
    else 
      payloadlen = MAX_FRAME_PAYLOAD_LENGTH;
    
    
    frame->PayloadLength = payloadlen;
    frame->Payload = (byte*)malloc(payloadlen);
    
    int i;
    for (i=0; i<payloadlen; i++)
      frame->Payload[i] = CurrentPacket[i+index];
    // frame created
    byte* rawframe = frame2Bytes(frame);
    int rawframelen = getRawFrameLength(rawframe);
    
    // stream the frame through the serial port 
    for (i=0; i<rawframelen; i++) 
      CommPort->write(rawframe[i]);
      
    
    // god forbid, packet was sent. Changing state now
    state = stWaitingAcknowledge;
    // marking frame transmission time for possible acknowledge timeout
    FrameTransmissionStartTime = millis();
    
    // disposing rawframe
    free(rawframe);
    // disposing the frame
    disposeFrame(frame);
}

// call this method upon first data reception
void EZRoboNetDevice::receivedNewPacket(EZFrame* frame) {
  // caspturing time of first reception
  ReceptionStartTime = millis();
  // retrieving sender id
  PacketSenderID = frame->SenderID;
  // calculating total packet length
  uint16_t packetlength = frame->Payload[3]+256*frame->Payload[4]+5;
  CurrentPacketLength = packetlength;
  // calculating total number of frames required for full packet transmission
  TotalFrames = packetlength / MAX_FRAME_PAYLOAD_LENGTH + 1;
  
  // allocating memory space for the packet being received
  CurrentPacket = (byte*)malloc(packetlength);
  // now filling initial space in CurrentPacket
  int i;
  for (i=0; i<frame->PayloadLength; i++)
    CurrentPacket[i] = frame->Payload[i];
  // done copying
  // disposing frame
  disposeFrame(frame);
  // increasing number of frames received
  FramesReceived++;
  // sending an acknowledgement packet 
  sendAcknowledge(PacketSenderID);
  if (FramesReceived<TotalFrames) { // need to receive more
    state = stReceiving;
    // marrking time of acknowledgement
    FrameReceptionStartTime = millis();
  }
  else {
       state = stPacketReady;
       FramesReceived = 0;
       TotalFrames = 0;
  }
       
}
    
// create an acknowledgement frame
EZFrame* EZRoboNetDevice::makeAcknowledge(byte receiver_id) {
  EZFrame* frame = (EZFrame*)malloc(sizeof(EZFrame));
  
  frame->SenderID = NodeID;
  frame->ReceiverID = receiver_id;
  frame->Ftype = t_Acknowledge;
  frame->PayloadLength = 2;
  frame->Payload = (byte*)malloc(frame->PayloadLength);
  frame->Payload[0] = 0;
  frame->Payload[1] = 0;
  
  return frame;
}

// send acknowledgment
void EZRoboNetDevice::sendAcknowledge(byte receiver_id) {
  EZFrame* ackframe = makeAcknowledge(receiver_id);
  // converting struct to  bytes
  byte* rawackframe = frame2Bytes(ackframe);
  
  
  // retrieving length
  uint16_t framelength = getRawFrameLength(rawackframe);
   Serial.println(framelength);
   delay(10);
  // disposing the struct
  disposeFrame(ackframe);
  // sending bytes
  int i;
  for (i=0; i<framelength; i++) 
    CommPort->write(rawackframe[i]);
    
  
  // done
  // disposing raw frame
  free(rawackframe);
}

// handle next frame
void EZRoboNetDevice::receivedNewFrame(EZFrame* frame) {
  
  // copying payload bytes into the packet
  int i;
  int startindex = MAX_FRAME_PAYLOAD_LENGTH * FramesReceived;
  for (i=0; i<frame->PayloadLength; i++)
      CurrentPacket[startindex+i] = frame->Payload[i];
  // frame payload copied
  // increasing number of received frames
  FramesReceived++;
  // sending ackjnowledgement
  sendAcknowledge(PacketSenderID);
  // disposing frame
  disposeFrame(frame);
  // now changing state
  if (FramesReceived==TotalFrames) state = stPacketReady;
    else {
          state = stReceiving;
          // marking time for next frame arrival possible timeout
          FrameReceptionStartTime = millis();
    }

}


// handling internal state according to input and triggering appropriate actions
void EZRoboNetDevice::transitionAction(EZFrame* frame) {
  switch(state) {
    case stIdle: // not in process process of receiving or sending
                 switch(frame->Ftype) {
                   case t_Ping:         // received a ping frame. will acknowledge to sender...
                                        sendAcknowledge(frame->SenderID);
                                        disposeFrame(frame);
                                        state = stIdle; // state remains idle
                                        break;
                   case t_Acknowledge: // acknowledgement frame. should ignore at this state
                                       state = stIdle;
                                       break;
                   case t_Data:        // A Data frame. Must initiate Packet reception
                                       PacketSenderID = frame->SenderID;
                                       TotalFrames = 0;
                                       FramesReceived = 0;
                                       // initialized reception globals
                                       // now calling to new packet reception method
                                       receivedNewPacket(frame);
                                       // state should change inside receivedNewPacket...
                                       break;
                 }
                 break;
    case stSending: // in the process of sending a packet. ignore incoming...
                 disposeFrame(frame); // frame bye-bye
                 // normally, flow should never enter this case clause
                 break;
    case stReceiving: // in the process of receiving frames
                      switch(frame->Ftype) {
                        case t_Ping: // just reply and go back to wait for the next frame in the sequence
                                     sendAcknowledge(frame->SenderID);
                                     disposeFrame(frame);
                                     state = stReceiving;
                                     break;
                        case t_Data: // Data frame arrived. See if we can fill the packet some more
                                     if (PacketSenderID==frame->SenderID) {// received a frame from the original sender
                                        receivedNewFrame(frame);
                                     }
                                     else { // packet does not originate fron the current sender
                                            // disposing inetercepted frame
                                            //Serial.println(frame->SenderID, DEC);
                                            disposeFrame(frame);
                                            state = stReceiving; // still waiting for a new frame
                                      }
                                      break;
                        case t_Acknowledge: // acknowledgment packet arrived
                                            // do nothing. dispose the frame
                                            disposeFrame(frame);
                                            break;
                      }
                  break;    
    case stWaitingAcknowledge: // expecting a frame reception acknowledgement
                               switch(frame->Ftype) {
                                 case t_Ping: // casual ping. replying...
                                              sendAcknowledge(frame->SenderID);
                                              disposeFrame(frame);
                                              // state remains
                                              state = stWaitingAcknowledge;
                                              break;
                                 case t_Data: // data frame received
                                              // dispose the frame and wait for acknowledge
                                              disposeFrame(frame);
                                              state = stWaitingAcknowledge;
                                              break;
                                 case t_Acknowledge: // received and acknowledge frame
                                                     if (frame->SenderID==PacketReceiverID) {
                                                       Serial.println("Got Acknowledge");
                                                        // increasing number of sent frames
                                                        FramesSent++;
                                                        if (FramesSent==TotalFrames) {// all sent
                                                          // disposing packet and reseting
                                                          Serial.println("ALL Frames Sent!");
                                                          free(CurrentPacket);
                                                          CurrentPacket = NULL;
                                                          PacketReceiverID = 0;
                                                          PacketSenderID = 0;
                                                          CurrentPacketLength = 0;
                                                          
                                                          state = stIdle;
                                                        } else { // need to send more frames
                                                                 state = stSending;
                                                                 sendNextFrame();
                                                        }
                                                     } else 
                                                           state = stWaitingAcknowledge;
                                                      // disposing frame
                                                      disposeFrame(frame);
                                                      break;
                               }
                               
                               break;
    case stPacketReady: // a packet is ready to be handled
                        switch(frame->Ftype) {
                         case t_Ping: //  ping frame
                                      sendAcknowledge(frame->SenderID);
                                      disposeFrame(frame);
                                      state = stPacketReady;
                                      break;
                         case t_Acknowledge: // acknowledge frame
                                             // ignore
                                             disposeFrame(frame);
                                             state = stPacketReady;
                                             break;    
                         case t_Data: // a data frame
                                      // ignore until packeready state has been served
                                      disposeFrame(frame);
                                      state = stPacketReady;
                                      break;
                        }
                        break;
  } // end big state switch
  
}                    


// clear the serial buffer
void EZRoboNetDevice::flushSerialBuffer() {
  int i;
  uint16_t numBytes = CommPort->available();
  byte c;
  for (i=0; i<numBytes; i++)
    c = CommPort->read();
}

// processData. processData assembles the frames and calls to transitionAction. 
void EZRoboNetDevice::processData() {
  int i;
    
  uint16_t numBytesToRead = CommPort->available(); // let's see what we 've got...
  
  
  do {
    if (remainingBytes==0) { // previous frame was fully read
        if (numBytesToRead>=6) { // nead at least 6 bytes to read frame header
                        // get the first 6 bytes (starter(1)+sender-receiverid(2)+frame type(1)+payload length (2))
                        // reading first 6 bytes into FrameHead
                        
                        for (i=0; i<6; i++)
                          FrameHead[i] = CommPort->read();
                        // now cheking for framestarter character mismatch
                        if (FrameHead[0]!=framestarter) {
                           
                          flushSerialBuffer();
                        }
                            else {// reamaining bytes should be the payload length plus the terminator and two CRC bytes 
                              remainingBytes = FrameHead[4] + FrameHead[5] * 256 +3; 
                              
                            }
                    }
     } else if (numBytesToRead >= remainingBytes) { // it's time to get the remaining frame(s)
            int totalBytes = remainingBytes + 6; // calculate total length
            byte buffer[totalBytes];
            byte remBuffer[remainingBytes];
            // now reading remaining bytes as estimated using the frame header
            // going altogether
            for (i=0; i<remainingBytes; i++)
              remBuffer[i] = CommPort->read();
        
            // tailoring bufhead and rembuffer into buffer
        
            for (i=0; i<totalBytes; i++)
                 if (i<6) buffer[i] = FrameHead[i];
                      else buffer[i] = remBuffer[i-6];
               
            // now handling the message...
            // checking terminator and CRC
            uint16_t CRC = buffer[totalBytes-2]+256*buffer[totalBytes-1];
            
            if ((buffer[totalBytes-3]==frameterminator)&&(checkCRC(buffer, totalBytes-2, CRC))) {
              Serial.println("Frame received!");
              EZFrame* frame = bytes2Frame(buffer);
              // assembled the frame
              if (frame->ReceiverID == NodeID) // frame was addressed to this node 
                transitionAction(frame); // change internal state and act
              else // frame was not addressed to this node and being disposed of
                disposeFrame(frame);  
            } else {
                     Serial.print("Error. Frame CRC :");
                     Serial.println(CRC, DEC);
                     Serial.print("Frame terminator :");
                     Serial.println(buffer[totalBytes-3], DEC);
            }
              
        
        // clearing remaining bytes
        remainingBytes = 0;
     }        
            
        numBytesToRead = CommPort->available();
        
  } while ((numBytesToRead >= remainingBytes)&&(remainingBytes>0));

}
                                             

// processEvents consults with the state and initiates packet handling 
// additionally, if a timeout occurs, it re-initiates or terminates transmission
// does NOT RESET the flags! Also, initiates a packet transmission
void EZRoboNetDevice::processEvents() {

  if (state == stPacketReady) { 
       // the packet is assembled and ready
       // increase number of packets received
       if (ReceivedPacketsNum<2) {
         ReceivedPacketsNum++;
         ReceivedPackets[ReceivedPacketsNum-1] = bytes2Packet(CurrentPacket);
         
       } else // raise overflow flag
            PacketQueueOverflow = true;       
       // disposing CurrentPacket
       free(CurrentPacket);
       // resetting the RoboNetDevice
       state = stIdle;
       PacketReceiverID = 0;
       TotalFrames = 0;
       FramesReceived = 0;
      }
       else if (state == stWaitingAcknowledge) { // waiting for acknowledge 
                // checking time ellapsed
                unsigned long currentTime = millis();
                unsigned long transmissionElapsed = currentTime - TransmissionStartTime;
                
                if (transmissionElapsed > PACKET_SENDING_TIMEOUT)
                { // transmission timedout
                    // raising error flag
                    Serial.println("Packet Sending Timedout!");
                    PacketSendTimeout = true;
                    // quiting everything and resetting
                    free(CurrentPacket);
                    
                    PacketSenderID = 0;
                    PacketReceiverID = 0;
                    TotalFrames = 0;
                    FramesSent = 0;
                    FramesReceived = 0;
                    state = stIdle;
                }
                else
                {
                    unsigned long elapsed = currentTime - FrameTransmissionStartTime;
                    
                    if (elapsed > FRAME_ACKNOWLEDGE_TIMEOUT)
                    { // timeout occured
                    Serial.println("Acknowledge timeout!");
                        // attempting to resend the frame
                        state = stSending; // sending again
                        sendNextFrame();
                    }
                }
            }
            else if (state == stReceiving)
            { // receiving a packet
                unsigned long currentTime = millis();
                unsigned long receptionElapsed = currentTime - ReceptionStartTime;
                if (receptionElapsed > PACKET_SENDING_TIMEOUT)
                { // packet's timedout
                    // raising error flag
                    PacketReceiveTimeout = true;
                    // disposing packet
                    free(CurrentPacket);
                    // resetting
                    PacketReceiverID = 0;
                    PacketSenderID = 0;
                    TotalFrames = 0;
                    FramesSent = 0;
                    FramesReceived = 0;
                    state = stIdle;
                }
                else
                {
                    // checking time elapsed since last acknowledgement
                    unsigned long elapsed = currentTime - FrameReceptionStartTime;
                    if (elapsed > FRAME_ACKNOWLEDGE_TIMEOUT)
                    { // next frame timedout
                        // sending a new acknowledgement
                        sendAcknowledge(PacketReceiverID);
                        // reseting time
                        FrameReceptionStartTime = millis();
                        state = stReceiving;
                    }
                }
            } else if (state==stIdle) {
                      if (OutboundPacketsNum>0) { // outbound queue is not empty. must initiate transmission
                        // removing a packet from the outbound queue  
                        EZPacket* outpacket = OutboundPackets[0];
                        
                        OutboundPacketsNum--;
                        OutboundPackets[0] = NULL;
                        // shifting the Outbound queue to the left
                        int i;
                        for (i=1; i<=OutboundPacketsNum; i++) {
                            OutboundPackets[i-1] = OutboundPackets[i];
                            OutboundPackets[i] = NULL;
                        }
                        // outbound queue shifted
                        // Now, sending packet
                        sendPacket(outpacket);
                      }
            }
                        
}


// Clear flags
void EZRoboNetDevice::clearFlags() {
    PacketSendTimeout = false;
    PacketReceiveTimeout = false;
    PacketQueueOverflow = false;
  }
              
// clear PacketSendTimeout flag
void EZRoboNetDevice::clearPacketSendTimeoutFlag() {
  PacketSendTimeout = false;
}

// clear PacketReceiveTimeout Flag
void EZRoboNetDevice::clearPacketReceiveTimeoutFlag() {
  
  PacketReceiveTimeout = false;
}

// clear PacketQueueOverflow flag
void EZRoboNetDevice::clearPacketQueueOverflowFlag() {
  
  PacketQueueOverflow = false;
}
  
// number of packets available in queue             
byte EZRoboNetDevice::packetsAvailable() {

return ReceivedPacketsNum;
}


bool EZRoboNetDevice::getPacketQueueOverflow() {
 
 return PacketQueueOverflow;
}

bool EZRoboNetDevice::getPacketReceiveTimeout() {
  bool f = PacketReceiveTimeout;
  PacketReceiveTimeout = false;
  return f;
}

bool EZRoboNetDevice::getPackeSendTimeout() {
  bool f = PacketSendTimeout;
  PacketSendTimeout = false;
  return f;
}

// get a packet from the queue
EZPacket* EZRoboNetDevice::getNextAvailablePacket() {
  EZPacket* pack = NULL;
  if (packetsAvailable()==0) return NULL;
  else {
    // pack is now pointing to the allocated Packet that the queue points in position 0
    pack = ReceivedPackets[0];
    // reducing number of available packets
    ReceivedPacketsNum--;
    // clearing a possible overflow flag
    clearPacketQueueOverflowFlag();
    // "sliding/shifting" contents of queue to the left
    int i; 
    for (i=2; i>0; i--) {
      ReceivedPackets[i-1] = ReceivedPackets[i];
      if (i==2) ReceivedPackets[i] = NULL;
    }
    
    return pack;
  }
}


// clear the Packet Queue, reset the flags
void EZRoboNetDevice::resetPacketTransceiver() {
  ReceivedPacketsNum = 0;
  int i;
  for (i=0; i<2; i++) 
    if (ReceivedPackets[i]!=NULL) {
      disposePacket(ReceivedPackets[i]);
      ReceivedPackets[i] = NULL;
    }
  clearFlags();
}


// examines if the outbound packet queue is fulll
boolean EZRoboNetDevice::OutboundQueueFull() {
  return (OutboundPacketsNum >= MAX_OUTBOUND_PACKETS_NUM);
}
 
// pushes a new packet into the outbound queue
boolean EZRoboNetDevice::pushOutboundPacket(EZPacket* pack) {
  boolean success=false;
  if (!OutboundQueueFull()) {
    OutboundPacketsNum++;
    OutboundPackets[OutboundPacketsNum-1] = pack;
    success = true;
  } else 
        success = false;
  return success;
}

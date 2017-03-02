#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <strings.h>
#include <string.h>
#include <sys/socket.h> /* for socket(), bind(), and connect() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_ntoa() */
#include <pthread.h>
#include <wsuart.h>
#include <dis.h>


#define DIS_LPWMN_MAC_SHORT_ADDR_LEN  2

static cntxt_s uart_cntxt;

int writeToUART(unsigned char *buff_p, unsigned int cnt);

#define MSG_SEQ_FIELD_LEN  4
int hdrAcked = 0;
int verbose = 0;

int hdrFlags = 0;

#define SER_BUFF_LEN 1024

unsigned char serTxBuff[SER_BUFF_LEN];
unsigned char serRxBuff[SER_BUFF_LEN];

/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
unsigned short crc16(unsigned char *buff_p, unsigned int len)
{
   unsigned int ckSum = 0;

   while (len > 1)
   {
      unsigned short tmp = *buff_p;
      tmp = (tmp << 8) | (*(buff_p + 1));
      ckSum = ckSum + tmp;
      buff_p += 2;
      len -= 2;
   }

   if (len > 0)
       ckSum += (*buff_p);

   while (ckSum >> 16)
   {
      ckSum = (ckSum & 0xffff) + (ckSum >> 16);
   }

   return (~ckSum);
}

           
#ifdef __CYGWIN__
    

/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int cfgPort(char *serDevName_p, int baudRate)
{
    struct termios newtio;
    struct termios oldtio;
    struct termios latesttio;
    cntxt_s *serialCntxt_p = &uart_cntxt;
    int rc;

    serialCntxt_p->serialFd = open(serDevName_p, O_RDWR | O_NOCTTY );
    if (serialCntxt_p->serialFd < 0)
    {
        printf("Failed to open serial device <%s> - errno<%d> !!\n",
               serDevName_p, errno);  
        return -1;
    }

    // printf("Opened serial device <%s> \n", serDevName_p); 
#if 0 
    rc = tcgetattr(serialCntxt_p->serialFd, &oldtio); /* save current port settings */
    if (rc < 0)
    {
        printf("\n tcgetattr() failed !! - rc<%d>, errno<%d> \n", rc, errno);
        return -1;
    }
#endif
    bzero(&newtio, sizeof(newtio));

    rc = cfsetspeed(&newtio, baudRate);
    if (rc < 0)
    {
        printf("\n cfsetspeed() failed !! - rc<%d>, errno<%d> \n", rc, errno);
        return -1;
    }

    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // if ((rc = fcntl(serialCntxt_p->serialFd, F_SETOWN, getpid())) < 0)
    // {
    //     printf("\n fcntl failed !! - rc<%d>, errno<%d> \n", rc, errno);
    //     return -1;
    // }

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 10 chars received */

    rc = tcsetattr(serialCntxt_p->serialFd, TCSANOW, &newtio);
    if (rc < 0)
    {
        printf("\n tcsetattr() failed !! - rc<%d> / errno<%d> \n", rc, errno);
        return -1;
    }

    rc = tcflush(serialCntxt_p->serialFd, TCIFLUSH);
    if (rc < 0)
    {
        printf("\n tcflush() failed !! - rc<%d> \n", rc);
        return -1;
    }
    
    tcgetattr(serialCntxt_p->serialFd, &latesttio); 
    if (rc < 0)
    {
        printf("\n tcgetattr() failed !! - rc<%d> \n", rc);
        return -1;
    }

    // printf("\nispeed<%d> / ospeed<%d> \n", latesttio.c_ispeed, latesttio.c_ospeed);
    // printf("\niflag<0x%x>/oflag<0x%x>/cflag<0x%x> \n", latesttio.c_iflag, latesttio.c_oflag, latesttio.c_cflag);

    return 1;
}



#else 
                             
/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int cfgPort(char *serDevName_p, int baudRate)
{
   cntxt_s *serialCntxt_p = &uart_cntxt;
   int rc;

   memset(serialCntxt_p, 0, sizeof(cntxt_s));

   serialCntxt_p->serialFd = open((char *)serDevName_p, O_RDWR | O_NOCTTY | O_NDELAY);
   if (serialCntxt_p->serialFd < 0)
   {
       printf("\n<%s> open(%s) failed !! - errno<%d> \n",
              __FUNCTION__, serDevName_p, errno);
       return -1;
   }
   
   // Zero out port status flags
   if (fcntl(serialCntxt_p->serialFd, F_SETFL, 0) != 0x0)
   {
       return -1;
   }

   bzero(&(serialCntxt_p->dcb), sizeof(serialCntxt_p->dcb));

   // serialCntxt_p->dcb.c_cflag |= serialCntxt_p->baudRate;  // Set baud rate first time
   serialCntxt_p->dcb.c_cflag |= baudRate;  // Set baud rate first time
   serialCntxt_p->dcb.c_cflag |= CLOCAL;  // local - don't change owner of port
   serialCntxt_p->dcb.c_cflag |= CREAD;  // enable receiver

   // Set to 8N1
   serialCntxt_p->dcb.c_cflag &= ~PARENB;  // no parity bit
   serialCntxt_p->dcb.c_cflag &= ~CSTOPB;  // 1 stop bit
   serialCntxt_p->dcb.c_cflag &= ~CSIZE;  // mask character size bits
   serialCntxt_p->dcb.c_cflag |= CS8;  // 8 data bits

   // Set output mode to 0
   serialCntxt_p->dcb.c_oflag = 0;
 
   serialCntxt_p->dcb.c_lflag &= ~ICANON;  // disable canonical mode
   serialCntxt_p->dcb.c_lflag &= ~ECHO;  // disable echoing of input characters
   serialCntxt_p->dcb.c_lflag &= ~ECHOE;
 
   // Set baud rate
   serialCntxt_p->baudRate = baudRate;  
   cfsetispeed(&serialCntxt_p->dcb, serialCntxt_p->baudRate);
   cfsetospeed(&serialCntxt_p->dcb, serialCntxt_p->baudRate);

   serialCntxt_p->dcb.c_cc[VTIME] = 0;  // timeout = 0.1 sec
   serialCntxt_p->dcb.c_cc[VMIN] = 1;
 
   if ((tcsetattr(serialCntxt_p->serialFd, TCSANOW, &(serialCntxt_p->dcb))) != 0)
   {
       printf("\ntcsetattr(%s) failed !! - errno<%d> \n",
              serDevName_p, errno);
       close(serialCntxt_p->serialFd);
       return -1;
   }

   // flush received data
   tcflush(serialCntxt_p->serialFd, TCIFLUSH);
   tcflush(serialCntxt_p->serialFd, TCOFLUSH);

   return 1;
}

#endif


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int writeToUART(unsigned char *buff_p, unsigned int cnt)
{
   int rc, bytesLeft = cnt, bytesWritten = 0;
    
   // printf("\n<%s> cnt<%d> \n", __FUNCTION__, cnt);
   
   while (bytesLeft > 0)
   {
      rc = write(uart_cntxt.serialFd, buff_p + bytesWritten, bytesLeft);
      if (rc <= 0)
          return -1;
      else
      {
          bytesLeft -= rc;
          bytesWritten += rc;
      }
   }

   return 1;
}
                   
            
/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
void print_usage()
{
    printf("Usage: ./stream.exe <serial-device> msg_string(max len 111 chars) \n");
    printf("On Cywgin, the serial port device is named /dev/ttySX if the COM port is COMY where X = Y - 1 \n");
    printf("Example: ./stream.exe /dev/ttyS20 hello!\n");
}




/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
void ___htonl(unsigned char *buff_p, unsigned int val)
{
   buff_p[0] = (val >> 24) & 0xff;
   buff_p[1] = (val >> 16) & 0xff;
   buff_p[2] = (val >> 8) & 0xff;
   buff_p[3] = (val) & 0xff;
}


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
void ___htons(unsigned char *buff_p, unsigned short val)
{
   buff_p[0] = (val >> 8) & 0xff;
   buff_p[1] = (val) & 0xff;
}


int readPortTmo(unsigned char *buff_p, unsigned int len, int timeOutInMilliSecs)
{
   int rc;
   fd_set set;
   struct timeval timeout;

   FD_ZERO(&set); /* clear the set */
   FD_SET(uart_cntxt.serialFd, &set); /* add our file descriptor to the set */

   timeout.tv_sec = (timeOutInMilliSecs / 1000);
   timeout.tv_usec = (timeOutInMilliSecs % 1000) * 1000;

   rc = select(uart_cntxt.serialFd + 1, &set, NULL, NULL, &timeout);
   if (rc == -1)
   {
       printf("\n<%s> select() failed - errno<%d> !! \n", __FUNCTION__, errno);
   }
   else
   {
       // if rc is 0, select has timed out !!
       if (rc > 0)
       {
          rc = readPort(buff_p, len);
          return rc;
       }
   }

   return rc;
}


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int GW_buildSendHdr(int msgType, unsigned char *pyldBuff_p, int pyldLen)
{
   unsigned char *buff_p = serTxBuff;
   unsigned short calcCrc16;
   static unsigned char seqNr = 0x0;
   int rc;
  
   if (verbose) 
       printf("<%s> msgType<0x%x> \n", __FUNCTION__, msgType);

   ___htons(buff_p, msgType);
   buff_p += UART_FRAME_HDR_MSG_TYPE_FIELD_LEN;

   *buff_p = 0x0;
   buff_p += UART_FRAME_HDR_FLAGS_FIELD_LEN;

   *buff_p = seqNr ++;
   buff_p += UART_FRAME_HDR_SEQ_NR_FIELD_LEN;

   ___htons(buff_p, pyldLen);
   buff_p += UART_FRAME_HDR_PYLD_LEN_FIELD_LEN;

   calcCrc16 = crc16(serTxBuff, UART_FRAME_HDR_HDR_CRC_FIELD_OFF);
   ___htons(buff_p, calcCrc16);  // no payload
   buff_p += UART_FRAME_HDR_HDR_CRC_FIELD_LEN;

   if (pyldLen > 0)
   {
       calcCrc16 = crc16(pyldBuff_p, pyldLen);
       ___htons(buff_p, calcCrc16);  // payload crc 
   }
   else
       ___htons(buff_p, 0x0);  // no payload 

   if (verbose)
   {
       int idx;
       
       printf("\n -------------------------- \n");

       for (idx=0; idx<UART_FRAME_HDR_LEN; idx++)
            printf(" 0x%02x ", serTxBuff[idx]);

       printf("\n -------------------------- \n");
   }

   rc = writePort(serTxBuff, UART_FRAME_HDR_LEN);
   if (rc != 1)
   {
       printf("\nwritePort() failed !!\n");
       rc = 20;
   }

   if (verbose) 
       printf("\nwritePort() done !!\n");

   return rc;
}


int writePort(unsigned char *buff_p, unsigned int cnt)
{
   int rc, bytesLeft = cnt, bytesWritten = 0;
    
   // printf("\n<%s> cnt<%d> \n", __FUNCTION__, cnt);
   
   while (bytesLeft > 0)
   {
      rc = write(uart_cntxt.serialFd, buff_p + bytesWritten, bytesLeft);
      if (rc <= 0)
          return -1;
      else
      {
          bytesLeft -= rc;
          bytesWritten += rc;
      }
   }

   return 1;
}


int readPort(unsigned char *buff_p, unsigned int len)
{
   int rdLen, readLeft = len, totRead = 0;

   while (readLeft > 0)
   {
      rdLen = read(uart_cntxt.serialFd, buff_p + totRead, readLeft);
      
      if (verbose)
          printf("\n<%s> rdLen<%d> \n", __FUNCTION__, rdLen);

      if (rdLen > 0)
      {
          totRead += rdLen;
          readLeft -= rdLen;
      }
      else
      {
          printf("\n<%s> read() failed  - %d !! \n", __FUNCTION__, rdLen);
          return rdLen;
      }
   }

   return totRead;
}

/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int GW_processRcvdMsg(unsigned char *buff_p, int offset, int currMsgType, int pyldLen)
{
   int rc = 0;

   if (verbose)
       printf("<%s> msgType<0x%x> / pyldLen <%d> / offset <%d> \n", 
              __FUNCTION__, currMsgType, pyldLen,  offset);

   if (verbose)
   {
       int idx;
        
       for (idx = 0; idx<pyldLen; idx++)
       {
            if (idx % 8 == 0)
                printf("\n <%s> : ", __FUNCTION__);
            printf(" 0x%02x ", buff_p[offset + idx]);
       }
       printf("\n");
   }

   switch (currMsgType)
   {
      case UART_MSG_TYPE_STREAM_TX_STS:
           {
              hdrFlags = buff_p[UART_FRAME_HDR_FLAGS_FIELD_OFF];
              rc = 1;
	   }
	   break;

      case UART_MSG_TYPE_ACK:
           {
              hdrFlags = buff_p[UART_FRAME_HDR_FLAGS_FIELD_OFF];
              if (verbose)
                  printf("Hdr Flags <0x%x> \n", hdrFlags);
              if (hdrFlags & UART_ACK_STS_OK_BM)
                  hdrAcked = 1;
              else
              {
                  printf("<%s> hdr flags <0x%x> \n", __FUNCTION__, hdrFlags);
                  hdrAcked = 0;
              }
              rc = 1;
           }
           break;

      default:
           {
             printf("Message type<%d> not handled !! \n", currMsgType);
           }
           break;
   }

   return rc;
}

/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int GW_readSerIntf(int expMsgType, int tmoSecs)
{
   int rc = 0, off = 0, readLen, pyldLen, totReadLen = 0;
   int currMsgType = 0xffff, done = 0;
   time_t startTime;

   time(&startTime); 

   readLen = UART_FRAME_HDR_PYLD_CRC_FIELD_OFF;
   memset(serRxBuff, 0, sizeof(serRxBuff));

   do
   {
      if (verbose) 
          printf("\noff<%d>/readLen<%d>/totReadLen<%d> \n", off, readLen, totReadLen);

      if (tmoSecs > 0)
      {
          rc = readPortTmo(serRxBuff + off, readLen, tmoSecs*1000);
          if (tmoSecs > 0)
          {
              if (rc == 0)
              {
                  time_t currTime;
    
                  // No data read from port !!
                  time(&currTime);
                  if (currTime - startTime > tmoSecs)
                      return 100;
                  continue;
              }
          }
      }
      else
      {
          rc = readPort(serRxBuff + off, readLen);
      }

      if (rc != readLen)
      {
          printf("\nreadPort() failed !! ................. \n");
          close(uart_cntxt.serialFd);
          rc = 10;
          break;
      }

      totReadLen += readLen;
      off += readLen;

      switch (totReadLen)
      {
         case UART_FRAME_HDR_PYLD_CRC_FIELD_OFF: 
              {
                  int idx;
                  unsigned short calcCrc16, rxdCrc16;

                  // Get the message length

                  if (verbose)
                  {
                      printf("\nRead <%d> bytes --- ", readLen);
                      for (idx=0; idx<totReadLen; idx++)
                           printf(" <0x%x> ", serRxBuff[idx]);
                  }

                  calcCrc16 = crc16(serRxBuff, UART_FRAME_HDR_HDR_CRC_FIELD_OFF);
                  rxdCrc16 = serRxBuff[UART_FRAME_HDR_HDR_CRC_FIELD_OFF];
                  rxdCrc16 = (rxdCrc16 << 8) + serRxBuff[UART_FRAME_HDR_HDR_CRC_FIELD_OFF + 1];
      
                  if (verbose)
                      printf("\ncalc-crc16<0x%x> rcvd-crc16<0x%x>\n", calcCrc16, rxdCrc16);

                  if (calcCrc16 != rxdCrc16)
                  {
                      for (idx=0; idx<UART_FRAME_HDR_PYLD_CRC_FIELD_OFF-1; idx++)
                           serRxBuff[idx] = serRxBuff[idx+1];
                      off = UART_FRAME_HDR_PYLD_CRC_FIELD_OFF - 1;
                      readLen = 1;
                      totReadLen = off;
                      break;
                  }

                  pyldLen = serRxBuff[UART_FRAME_HDR_PYLD_LEN_FIELD_OFF];
                  pyldLen = (pyldLen << 8) |  serRxBuff[UART_FRAME_HDR_PYLD_LEN_FIELD_OFF + 1];

                  currMsgType = serRxBuff[UART_FRAME_HDR_MSG_TYPE_FIELD_OFF];
                  currMsgType = (currMsgType << 8) | serRxBuff[UART_FRAME_HDR_MSG_TYPE_FIELD_OFF + 1];

                  if (verbose)
                      printf("\nMessage Type<%d> / Length<%d>", currMsgType, pyldLen);                   

                  readLen = pyldLen + UART_FRAME_HDR_PYLD_CRC_FIELD_LEN;
              }
              break;

         default:
              {
                  if (currMsgType == expMsgType)
                  {
                      if (GW_processRcvdMsg(serRxBuff, UART_FRAME_HDR_LEN, currMsgType, pyldLen) == 1)
                          done = 1;
                  }
        
                  readLen = UART_FRAME_HDR_PYLD_CRC_FIELD_OFF;
                  memset(serRxBuff, 0, sizeof(serRxBuff));
                  totReadLen = 0;
                  off = 0;

                  if (tmoSecs > 0 && done == 0)
                  {
                      time_t currTime;
                      time(&currTime);
                      if (currTime - startTime > tmoSecs)
                          return 100;
                  }
              }
              break;
      }
   } while (done == 0x0);

   if (done == 1)
       rc = 0;
   else
       rc = 6;

   return rc;
}


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int __sendMsg(int destAddr, int seqNr, int pyldLen)
{
   int rc, msgLen;
   unsigned char *msg_p; 

   printf("\n------------------------------------------------------------ \n");
   printf("Sending msg<%d> of length<%d> to node<%d> ... \n",
          seqNr, pyldLen, destAddr);

   // Sets header of length 6 bytes
   msgLen = DIS_LPWMN_MAC_SHORT_ADDR_LEN + MSG_SEQ_FIELD_LEN + pyldLen;
   
   msg_p = (unsigned char *)malloc(msgLen);
   if (msg_p == NULL)
   {
       printf("malloc(%d) failed !! \n", pyldLen);
       return -1;
   }

   ___htons(msg_p, destAddr);  

   ___htonl(msg_p + DIS_LPWMN_MAC_SHORT_ADDR_LEN, seqNr);  

   memset(msg_p + DIS_LPWMN_MAC_SHORT_ADDR_LEN + MSG_SEQ_FIELD_LEN, 'a' + (seqNr%26), pyldLen);
   
   // Print the message
   for (int i = 0; i < pyldLen; ++i)
   {
     printf("->%c\n", msg_p[6+i]);
   }

   rc = GW_buildSendHdr(DIS_MSG_TYPE_STREAM_SEGMENT, msg_p, msgLen);
   if (rc != 1)
       return rc;

   printf("Sent hdr to attached node .... \n", msgLen);

   rc = GW_readSerIntf(UART_MSG_TYPE_ACK, 0);
   if (rc != 0)
       return rc;

   if (hdrAcked == 0x0)
   {
       printf("Ack indicates failure <0x%x>!! \n", hdrFlags);
       return -2;
   }

   printf("Rcvd hdr ack ... \n");

   // Send payload
   rc = writePort(msg_p, msgLen);
   if (rc != 1)
   {
       printf("writePort(%d) failed !! \n", msgLen);
       return -3;
   }
   else
       printf("Sent msg of total length<%u> to attached node ... \n", pyldLen);
   
   rc = GW_readSerIntf(UART_MSG_TYPE_STREAM_TX_STS, 0);

   if (hdrFlags == 0)
   {
       rc = 0;
       printf("Message could not be sent to %d \n", destAddr);
   }
   else
   {
       printf("Message sent ....  \n", destAddr);
       rc = 1;
   }

   printf("rcvd msg ack  ....\n");
   printf("\n------------------------------------------------------------ \n");
       
   return rc;
}


int __sendMsg_string(int destAddr, int pyldLen, char *msg_string)
{
   int rc, msgLen;
   unsigned char *msg_p; 

   printf("\n------------------------------------------------------------ \n");
   // printf("Sending msg<%d> of length<%d> to node<%d> ... \n",
   //        1, pyldLen, destAddr);

   // Sets header of length 6 bytes
   msgLen = DIS_LPWMN_MAC_SHORT_ADDR_LEN + MSG_SEQ_FIELD_LEN + pyldLen;
   
   msg_p = (unsigned char *)malloc(msgLen);
   if (msg_p == NULL)
   {
       printf("malloc(%d) failed !! \n", pyldLen);
       return -1;
   }

   ___htons(msg_p, destAddr);  

   ___htonl(msg_p + DIS_LPWMN_MAC_SHORT_ADDR_LEN, 1);  

   // memset(msg_p + DIS_LPWMN_MAC_SHORT_ADDR_LEN + MSG_SEQ_FIELD_LEN, 'a' + (seqNr%26), pyldLen);

   // Store the message
   for (int i = 0; i < pyldLen; ++i)
   {
     msg_p[6 + i] = msg_string[i];
     // printf("->%c\n", msg_p[6+i]);
   }
   
   // Print the message
   for (int i = 0; i < pyldLen; ++i)
   {
     printf("->%c\n", msg_p[6+i]);
   }

   rc = GW_buildSendHdr(DIS_MSG_TYPE_STREAM_SEGMENT, msg_p, msgLen);
   if (rc != 1)
       return rc;

   printf("Sent hdr to attached node .... \n", msgLen);

   rc = GW_readSerIntf(UART_MSG_TYPE_ACK, 0);
   if (rc != 0)
       return rc;

   if (hdrAcked == 0x0)
   {
       printf("Ack indicates failure <0x%x>!! \n", hdrFlags);
       return -2;
   }

   printf("Rcvd hdr ack ... \n");

   // Send payload
   rc = writePort(msg_p, msgLen);
   if (rc != 1)
   {
       printf("writePort(%d) failed !! \n", msgLen);
       return -3;
   }
   else
       printf("Sent msg of total length<%u> to attached node ... \n", pyldLen);
   
   rc = GW_readSerIntf(UART_MSG_TYPE_STREAM_TX_STS, 0);

   if (hdrFlags == 0)
   {
       rc = 0;
       printf("Message could not be sent to %d \n", destAddr);
   }
   else
   {
       printf("Message sent ....  \n", destAddr);
       rc = 1;
   }

   printf("rcvd msg ack  ....\n");
   printf("\n------------------------------------------------------------ \n");
       
   return rc;
}


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int main(int argc, const char* argv[])
{
    int rc = 0, idx, msgCnt = 0, msgLen = 0;

    if (argc < 3)
    {
        print_usage();
        return 1;
    }

    if (cfgPort((char *)argv[1], B9600) < 0)
        return 2;

    char msg_str[111];
    int msg_len;

    strcpy(msg_str, argv[2]);
    msg_len = strlen(msg_str);

    __sendMsg_string(1, msg_len, msg_str);

    // printf("%d\n", strlen(msg_str));

 //    msgCnt = atoi(argv[3]);
 //    if (msgCnt <= 0)
 //    {
 //        printf("msgcnt should be > 0 !! \n");
	// return 1;
 //    }

 //    msgLen = atoi(argv[2]);
 //    if (msgLen <= 0 || msgLen > 111) 
 //    {
 //        printf("msgLen should be > 0 && <= 111 !! \n");
	// return 2;
 //    }

 //    for (idx=0; idx<msgCnt; idx++)
 //    {
 //         if (__sendMsg(1, idx + 1, msgLen) <= 0)
 //             break;
 //    }

 //    printf("\ndone \n");
     
    return 0;
}

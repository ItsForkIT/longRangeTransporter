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
#include <wsuart.h>
#include <dis.h>
#include <gw.h>

static cntxt_s uart_cntxt;
int hdrAcked = 1;
int verbose = 0;

unsigned char serTxBuff[1024];
unsigned char serRxBuff[1024];

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
int writePort(unsigned char *buff_p, int cnt)
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
int readPort(unsigned char *buff_p, int len)
{
   int rdLen, readLeft = len, totRead = 0;

   while (readLeft > 0)
   {
      rdLen = read(uart_cntxt.serialFd, buff_p + totRead, readLeft);
      // printf("\n<%s> rdLen<%d> \n", __FUNCTION__, rdLen);
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
int __buildSendHdr(int msgType, unsigned char *pyldBuff_p, int pyldLen)
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
                   

/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int __readSerIntf(int expMsgType)
{
   int rc = 1, off = 0, readLen, pyldLen, totReadLen = 0;
   int currMsgType = 0xffff, done = 0;

   readLen = UART_FRAME_HDR_PYLD_CRC_FIELD_OFF;
   memset(serRxBuff, 0, sizeof(serRxBuff));

   do
   {
      if (verbose)
          printf("\noff<%d>/readLen<%d>/totReadLen<%d> \n", off, readLen, totReadLen);

      rc = readPort(serRxBuff + off, readLen);
      if (rc != readLen)
      {
          printf("\nreadPort() failed !! ................. \n");
          close(uart_cntxt.serialFd);
          rc = 0;
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
            	  // At this point we have received the 2 bytes of payload CRC and the payload
            	  // (if the payload was sent).
                  if (currMsgType == UART_MSG_TYPE_ACK)
                  {
                      unsigned char hdrFlags = serRxBuff[UART_FRAME_HDR_FLAGS_FIELD_OFF];
                      if (verbose)
                          printf("Hdr Flags <0x%x> \n", hdrFlags);
                      if (hdrFlags & UART_ACK_STS_OK_BM)
                          hdrAcked = 1;
                      else
                      {
                          printf("<%s> hdr flags <0x%x> \n", __FUNCTION__, hdrFlags);
			  if (hdrFlags & UART_ACK_STS_RELAY_IN_PROGRESS)
                              printf("Relay in progress !!\n");
			  else
                          hdrAcked = 0;
                      }
                      done = 1;
                  }
                  else
                  {
                	  // Read next header
                      readLen = UART_FRAME_HDR_PYLD_CRC_FIELD_OFF;
                      memset(serRxBuff, 0, sizeof(serRxBuff));
                      totReadLen = 0;
                      off = 0;
                  }
              }
              break;
      }
   } while (done == 0x0);

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
void __send(int shortAddr, int infoLen, unsigned char *infoBuff_p)
{
   int off, rc = 1, pyldLen;
   unsigned char *pyld_p;
  
   pyldLen = LPWMN_MAC_SHORT_ADDR_LEN
             + DIS_MSG_TYPE_SZ
	     + infoLen;

   pyld_p = (unsigned char *)malloc(pyldLen);
   if (pyld_p == NULL)
   {
       printf("malloc(%d) failed !! \n", pyldLen);
       return;
   }
  
   ___htons(pyld_p, shortAddr);
   off = LPWMN_MAC_SHORT_ADDR_LEN;
   pyld_p[off ++] = DIS_MSG_TYPE_STREAM_SEGMENT;

   memcpy(pyld_p + off, infoBuff_p, infoLen);

   // Send header 
   rc = __buildSendHdr(LPWMN_GW_MSG_TYPE_RELAY_TO_NODE, 
                       pyld_p, pyldLen);
   if (rc != 1)
   {
       return;
   }
       
   // wait for coordinator to ack the header 
   __readSerIntf(UART_MSG_TYPE_ACK);
   if (rc != 1)
   {
       return;
   }

   if (hdrAcked == 0x0)
   {
       return;
   }
      
   printf("Header acked  .... \n");
   printf("sending payload .... \n");

   // Send payload
   rc = writePort(pyld_p, pyldLen);
   if (rc != 1)
   {
       printf("<%s> writePort(%d) failed !! \n", 
              __FUNCTION__, pyldLen);
       return;
   }
     
   printf("Msg sent to coord .... \n");
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
    printf("Usage: ./wsnd.exe <serial-device> <short_addr> <pyld_len>\n");
    printf("On Cywgin, the serial port device is named /dev/ttySX if the COM port is COMY where X = Y - 1 \n");
    printf("Example: ./wsnd.exe /dev/ttyS20 100 32\n");
}


unsigned char infoBuff[64];


/*
 ********************************************************************
 *
 *
 *
 *
 ********************************************************************
 */
int main(int argc, const char* argv[] )
{
    int rc = 0, idx, dstShortAddr = -1, infoLen = -1;
    char *message;
    if (argc < 4)
    {
        print_usage();
        return 1;
    }

    if (cfgPort((char *)argv[1], B38400) < 0)
        return 2;

    if (sscanf(argv[2], "%d", &dstShortAddr) != 1
        || dstShortAddr <= 1 || dstShortAddr >= 32767)
    {
        printf("Please enter valid short address of the intended recepient !! \n",
               dstShortAddr);
	return 2;
    }

    if (verbose)
        printf("dsa: %d  \n", dstShortAddr);

    infoLen = strlen(argv[3]) + 1;

    message = (char *)malloc(infoLen + 1);

    strcpy(message, argv[3]);

    printf("->%s\n", message);
    printf("->%d\n", infoLen);

    if (verbose)
        printf("infoLen: %d  \n", infoLen);

    if(infoLen > 80){
      return 3;
    }

    __send(dstShortAddr, infoLen, message);

    return 0;
}

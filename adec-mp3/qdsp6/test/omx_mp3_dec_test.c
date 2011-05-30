/*--------------------------------------------------------------------------
Copyright (c) 2009, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------*/


/*
    An Open max test application ....
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <time.h>
#include <sys/ioctl.h>
#include "OMX_Core.h"
#include "OMX_Component.h"
#include "pthread.h"
#include <signal.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include<unistd.h>
#include<string.h>
#include <pthread.h>

#include <linux/ioctl.h>



uint32_t pcmplayback = 0;
uint32_t tunnel      = 0;
uint32_t filewrite   = 0;
uint32_t flushinprogress = 0;

OMX_U32 mp3_frequency_index[3][4] = {
  { 11025,0,22050,44100},
  {12000,0,24000,48000},
  {8000,0,16000,32000}
};
#define DEBUG_PRINT printf



#define PCM_PLAYBACK /* To write the pcm decoded data to the msm_pcm device for playback*/

#ifdef PCM_PLAYBACK
  int                          m_pcmdrv_fd;

struct msm_audio_pcm_config {
  uint32_t buffer_size;
  uint32_t buffer_count;
  uint32_t channel_count;
  uint32_t sample_rate;
  uint32_t type;
  uint32_t unused[3];
};

struct mp3_header
{
    OMX_U8 sync;
    OMX_U8 version;
    uint8_t Layer;
    OMX_U8 protection;
    OMX_U32  bitrate;
    OMX_U32 sampling_rate;
    OMX_U8 padding;
    OMX_U8 private_bit;
    OMX_U8 channel_mode;
};
OMX_U8* pBuffer_tmp = NULL;
#define DEFAULT_SAMPLING_RATE  44100
#define DEFAULT_CHANNEL_MODE   2

#define AUDIO_IOCTL_MAGIC 'a'
#define AUDIO_START        _IOW(AUDIO_IOCTL_MAGIC, 0, unsigned)
#define AUDIO_STOP         _IOW(AUDIO_IOCTL_MAGIC, 1, unsigned)
#define AUDIO_FLUSH        _IOW(AUDIO_IOCTL_MAGIC, 2, unsigned)
#define AUDIO_GET_CONFIG   _IOR(AUDIO_IOCTL_MAGIC, 3, unsigned)
#define AUDIO_SET_CONFIG   _IOW(AUDIO_IOCTL_MAGIC, 4, unsigned)
#define AUDIO_GET_STATS    _IOR(AUDIO_IOCTL_MAGIC, 5, unsigned)

#endif  // PCM_PLAYBACK


/************************************************************************/
/*        #DEFINES                          */
/************************************************************************/
#define false 0
#define true 1

#define CONFIG_VERSION_SIZE(param) \
  param.nVersion.nVersion = CURRENT_OMX_SPEC_VERSION;\
  param.nSize = sizeof(param);

#define FAILED(result) (result != OMX_ErrorNone)

#define SUCCEEDED(result) (result == OMX_ErrorNone)

/************************************************************************/
/*        GLOBAL DECLARATIONS                     */
/************************************************************************/

pthread_mutex_t lock;
pthread_cond_t cond;
pthread_mutex_t elock;
pthread_cond_t econd;
//pthread_mutex_t flock;
pthread_cond_t fcond;
FILE * inputBufferFile = NULL;
FILE * outputBufferFile;
OMX_PARAM_PORTDEFINITIONTYPE inputportFmt;
OMX_PARAM_PORTDEFINITIONTYPE outputportFmt;
OMX_AUDIO_PARAM_PCMMODETYPE pcmParam;
OMX_AUDIO_PARAM_MP3TYPE mp3param;
OMX_PORT_PARAM_TYPE portParam;
OMX_ERRORTYPE error;
struct mp3_header mp3Header;

OMX_ERRORTYPE  parse_mp3_frameheader(OMX_BUFFERHEADERTYPE* buffer,
                                     struct mp3_header *header);

/* http://ccrma.stanford.edu/courses/422/projects/WaveFormat/ */

#define ID_RIFF 0x46464952
#define ID_WAVE 0x45564157
#define ID_FMT  0x20746d66
#define ID_DATA 0x61746164

#define FORMAT_PCM 1

static bFileclose = 0;
int bReconfigureOutputPort = 0;
int bParseHeader = 0;
int bConfigureOmxComp = 0;

struct wav_header {
  uint32_t riff_id;
  uint32_t riff_sz;
  uint32_t riff_fmt;
  uint32_t fmt_id;
  uint32_t fmt_sz;
  uint16_t audio_format;
  uint16_t num_channels;
  uint32_t sample_rate;
  uint32_t byte_rate;       /* sample_rate * num_channels * bps / 8 */
  uint16_t block_align;     /* num_channels * bps / 8 */
  uint16_t bits_per_sample;
  uint32_t data_id;
  uint32_t data_sz;
};

static unsigned totaldatalen = 0;

/************************************************************************/
/*        GLOBAL INIT              */
/************************************************************************/

int input_buf_cnt = 0;
int output_buf_cnt = 0;
int used_ip_buf_cnt = 0;
volatile int event_is_done = 0;
volatile int ebd_event_is_done = 0;
volatile int fbd_event_is_done = 0;
int ebd_cnt;
int bOutputEosReached = 0;
int bInputEosReached = 0;
int bEosOnInputBuf = 0;
int bEosOnOutputBuf = 0;
int bFlushing = false;
int bPause    = false;
const char *in_filename;


static int pcm_play(unsigned rate, unsigned channels,
                    int (*fill)(void *buf, unsigned sz, void *cookie),
                    void *cookie);

static char *next;
static unsigned avail;

int timeStampLfile = 0;
int timestampInterval = 100;

//* OMX Spec Version supported by the wrappers. Version = 1.1 */
const OMX_U32 CURRENT_OMX_SPEC_VERSION = 0x00000101;
OMX_COMPONENTTYPE* mp3_dec_handle = 0;

OMX_BUFFERHEADERTYPE  **pInputBufHdrs = NULL;
OMX_BUFFERHEADERTYPE  **pOutputBufHdrs = NULL;

/************************************************************************/
/*        GLOBAL FUNC DECL                        */
/************************************************************************/
int Init_Decoder(char*);
int Play_Decoder();
OMX_STRING aud_comp;

/**************************************************************************/
/*        STATIC DECLARATIONS                       */
/**************************************************************************/

static int open_audio_file ();
static int Read_Buffer(OMX_BUFFERHEADERTYPE  *pBufHdr );
static OMX_ERRORTYPE Allocate_Buffer ( OMX_COMPONENTTYPE *mp3_dec_handle,
                                       OMX_BUFFERHEADERTYPE  ***pBufHdrs,
                                       OMX_U32 nPortIndex,
                                       long bufCntMin, long bufSize);


static OMX_ERRORTYPE EventHandler(OMX_IN OMX_HANDLETYPE hComponent,
                                  OMX_IN OMX_PTR pAppData,
                                  OMX_IN OMX_EVENTTYPE eEvent,
                                  OMX_IN OMX_U32 nData1, OMX_IN OMX_U32 nData2,
                                  OMX_IN OMX_PTR pEventData);
static OMX_ERRORTYPE EmptyBufferDone(OMX_IN OMX_HANDLETYPE hComponent,
                                     OMX_IN OMX_PTR pAppData,
                                     OMX_IN OMX_BUFFERHEADERTYPE* pBuffer);

static OMX_ERRORTYPE FillBufferDone(OMX_IN OMX_HANDLETYPE hComponent,
                                     OMX_IN OMX_PTR pAppData,
                                     OMX_IN OMX_BUFFERHEADERTYPE* pBuffer);

void wait_for_event(void)
{
    pthread_mutex_lock(&lock);
    DEBUG_PRINT("%s: event_is_done=%d", __FUNCTION__, event_is_done);
    while (event_is_done == 0) {
        pthread_cond_wait(&cond, &lock);
    }
    event_is_done = 0;
    pthread_mutex_unlock(&lock);
}

void event_complete(void )
{
    pthread_mutex_lock(&lock);
    if (event_is_done == 0) {
        event_is_done = 1;
        pthread_cond_broadcast(&cond);
    }
    pthread_mutex_unlock(&lock);
}


OMX_ERRORTYPE EventHandler(OMX_IN OMX_HANDLETYPE hComponent,
                           OMX_IN OMX_PTR pAppData,
                           OMX_IN OMX_EVENTTYPE eEvent,
                           OMX_IN OMX_U32 nData1, OMX_IN OMX_U32 nData2,
                           OMX_IN OMX_PTR pEventData)
{
   DEBUG_PRINT("Function %s \n command %d  Event complete %d", __FUNCTION__,(OMX_COMMANDTYPE)nData1,nData2);

    switch(eEvent) {
      case OMX_EventCmdComplete:
         DEBUG_PRINT("*********************************************\n");
         DEBUG_PRINT("\n OMX_EventCmdComplete \n");
         DEBUG_PRINT("*********************************************\n");
         if(OMX_CommandPortDisable == (OMX_COMMANDTYPE)nData1) {
            DEBUG_PRINT("******************************************\n");
            DEBUG_PRINT("Recieved DISABLE Event Command Complete[%d]\n",nData2);
            DEBUG_PRINT("******************************************\n");
         }
         else if(OMX_CommandPortEnable == (OMX_COMMANDTYPE)nData1) {
            DEBUG_PRINT("*********************************************\n");
            DEBUG_PRINT("Recieved ENABLE Event Command Complete[%d]\n",nData2);
            DEBUG_PRINT("*********************************************\n");
         }
         else if(OMX_CommandFlush== (OMX_COMMANDTYPE)nData1)
         {
             DEBUG_PRINT("*********************************************\n");
             DEBUG_PRINT("Recieved FLUSH Event Command Complete[%d]\n",nData2);
             DEBUG_PRINT("*********************************************\n");
         }
         event_complete();
       break;

       case OMX_EventError:
         DEBUG_PRINT("*********************************************\n");
         DEBUG_PRINT("\n OMX_EventError \n");
         DEBUG_PRINT("*********************************************\n");
       break;

       case OMX_EventPortSettingsChanged:
          if(tunnel == 0)
          {
              bReconfigureOutputPort = 1;
              DEBUG_PRINT("*********************************************\n");
            DEBUG_PRINT("\n OMX_EventPortSettingsChanged \n");
              DEBUG_PRINT("*********************************************\n");
              event_complete();
          }
       break;

       case OMX_EventBufferFlag:
         DEBUG_PRINT("\n *********************************************\n");
         DEBUG_PRINT("\n OMX_EventBufferFlag \n");
         DEBUG_PRINT("\n *********************************************\n");
         if(tunnel)
         {
             bInputEosReached = true;
         }
         else
         {
             bOutputEosReached = true;
         }
         event_complete();
       break;

       default:
         DEBUG_PRINT("\n Unknown Event \n");
         break;
    }
    return OMX_ErrorNone;
}


OMX_ERRORTYPE FillBufferDone(OMX_IN OMX_HANDLETYPE hComponent,
                              OMX_IN OMX_PTR pAppData,
                              OMX_IN OMX_BUFFERHEADERTYPE* pBuffer)
{
  int bytes_read=0;
  int bytes_writen = 0;
  static int count = 0;
  int tlen = 0;
  count = count + 1;
  DEBUG_PRINT(" FillBufferDone #%d size %d\n", count,pBuffer->nFilledLen);

  if((tunnel == 0) && (filewrite == 1))
  {
      bytes_writen = fwrite(pBuffer->pBuffer,1,pBuffer->nFilledLen,outputBufferFile);
      DEBUG_PRINT(" FillBufferDone size writen to file  %d\n",bytes_writen);
      totaldatalen += bytes_writen ;
  }

#ifdef PCM_PLAYBACK
   if(pcmplayback == 1)
   {
          DEBUG_PRINT(" FillBufferDone: start Writing data to pcm device for play \n");

          tlen = pBuffer->nFilledLen /2;
          if(count == 1)
          {
              if (write(m_pcmdrv_fd, (pBuffer->pBuffer), tlen ) != tlen) {
                  DEBUG_PRINT("FillBufferDone: Write data to PCM failed\n");
                  return OMX_ErrorNone;
              }
              if (write(m_pcmdrv_fd, (pBuffer->pBuffer+tlen), tlen ) != tlen) {
                  DEBUG_PRINT("FillBufferDone: Write data to PCM failed\n");
                  return OMX_ErrorNone;
              }
              DEBUG_PRINT("FillBufferDone: PCM AUDIO_START\n");
              ioctl(m_pcmdrv_fd, AUDIO_START, 0);
          }
          else
          {
              if (write(m_pcmdrv_fd, pBuffer->pBuffer, pBuffer->nFilledLen ) != pBuffer->nFilledLen) {
                  DEBUG_PRINT("FillBufferDone: Write data to PCM failed\n");
                  return OMX_ErrorNone;
              }
          }
          DEBUG_PRINT(" FillBufferDone: writing data to pcm device for play succesfull \n");
}
#endif   // PCM_PLAYBACK

    if ( pBuffer->nFlags != OMX_BUFFERFLAG_EOS )
    {
       DEBUG_PRINT(" FBD calling FTB");
       OMX_FillThisBuffer(hComponent,pBuffer);
    }
    else
    {
        DEBUG_PRINT(" FBD EOS REACHED...........\n");
        bEosOnOutputBuf = true;
    }
    return OMX_ErrorNone;
}


OMX_ERRORTYPE EmptyBufferDone(OMX_IN OMX_HANDLETYPE hComponent,
                              OMX_IN OMX_PTR pAppData,
                              OMX_IN OMX_BUFFERHEADERTYPE* pBuffer)
{
   int readBytes =0;
   DEBUG_PRINT("\nFunction %s cnt[%d]\n", __FUNCTION__, ebd_cnt);
   ebd_cnt++;
   used_ip_buf_cnt--;
   if(bEosOnInputBuf) {
        DEBUG_PRINT("\n*********************************************\n");
      DEBUG_PRINT("   EBD::EOS on input port\n ");
      DEBUG_PRINT("   TBD:::De Init the open max here....!!!\n");
      DEBUG_PRINT("*********************************************\n");

        return OMX_ErrorNone;
   }
   else if (bFlushing == true) {
      if (used_ip_buf_cnt == 0) {
        fseek(inputBufferFile, 0, 0);
        bFlushing = false;
      }
      else {
        DEBUG_PRINT("omx_mp3_adec_test: more buffer to come back\n");
        return OMX_ErrorNone;
      }
    }
    if((readBytes = Read_Buffer(pBuffer)) > 0) {
        pBuffer->nFilledLen = readBytes;
        used_ip_buf_cnt++;
        OMX_EmptyThisBuffer(hComponent,pBuffer);
    }
    else{
      pBuffer->nFlags |= OMX_BUFFERFLAG_EOS;
        bEosOnInputBuf = true;
        used_ip_buf_cnt++;
        pBuffer->nFilledLen = 0;
        OMX_EmptyThisBuffer(hComponent,pBuffer);
        DEBUG_PRINT("EBD..Either EOS or Some Error while reading file\n");
    }
    return OMX_ErrorNone;
}

void signal_handler(int sig_id)
{
  /* Flush */

   if (sig_id == SIGUSR1) {
    DEBUG_PRINT("%s Initiate flushing\n", __FUNCTION__);
    bFlushing = true;
    OMX_SendCommand(mp3_dec_handle, OMX_CommandFlush, OMX_ALL, NULL);
   }
   else if (sig_id == SIGUSR2) {
    if (bPause == true) {
      DEBUG_PRINT("%s resume playback\n", __FUNCTION__);
      bPause = false;
      OMX_SendCommand(mp3_dec_handle, OMX_CommandStateSet, OMX_StateExecuting, NULL);
      }
      else {
      DEBUG_PRINT("%s pause playback\n", __FUNCTION__);
      bPause = true;
      OMX_SendCommand(mp3_dec_handle, OMX_CommandStateSet, OMX_StatePause, NULL);
    }
  }
}

int main(int argc, char **argv)
{
    int bufCnt=0;
    OMX_ERRORTYPE result;
    struct sigaction sa;


    struct wav_header hdr;
    int bytes_writen = 0;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = &signal_handler;
    sigaction(SIGABRT, &sa, NULL);
    sigaction(SIGUSR1, &sa, NULL);
    sigaction(SIGUSR2, &sa, NULL);


    pthread_cond_init(&cond, 0);
    pthread_mutex_init(&lock, 0);


    if (argc == 6) {
      in_filename = argv[1];
      bParseHeader = atoi(argv[2]);
      pcmplayback = atoi(argv[3]);
      tunnel  = atoi(argv[4]);
      filewrite = atoi(argv[5]);
      if (tunnel == 1) {
           pcmplayback = 0; /* This feature holds good only for non tunnel mode*/
           filewrite = 0;  /* File write not supported in tunnel mode */
      }
   }
   else {

        DEBUG_PRINT(" invalid format: \n");
        DEBUG_PRINT("ex: ./mm-adec-omxmp3 MP3INPUTFILE AUTO_CONFIG PCMPLAYBACK TUNNEL FILEWRITE\n");
        DEBUG_PRINT("Note: If the SAMPFREQ or CHANNEL is zero, it will be autodetected from the header\n");
        DEBUG_PRINT( "AUTO_CONFIG = 1 (PARSE HEADER AND CONFIGURE OMX COMPONENT) \n");
        DEBUG_PRINT( "AUTO_CONFIG = 0 (DO NOT CONFIGURE OMX COMPONENT) \n");
        DEBUG_PRINT( "PCMPLAYBACK = 1 (ENABLES PCM PLAYBACK IN NON TUNNEL MODE) \n");
        DEBUG_PRINT( "PCMPLAYBACK = 0 (DISABLES PCM PLAYBACK IN NON TUNNEL MODE) \n");
        DEBUG_PRINT( "TUNNEL = 1 (DECODED MP3 SAMPLES IS PLAYED BACK)\n");
        DEBUG_PRINT( "TUNNEL = 0 (DECODED MP3 SAMPLES IS LOOPED BACK TO THE USER APP)\n");
        DEBUG_PRINT( "FILEWRITE = 1 (ENABLES PCM FILEWRITE IN NON TUNNEL MODE) \n");
        DEBUG_PRINT( "FILEWRITE = 0 (DISABLES PCM FILEWRITE IN NON TUNNEL MODE) \n");
        return 0;
    }

    if(tunnel == 0)
        aud_comp = "OMX.qcom.audio.decoder.mp3";
    else
        aud_comp = "OMX.qcom.audio.decoder.tunneled.mp3";

    DEBUG_PRINT(" OMX test app : aud_comp = %s\n",aud_comp);

   if(Init_Decoder(aud_comp)!= 0x00) {
        DEBUG_PRINT("Decoder Init failed\n");
        return -1;
    }

   if(Play_Decoder() != 0x00) {
        DEBUG_PRINT("Play_Decoder failed\n");
        return -1;
    }

   // Wait till EOS is reached...

   if(bReconfigureOutputPort)
   {
       wait_for_event();
   }
   DEBUG_PRINT(" bOutputEosReached = %d bInputEosReached = %d \n",bOutputEosReached, bInputEosReached);

  if(bOutputEosReached || (tunnel && bInputEosReached)) {

/******************************************************************/
#ifdef PCM_PLAYBACK
      if(pcmplayback == 1) {
         ioctl(m_pcmdrv_fd, AUDIO_STOP, 0);

         if(m_pcmdrv_fd >= 0)   {
             close(m_pcmdrv_fd);
             m_pcmdrv_fd = -1;
             DEBUG_PRINT(" PCM device closed succesfully \n");
         }
         else {
             DEBUG_PRINT(" PCM device close failure \n");
         }
   }
#endif // PCM_PLAYBACK

      if((tunnel == 0) && (filewrite == 1)) {
        hdr.riff_id = ID_RIFF;
        hdr.riff_sz = 0;
        hdr.riff_fmt = ID_WAVE;
        hdr.fmt_id = ID_FMT;
        hdr.fmt_sz = 16;
        hdr.audio_format = FORMAT_PCM;
        hdr.num_channels = mp3Header.channel_mode;
        hdr.sample_rate  = mp3Header.sampling_rate;
        hdr.byte_rate = hdr.sample_rate * hdr.num_channels * 2;
        hdr.block_align = hdr.num_channels * 2;
        hdr.bits_per_sample = 16;
        hdr.data_id = ID_DATA;
        hdr.data_sz = 0;

        DEBUG_PRINT("output file closed and EOS reached total decoded data length %d\n",totaldatalen);
        hdr.data_sz = totaldatalen;
        hdr.riff_sz = totaldatalen + 8 + 16 + 8;
        fseek(outputBufferFile, 0L , SEEK_SET);
        bytes_writen = fwrite(&hdr,1,sizeof(hdr),outputBufferFile);
        if (bytes_writen <= 0) {
           DEBUG_PRINT("Invalid Wav header write failed\n");
        }
        bFileclose = 1;
    fclose(outputBufferFile);
}
/************************************************************************************/

      DEBUG_PRINT("\nMoving the decoder to idle state \n");
      OMX_SendCommand(mp3_dec_handle, OMX_CommandStateSet, OMX_StateIdle,0);
      wait_for_event();


      DEBUG_PRINT("\nMoving the decoder to loaded state \n");
      OMX_SendCommand(mp3_dec_handle, OMX_CommandStateSet, OMX_StateLoaded,0);

      DEBUG_PRINT("\nFillBufferDone: Deallocating i/p buffers \n");
      for(bufCnt=0; bufCnt < input_buf_cnt; ++bufCnt) {
          OMX_FreeBuffer(mp3_dec_handle, 0, pInputBufHdrs[bufCnt]);
       }

      if(tunnel == 0) {
      DEBUG_PRINT("\nFillBufferDone: Deallocating o/p buffers \n");
      for(bufCnt=0; bufCnt < output_buf_cnt; ++bufCnt) {
          OMX_FreeBuffer(mp3_dec_handle, 1, pOutputBufHdrs[bufCnt]);
       }
}

      DEBUG_PRINT("*******************************************\n");
        wait_for_event();

      ebd_cnt=0;
      bOutputEosReached = false;
      bInputEosReached = false;
      bEosOnInputBuf = 0;
      bEosOnOutputBuf = 0;
      bReconfigureOutputPort = 0;

      result = OMX_FreeHandle(mp3_dec_handle);
      if (result != OMX_ErrorNone) {
          DEBUG_PRINT("\nOMX_FreeHandle error. Error code: %d\n", result);
      }
        mp3_dec_handle = NULL;
      /* Deinit OpenMAX */

      OMX_Deinit();

      pthread_cond_destroy(&cond);
      pthread_mutex_destroy(&lock);


      DEBUG_PRINT("*****************************************\n");
      DEBUG_PRINT("******...TEST COMPLETED...***************\n");
      DEBUG_PRINT("*****************************************\n");

    }
    return 0;
}

int Init_Decoder(OMX_STRING audio_component)
{
    DEBUG_PRINT("Inside %s \n", __FUNCTION__);
    OMX_ERRORTYPE omxresult;
    OMX_U32 total = 0;
    OMX_U8** audCompNames;
    typedef OMX_U8* OMX_U8_PTR;
    char *role ="audio_decoder";

    static OMX_CALLBACKTYPE call_back = {
        &EventHandler,&EmptyBufferDone,&FillBufferDone
    };

    int i = 0;

    /* Init. the OpenMAX Core */
    DEBUG_PRINT("\nInitializing OpenMAX Core....\n");
    omxresult = OMX_Init();

    if(OMX_ErrorNone != omxresult) {
        DEBUG_PRINT("\n Failed to Init OpenMAX core");
        return -1;
    }
    else {
        DEBUG_PRINT("\nOpenMAX Core Init Done\n");
    }

    /* Query for audio decoders*/
    DEBUG_PRINT("Mp3_test: Before entering OMX_GetComponentOfRole");
    OMX_GetComponentsOfRole(role, &total, 0);
    DEBUG_PRINT("\nTotal components of role=%s :%d", role, total);


    DEBUG_PRINT("\nComponent before GEThandle %s \n", audio_component);


    omxresult = OMX_GetHandle((OMX_HANDLETYPE*)(&mp3_dec_handle),
                        (OMX_STRING)audio_component, NULL, &call_back);
    if (FAILED(omxresult)) {
        DEBUG_PRINT("\nFailed to Load the component:%s\n", audio_component);
        return -1;
    }
    else {
        DEBUG_PRINT("\nComponent %s is in LOADED state\n", audio_component);
    }

    /* Get the port information */
    CONFIG_VERSION_SIZE(portParam);
    omxresult = OMX_GetParameter(mp3_dec_handle, OMX_IndexParamAudioInit,
                                (OMX_PTR)&portParam);

    if(FAILED(omxresult)) {
        DEBUG_PRINT("\nFailed to get Port Param\n");
        return -1;
    }
    else {
        DEBUG_PRINT("\nportParam.nPorts:%d\n", portParam.nPorts);
        DEBUG_PRINT("\nportParam.nStartPortNumber:%d\n",
                                             portParam.nStartPortNumber);
    }
    return 0;
}

int Play_Decoder()
{
    int i;
    int Size=0;
    DEBUG_PRINT("Inside %s \n", __FUNCTION__);
    OMX_ERRORTYPE ret;
    OMX_STATETYPE state;
    struct msm_audio_pcm_config drv_pcm_config;
    static int pcm_buf_size = 4800;
    static int pcm_buf_count = 2;
    unsigned bufCnt=0;
    static int first_buffer = 1;

    DEBUG_PRINT("sizeof[%d]\n", sizeof(OMX_BUFFERHEADERTYPE));

    /* open the i/p and o/p files based on the video file format passed */
    if(open_audio_file()) {
        DEBUG_PRINT("\n Returning -1");
        return -1;
    }
    /* Query the decoder input min buf requirements */
    CONFIG_VERSION_SIZE(inputportFmt);

    /* Port for which the Client needs to obtain info */
    inputportFmt.nPortIndex = portParam.nStartPortNumber;

    OMX_GetParameter(mp3_dec_handle,OMX_IndexParamPortDefinition,&inputportFmt);
    DEBUG_PRINT ("\nDec: Input Buffer Count %d\n", inputportFmt.nBufferCountMin);
    DEBUG_PRINT ("\nDec: Input Buffer Size %d\n", inputportFmt.nBufferSize);

    if(OMX_DirInput != inputportFmt.eDir) {
        DEBUG_PRINT ("\nDec: Expect Input Port\n");
        return -1;
    }

   inputportFmt.nBufferCountActual = inputportFmt.nBufferCountMin + 5;
   OMX_SetParameter(mp3_dec_handle,OMX_IndexParamPortDefinition,&inputportFmt);
   if(tunnel == 0) {
    /* Query the decoder outport's min buf requirements */
    CONFIG_VERSION_SIZE(outputportFmt);
    /* Port for which the Client needs to obtain info */
    outputportFmt.nPortIndex = portParam.nStartPortNumber + 1;

    OMX_GetParameter(mp3_dec_handle,OMX_IndexParamPortDefinition,&outputportFmt);
    DEBUG_PRINT ("\nDec: Output Buffer Count %d\n", outputportFmt.nBufferCountMin);
    DEBUG_PRINT ("\nDec: Output Buffer Size %d\n", outputportFmt.nBufferSize);

    if(OMX_DirOutput != outputportFmt.eDir) {
        DEBUG_PRINT ("\nDec: Expect Output Port\n");
        return -1;
    }
    outputportFmt.nBufferCountActual = outputportFmt.nBufferCountMin;
    OMX_SetParameter(mp3_dec_handle,OMX_IndexParamPortDefinition,&outputportFmt);
}

  CONFIG_VERSION_SIZE(mp3param);
if(pcmplayback == 1)
{
          DEBUG_PRINT(" open pcm device \n");
          m_pcmdrv_fd = open("/dev/msm_pcm_out", O_RDWR);
          if ( m_pcmdrv_fd < 0 )
          {
            DEBUG_PRINT("Cannot open audio device\n");
          }
          else
          {
            DEBUG_PRINT("Open pcm device successfull\n");
            DEBUG_PRINT("Configure Driver for PCM playback \n");
            ioctl(m_pcmdrv_fd, AUDIO_GET_CONFIG, &drv_pcm_config);
            DEBUG_PRINT("drv_pcm_config.buffer_count %d \n", drv_pcm_config.buffer_count);
            DEBUG_PRINT("drv_pcm_config.buffer_size %d \n",  drv_pcm_config.buffer_size);
            drv_pcm_config.sample_rate   = mp3Header.sampling_rate;
            drv_pcm_config.channel_count = mp3Header.channel_mode;
            ioctl(m_pcmdrv_fd, AUDIO_SET_CONFIG, &drv_pcm_config);
            DEBUG_PRINT("Configure Driver for PCM playback \n");
            ioctl(m_pcmdrv_fd, AUDIO_GET_CONFIG, &drv_pcm_config);
            DEBUG_PRINT("drv_pcm_config.buffer_count %d \n", drv_pcm_config.buffer_count);
            DEBUG_PRINT("drv_pcm_config.buffer_size %d \n",  drv_pcm_config.buffer_size);
            pcm_buf_size = drv_pcm_config.buffer_size;
            pcm_buf_count = drv_pcm_config.buffer_count;
          }
}

    DEBUG_PRINT ("\nOMX_SendCommand Decoder -> IDLE\n");
    OMX_SendCommand(mp3_dec_handle, OMX_CommandStateSet, OMX_StateIdle,0);
    /* wait_for_event(); should not wait here event complete status will
       not come until enough buffer are allocated */

   input_buf_cnt = inputportFmt.nBufferCountActual; //inputportFmt.nBufferCountMin + 5;
    DEBUG_PRINT("Transition to Idle State succesful...\n");
    /* Allocate buffer on decoder's i/p port */
    error = Allocate_Buffer(mp3_dec_handle, &pInputBufHdrs, inputportFmt.nPortIndex,
                            input_buf_cnt, inputportFmt.nBufferSize);
    if (error != OMX_ErrorNone) {
        DEBUG_PRINT ("\nOMX_AllocateBuffer Input buffer error\n");
        return -1;
    }
    else {
        DEBUG_PRINT ("\nOMX_AllocateBuffer Input buffer success\n");
    }

   if(tunnel == 0) {
      output_buf_cnt = outputportFmt.nBufferCountActual ;

    /* Allocate buffer on decoder's O/Pp port */
    error = Allocate_Buffer(mp3_dec_handle, &pOutputBufHdrs, outputportFmt.nPortIndex,
                            output_buf_cnt, outputportFmt.nBufferSize);
    if (error != OMX_ErrorNone) {
        DEBUG_PRINT ("\nOMX_AllocateBuffer Output buffer error\n");
        return -1;
    }
    else {
        DEBUG_PRINT ("\nOMX_AllocateBuffer Output buffer success\n");
    }
}

    wait_for_event();

   if (tunnel == 1) {
    DEBUG_PRINT ("\nOMX_SendCommand to enable TUNNEL MODE during IDLE\n");
    OMX_SendCommand(mp3_dec_handle, OMX_CommandPortDisable,1,0);
    wait_for_event();
   }

    DEBUG_PRINT ("\nOMX_SendCommand Decoder -> Executing\n");
    OMX_SendCommand(mp3_dec_handle, OMX_CommandStateSet, OMX_StateExecuting,0);
    wait_for_event();


   if((tunnel == 0))
   {
    DEBUG_PRINT(" Start sending OMX_FILLthisbuffer\n");

    for(i=0; i < output_buf_cnt; i++) {
        DEBUG_PRINT ("\nOMX_FillThisBuffer on output buf no.%d\n",i);
        pOutputBufHdrs[i]->nOutputPortIndex = 1;
        pOutputBufHdrs[i]->nFlags &= ~OMX_BUFFERFLAG_EOS;
        ret = OMX_FillThisBuffer(mp3_dec_handle, pOutputBufHdrs[i]);
        if (OMX_ErrorNone != ret) {
            DEBUG_PRINT("OMX_FillThisBuffer failed with result %d\n", ret);
        }
        else {
            DEBUG_PRINT("OMX_FillThisBuffer success!\n");
     }
    }
}

  DEBUG_PRINT(" Start sending OMX_emptythisbuffer\n");

   for (i = 0;i < input_buf_cnt;i++) {
    DEBUG_PRINT ("\nOMX_EmptyThisBuffer on Input buf no.%d\n",i);
    pInputBufHdrs[i]->nInputPortIndex = 0;
    Size = Read_Buffer(pInputBufHdrs[i]);
    if(Size <=0 ){
       DEBUG_PRINT("NO DATA READ\n");
    }
    pInputBufHdrs[i]->nFilledLen = Size;
    pInputBufHdrs[i]->nInputPortIndex = 0;
    used_ip_buf_cnt++;
    if(first_buffer)
    {
      first_buffer = 0;
      ret = parse_mp3_frameheader(pInputBufHdrs[i],&mp3Header);
      if(ret != OMX_ErrorNone)
      {
        DEBUG_PRINT("parse_mp3_frameheader return failure\n");
        mp3Header.sampling_rate = DEFAULT_SAMPLING_RATE;
        mp3Header.channel_mode  = DEFAULT_CHANNEL_MODE;
      }
      mp3param.nPortIndex   = 0;
      mp3param.nSampleRate  = mp3Header.sampling_rate;
      mp3param.nChannels    = mp3Header.channel_mode;
      mp3param.nBitRate     = 0;
      mp3param.eChannelMode = OMX_AUDIO_ChannelModeStereo;
      mp3param.eFormat      = OMX_AUDIO_MP3StreamFormatMP1Layer3;
      if(!bParseHeader)
      {
         mp3param.nSampleRate = DEFAULT_SAMPLING_RATE;
         mp3param.nChannels   = DEFAULT_CHANNEL_MODE;
      }
      OMX_SetParameter(mp3_dec_handle, OMX_IndexParamAudioMp3,
                          (OMX_PTR)&mp3param);
    }
    ret = OMX_EmptyThisBuffer(mp3_dec_handle, pInputBufHdrs[i]);
    if (OMX_ErrorNone != ret) {
            DEBUG_PRINT("OMX_EmptyThisBuffer failed with result %d\n", ret);
    }
    else {
            DEBUG_PRINT("OMX_EmptyThisBuffer success!\n");
    }
   }
    /* Waiting for EOS or PortSettingsChange*/
    wait_for_event();
    if(bReconfigureOutputPort)
    {
       DEBUG_PRINT("************************************");
       DEBUG_PRINT("RECIEVED EVENT PORT SETTINGS CHANGED EVENT\n");
       DEBUG_PRINT("******************************************\n");
           DEBUG_PRINT("*PORT SETTINGS CHANGED: FLUSHCOMMAND TO COMPONENT*******\n");
       flushinprogress = 1;
       OMX_SendCommand(mp3_dec_handle, OMX_CommandFlush, 1, NULL);
       wait_for_event();  // output port
       // Send DISABLE command
       OMX_SendCommand(mp3_dec_handle, OMX_CommandPortDisable, 1, 0);
       DEBUG_PRINT("******************************************\n");
       DEBUG_PRINT("FREEING BUFFERS output_buf_cnt=%d\n",output_buf_cnt);
       DEBUG_PRINT("******************************************\n");
       for(bufCnt=0; bufCnt < output_buf_cnt; ++bufCnt) {
        OMX_FreeBuffer(mp3_dec_handle, 1, pOutputBufHdrs[bufCnt]);
       }

       wait_for_event();
       DEBUG_PRINT("******************************************\n");
       DEBUG_PRINT("DISABLE EVENT RECD\n");
       DEBUG_PRINT("******************************************\n");
       OMX_SendCommand(mp3_dec_handle, OMX_CommandPortEnable, 1, 0);
       flushinprogress = 0;
       DEBUG_PRINT("******************************************\n");
       DEBUG_PRINT("ALLOC BUFFER AFTER PORT REENABLE");
       DEBUG_PRINT("******************************************\n");
       error = Allocate_Buffer(mp3_dec_handle, &pOutputBufHdrs, outputportFmt.nPortIndex,
                  output_buf_cnt, outputportFmt.nBufferSize);
       if (error != OMX_ErrorNone) {
        DEBUG_PRINT ("\nOMX_AllocateBuffer Output buffer error output_buf_cnt=%d\n",output_buf_cnt);
        return -1;
       }
       else {
        DEBUG_PRINT ("\nOMX_AllocateBuffer Output buffer success output_buf_cnt=%d\n",output_buf_cnt);
       }
       DEBUG_PRINT("******************************************\n");
       DEBUG_PRINT("ENABLE EVENTiHANDLER RECD\n");
       DEBUG_PRINT("******************************************\n");
       wait_for_event();
       DEBUG_PRINT("******************************************\n");
       DEBUG_PRINT("FTB after PORT RENABLE\n");
       DEBUG_PRINT("******************************************\n");
       for(i=0; i < output_buf_cnt; i++) {
        DEBUG_PRINT ("\nOMX_FillThisBuffer on output buf no.%d\n",i);
        pOutputBufHdrs[i]->nOutputPortIndex = 1;
        pOutputBufHdrs[i]->nFlags &= ~OMX_BUFFERFLAG_EOS;
        ret = OMX_FillThisBuffer(mp3_dec_handle, pOutputBufHdrs[i]);
        if (OMX_ErrorNone != ret) {
         DEBUG_PRINT("OMX_FillThisBuffer failed with result %d\n", ret);
        }
        else {
         DEBUG_PRINT("OMX_FillThisBuffer success!\n");
        }
       }
    }
    return 0;
}

OMX_ERRORTYPE  parse_mp3_frameheader(OMX_BUFFERHEADERTYPE* buffer,
                                     struct mp3_header *header)
{
    OMX_U8* temp_pBuf1 = NULL,*pBuf1;
    unsigned int i = 0;
    OMX_U8 temp;

    if ( buffer->nFilledLen == 0 )
    {
        DEBUG_PRINT ("\n Length is zero hence no point in processing \n");
        return OMX_ErrorNone;
    }
    temp_pBuf1 = buffer->pBuffer;
    i = 0;
    while ( *temp_pBuf1 != 0xFF )
    {
        i++;
        temp_pBuf1++;
        if ( i==buffer->nFilledLen )
            return OMX_ErrorMax;
    }
    temp = temp_pBuf1[0];
    header->sync = temp & 0xFF;
    if ( header->sync == 0xFF )
    {
        temp = temp_pBuf1[1];
        header->sync = temp & 0xC0;
        if ( header->sync != 0xC0 )
        {
            DEBUG_PRINT("parse_mp3_frameheader failure");
            return OMX_ErrorMax;
        }
    }
    else
    {
        DEBUG_PRINT("parse_mp3_frameheader failure");
        return OMX_ErrorMax;
    }
    temp = temp_pBuf1[1];
    header->version = (temp & 0x18)>>3;
    header->Layer = (temp & 0x06)>>1;
    temp = temp_pBuf1[2];
    header->sampling_rate = (temp & 0x0C)>>2;
    temp = temp_pBuf1[3];
    header->channel_mode = (temp & 0xC0)>>6;
    DEBUG_PRINT("Channel Mode: %d, Sampling rate: %d and header version: %d from the header\n",
                header->channel_mode, header->sampling_rate, header->version);
    if ( (header->channel_mode == 0)||(header->channel_mode == 1)||(header->channel_mode == 2) )
    {
        header->channel_mode = 2;  // stereo
    }
    else if ( header->channel_mode == 3 )
    {
        header->channel_mode = 1; // for all other cases configuring as mono TBD
    }
    else
    {
        header->channel_mode = 2; // if the channel is not recog. making the channel by default to Stereo.
        DEBUG_PRINT("Defauting the channel mode to Stereo");
    }
    header->sampling_rate = mp3_frequency_index[header->sampling_rate][header->version];
    DEBUG_PRINT(" frequency = %d, channels = %d\n",header->sampling_rate,header->channel_mode);
    return OMX_ErrorNone;
}

static OMX_ERRORTYPE Allocate_Buffer ( OMX_COMPONENTTYPE *avc_dec_handle,
                                       OMX_BUFFERHEADERTYPE  ***pBufHdrs,
                                       OMX_U32 nPortIndex,
                                       long bufCntMin, long bufSize)
{
    DEBUG_PRINT("Inside %s \n", __FUNCTION__);
    OMX_ERRORTYPE error=OMX_ErrorNone;
    long bufCnt=0;

    *pBufHdrs= (OMX_BUFFERHEADERTYPE **)
                   malloc(sizeof(OMX_BUFFERHEADERTYPE*)*bufCntMin);

    for(bufCnt=0; bufCnt < bufCntMin; ++bufCnt) {
        DEBUG_PRINT("\n OMX_AllocateBuffer No %d \n", bufCnt);
        error = OMX_AllocateBuffer(mp3_dec_handle, &((*pBufHdrs)[bufCnt]),
                                   nPortIndex, NULL, bufSize);
    }

    return error;
}




static int Read_Buffer (OMX_BUFFERHEADERTYPE  *pBufHdr )
{

   int bytes_read=0;


   pBufHdr->nFilledLen = 0;
   pBufHdr->nFlags |= OMX_BUFFERFLAG_EOS;

   bytes_read = fread(pBufHdr->pBuffer, 1, pBufHdr->nAllocLen , inputBufferFile);
   pBufHdr->nFilledLen = bytes_read;
   if(bytes_read == 0) {

     pBufHdr->nFlags |= OMX_BUFFERFLAG_EOS;
     DEBUG_PRINT ("\nBytes read zero\n");
   }
   else {
        pBufHdr->nFlags &= ~OMX_BUFFERFLAG_EOS;
        DEBUG_PRINT ("\nBytes read is Non zero\n");
   }

    return bytes_read;;
}

static int open_audio_file ()
{
    int error_code = 0;
    const char *outfilename = "Audio_mp3.pcm";
    struct wav_header hdr;
    int header_len = 0;
    memset(&hdr,0,sizeof(hdr));

    hdr.riff_id = ID_RIFF;
    hdr.riff_sz = 0;
    hdr.riff_fmt = ID_WAVE;
    hdr.fmt_id = ID_FMT;
    hdr.fmt_sz = 16;
    hdr.audio_format = FORMAT_PCM;
    hdr.num_channels = 2; // Will be updated in the end
    hdr.sample_rate = 44100; // Will be updated in the end
    hdr.byte_rate = hdr.sample_rate * hdr.num_channels * 2;
    hdr.block_align = hdr.num_channels * 2;
    hdr.bits_per_sample = 16;
    hdr.data_id = ID_DATA;
    hdr.data_sz = 0;



    DEBUG_PRINT("Inside %s filename=%s\n", __FUNCTION__, in_filename);
    inputBufferFile = fopen (in_filename, "rb");
    if (inputBufferFile < 0) {
        DEBUG_PRINT("\ni/p file %s could NOT be opened\n",
                                     in_filename);
    error_code = -1;
  }

  if((tunnel == 0) && (filewrite == 1)) {
  DEBUG_PRINT("output file is opened\n");
  outputBufferFile = fopen("Audio_mp3.wav","wb");
  if (outputBufferFile < 0) {
        DEBUG_PRINT("\no/p file %s could NOT be opened\n",
                                     outfilename);
    error_code = -1;
  }

  header_len = fwrite(&hdr,1,sizeof(hdr),outputBufferFile);


  if (header_len <= 0) {
        DEBUG_PRINT("Invalid Wav header \n");
  }
  DEBUG_PRINT(" Length og wav header is %d \n",header_len );
}
     return error_code;
}




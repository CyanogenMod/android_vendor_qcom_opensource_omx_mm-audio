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
#include <stdint.h>
#include<string.h>
#include <pthread.h>
#include <linux/ioctl.h>

typedef unsigned char uint8;
typedef unsigned char byte;
typedef unsigned int  uint32;
typedef unsigned int  uint16;

void Release_Encoder();

/************************************************************************/
/*              #DEFINES                            */
/************************************************************************/
#define false 0
#define true 1

#define MIN(A,B)    (((A) < (B))?(A):(B))

#define SAMPLE_RATE 48000
#define STEREO      2

#define CONFIG_VERSION_SIZE(param) \
    param.nVersion.nVersion = CURRENT_OMX_SPEC_VERSION;\
    param.nSize = sizeof(param);

#define FAILED(result) (result != OMX_ErrorNone)

#define SUCCEEDED(result) (result == OMX_ErrorNone)
#define DEBUG_PRINT printf

/************************************************************************/
/*             GLOBAL DECLARATIONS                     */
/************************************************************************/

pthread_mutex_t lock;
pthread_cond_t cond;
pthread_mutex_t elock;
pthread_cond_t econd;
pthread_cond_t fcond;
FILE * inputBufferFile;
FILE * outputBufferFile;
OMX_PARAM_PORTDEFINITIONTYPE inputportFmt;
OMX_PARAM_PORTDEFINITIONTYPE outputportFmt;
OMX_AUDIO_PARAM_AACPROFILETYPE aacparam;
OMX_PORT_PARAM_TYPE portParam;
OMX_PORT_PARAM_TYPE portFmt;
OMX_ERRORTYPE error;

static bFileclose = 0;
static unsigned totaldatalen = 0;

/************************************************************************/
/*             GLOBAL INIT             */
/************************************************************************/

int output_buf_cnt = 0;
int bInputEosReached = 0;
int bFlushing = false;
int bPause    = false;
volatile int event_is_done = 0;
const char *out_filename;
uint32_t samplerate = 16000;
uint32_t channels = 2;
uint32_t bitrate = 32000;
uint32_t pcmplayback = 0;
uint32_t tunnel      = 0;
unsigned to_idle_transition = 0;

//* OMX Spec Version supported by the wrappers. Version = 1.1 */
const OMX_U32 CURRENT_OMX_SPEC_VERSION = 0x00000101;
OMX_COMPONENTTYPE* aac_enc_handle = 0;

OMX_BUFFERHEADERTYPE  **pOutputBufHdrs = NULL;

/************************************************************************/
/*           GLOBAL FUNC DECL                        */
/************************************************************************/
int Init_Encoder();
int Rec_Encoder();

/**************************************************************************/
/*           STATIC DECLARATIONS                       */
/**************************************************************************/

static int open_audio_file ();

static OMX_ERRORTYPE Allocate_Buffer ( OMX_COMPONENTTYPE *aac_enc_handle,
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
    DEBUG_PRINT("Function %s \n", __FUNCTION__);

    switch(eEvent) {
        case OMX_EventCmdComplete:
            DEBUG_PRINT("\n OMX_EventCmdComplete event=%d data1=%d data2=%d\n",(OMX_EVENTTYPE)eEvent,
                nData1,nData2);
            if(to_idle_transition)
                bInputEosReached = 1;
            event_complete();
            break;
        case OMX_EventError:
            DEBUG_PRINT("\n OMX_EventError \n");
            break;

        case OMX_EventPortSettingsChanged:
            DEBUG_PRINT("\n OMX_EventPortSettingsChanged \n");
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
    static unsigned int  count = 0;
    static unsigned int tcount = 0;
    static unsigned int taudcount = 0;
    static unsigned int tlen = 0;
    pthread_t thread;
    int r = 0;
    unsigned char readBuf;
    static int releaseCount = 0;

    if(bInputEosReached || (pBuffer->nFilledLen == 0)) {
        DEBUG_PRINT("\n*********************************************\n");
        DEBUG_PRINT("   EBD::EOS on output port\n ");
        DEBUG_PRINT("   TBD:::De Init the open max here....!!!\n");
        DEBUG_PRINT("*********************************************\n");
        return OMX_ErrorNone;
    }

    DEBUG_PRINT(" FillBufferDone #%d size %d\n", ++count,pBuffer->nFilledLen);

    bytes_writen = fwrite(pBuffer->pBuffer,1,pBuffer->nFilledLen,outputBufferFile);
    if(bytes_writen < pBuffer->nFilledLen)
    {
        DEBUG_PRINT("error: invalid AAC encoded data \n");
        return OMX_ErrorNone;
    }

    DEBUG_PRINT(" FillBufferDone size writen to file  %d\n",bytes_writen);
    totaldatalen += bytes_writen ;

    if(!releaseCount)
    {
        if(read(0, &readBuf, 1) == 1)
        {
            if ((readBuf == 13) || (readBuf == 10))
            {
                printf("\n GOT THE ENTER KEY\n");
                releaseCount++;
            }
        }
    }
    if(releaseCount == 1)
    {
        // Dont issue any more FTB
        // Trigger Exe-->Idle Transition
        sleep(1);
        OMX_SendCommand(aac_enc_handle, OMX_CommandStateSet, OMX_StateIdle,0);
        sleep(1);
        to_idle_transition = 1;
        releaseCount++;
        // wait till Idle transition is complete
        // Trigger ReleaseEncoder procedure

    }
    else if(!releaseCount)
    {
        DEBUG_PRINT(" FBD calling FTB");
        OMX_FillThisBuffer(hComponent,pBuffer);
    }
    else{}

    return OMX_ErrorNone;
}

OMX_ERRORTYPE EmptyBufferDone(OMX_IN OMX_HANDLETYPE hComponent,
                              OMX_IN OMX_PTR pAppData,
                              OMX_IN OMX_BUFFERHEADERTYPE* pBuffer)
{
    DEBUG_PRINT("\nFunction %s \n", __FUNCTION__);
    return OMX_ErrorNone;
}

void signal_handler(int sig_id) {

    /* Flush */
    if (sig_id == SIGUSR1) {
        DEBUG_PRINT("%s Initiate flushing\n", __FUNCTION__);
        bFlushing = true;
        OMX_SendCommand(aac_enc_handle, OMX_CommandFlush, OMX_ALL, NULL);
    } else if (sig_id == SIGUSR2) {
        if (bPause == true) {
            DEBUG_PRINT("%s resume recording\n", __FUNCTION__);
            bPause = false;
            OMX_SendCommand(aac_enc_handle, OMX_CommandStateSet, OMX_StateExecuting, NULL);
        } else {
            DEBUG_PRINT("%s pause recording\n", __FUNCTION__);
            bPause = true;
            OMX_SendCommand(aac_enc_handle, OMX_CommandStateSet, OMX_StatePause, NULL);
        }
    }
}

int main(int argc, char **argv)
{
    int bufCnt=0;
    OMX_ERRORTYPE result;

    struct sigaction sa;
    unsigned char tmp;

    int bytes_writen = 0;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = &signal_handler;
    sigaction(SIGABRT, &sa, NULL);
    sigaction(SIGUSR1, &sa, NULL);
    sigaction(SIGUSR2, &sa, NULL);

    (void) signal(SIGINT, Release_Encoder);

    pthread_cond_init(&cond, 0);
    pthread_mutex_init(&lock, 0);

    if (argc >= 7) {
        out_filename = argv[1];
        samplerate = atoi(argv[2]);
        channels = atoi(argv[3]);
        pcmplayback = atoi(argv[4]);
        tunnel  = atoi(argv[5]);
        bitrate = atoi(argv[6]);
        if (tunnel == 1) {
            pcmplayback = 0; /* This feature holds good only for non tunnel mode*/
        }
        else
        {
            DEBUG_PRINT(" NON-TUNNEL MODE TBD \n");
            return 0;
        }
    } else {
        DEBUG_PRINT(" invalid format: \n");
        DEBUG_PRINT("ex: ./mm-aenc-omxaac AAC_OUTPUTFILE SAMPFREQ CHANNEL PCMPLAYBACK TUNNEL BITRATE\n");
        DEBUG_PRINT( "TUNNEL = 1 (ENCODED AAC SAMPLES WRITEN TO AAC_OUTPUTFILE)\n");
        DEBUG_PRINT( "NOTE: NON-TUNNEL & PCMPLAYBACK MODE NOT SUPPORTED: TBD : \n");
        DEBUG_PRINT( "BITRATE in bits/sec \n");
        return 0;
    }

    if(Init_Encoder()!= 0x00)
    {
        DEBUG_PRINT("Decoder Init failed\n");
        return -1;
    }

    fcntl(0, F_SETFL, O_NONBLOCK);

    if(Rec_Encoder() != 0x00)
    {
        DEBUG_PRINT("Rec_Encoder failed\n");
        return -1;
    }

    // Wait till EOS is reached...
    wait_for_event();

    if(bInputEosReached)
    {
        DEBUG_PRINT("\nMoving the encoder to loaded state \n");
        OMX_SendCommand(aac_enc_handle, OMX_CommandStateSet, OMX_StateLoaded,0);

        DEBUG_PRINT ("\nDeallocating o/p buffers \n");
        for(bufCnt=0; bufCnt < output_buf_cnt; ++bufCnt) {
            OMX_FreeBuffer(aac_enc_handle, 1, pOutputBufHdrs[bufCnt]);
        }
        wait_for_event();

        fclose(outputBufferFile);

        result = OMX_FreeHandle(aac_enc_handle);
        if (result != OMX_ErrorNone) {
            DEBUG_PRINT ("\nOMX_FreeHandle error. Error code: %d\n", result);
        }

        /* Deinit OpenMAX */

        OMX_Deinit();
        bInputEosReached = false;
        aac_enc_handle = NULL;
        pthread_cond_destroy(&cond);
        pthread_mutex_destroy(&lock);
        DEBUG_PRINT("*****************************************\n");
        DEBUG_PRINT("******...AAC ENC TEST COMPLETED...***************\n");
        DEBUG_PRINT("*****************************************\n");
    }
    return 0;
}

void Release_Encoder()
{
    static cnt=0;
    int bufCnt=0;
    OMX_ERRORTYPE result;

    DEBUG_PRINT("END OF AAC ENCODING: EXITING PLEASE WAIT\n");
    bInputEosReached = 1;
    event_complete();
    cnt++;
    if(cnt > 1)
    {
        /* FORCE RESET  */
        aac_enc_handle = NULL;
        bInputEosReached = false;

        result = OMX_FreeHandle(aac_enc_handle);
        if (result != OMX_ErrorNone) {
            DEBUG_PRINT ("\nOMX_FreeHandle error. Error code: %d\n", result);
        }

        /* Deinit OpenMAX */

        OMX_Deinit();

        pthread_cond_destroy(&cond);
        pthread_mutex_destroy(&lock);
        DEBUG_PRINT("*****************************************\n");
        DEBUG_PRINT("******...AAC ENC TEST COMPLETED...***************\n");
        DEBUG_PRINT("*****************************************\n");
        exit(0);
    }
}

int Init_Encoder()
{
    DEBUG_PRINT("Inside %s \n", __FUNCTION__);
    OMX_ERRORTYPE omxresult;
    OMX_U32 total = 0;
    OMX_U8** audCompNames;
    typedef OMX_U8* OMX_U8_PTR;
    char *role ="audio_encoder";

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

    /* Query for audio encoders*/
    DEBUG_PRINT("Aac_test: Before entering OMX_GetComponentOfRole");
    OMX_GetComponentsOfRole(role, &total, 0);
    DEBUG_PRINT ("\nTotal components of role=%s :%d", role, total);

    omxresult = OMX_GetHandle((OMX_HANDLETYPE*)(&aac_enc_handle),
        "OMX.qcom.audio.encoder.tunneled.aac", NULL, &call_back);

    if (FAILED(omxresult)) {
        DEBUG_PRINT ("\nFailed to Load the component:\n");
        return -1;
    }
    else
    {
        DEBUG_PRINT ("\nComponent is in LOADED state\n");
    }

    /* Get the port information */
    CONFIG_VERSION_SIZE(portParam);
    omxresult = OMX_GetParameter(aac_enc_handle, OMX_IndexParamAudioInit,
        (OMX_PTR)&portParam);

    if(FAILED(omxresult)) {
        DEBUG_PRINT("\nFailed to get Port Param\n");
        return -1;
    }
    else
    {
        DEBUG_PRINT ("\nportParam.nPorts:%d\n", portParam.nPorts);
        DEBUG_PRINT ("\nportParam.nStartPortNumber:%d\n",
            portParam.nStartPortNumber);
    }

    DEBUG_PRINT("Set parameter immediately followed by getparameter");
    omxresult = OMX_SetParameter(aac_enc_handle,
        OMX_IndexParamPortDefinition,
        &portFmt);

    if(OMX_ErrorNone != omxresult)
    {
        DEBUG_PRINT("Set parameter failed");
    }

    return 0;
}

int Rec_Encoder()
{
    int i;
    int Size=0;
    DEBUG_PRINT("Inside %s \n", __FUNCTION__);
    OMX_ERRORTYPE ret;
    OMX_STATETYPE state;

    DEBUG_PRINT("sizeof[%d]\n", sizeof(OMX_BUFFERHEADERTYPE));

    /* open the i/p and o/p files based on the video file format passed */
    if(open_audio_file()) {
        DEBUG_PRINT("\n Returning -1");
        return -1;
    }

    /* Query the encoder input min buf requirements */
    CONFIG_VERSION_SIZE(inputportFmt);

    /* Port for which the Client needs to obtain info */
    inputportFmt.nPortIndex = portParam.nStartPortNumber;

    OMX_GetParameter(aac_enc_handle,OMX_IndexParamPortDefinition,&inputportFmt);
    DEBUG_PRINT ("\nEnc Input Buffer Count %d\n", inputportFmt.nBufferCountMin);
    DEBUG_PRINT ("\nEnc: Input Buffer Size %d\n", inputportFmt.nBufferSize);

    if(OMX_DirInput != inputportFmt.eDir) {
        DEBUG_PRINT ("\nEnc: Expect Input Port\n");
        return -1;
    }

    /* Query the encoder outport's min buf requirements */
    CONFIG_VERSION_SIZE(outputportFmt);
    /* Port for which the Client needs to obtain info */
    outputportFmt.nPortIndex = portParam.nStartPortNumber + 1;

    OMX_GetParameter(aac_enc_handle,OMX_IndexParamPortDefinition,&outputportFmt);
    DEBUG_PRINT ("\nEnc: Output Buffer Count %d\n", outputportFmt.nBufferCountMin);
    DEBUG_PRINT ("\nEnc: Output Buffer Size %d\n", outputportFmt.nBufferSize);

    if(OMX_DirOutput != outputportFmt.eDir) {
        DEBUG_PRINT ("\nEnc: Expect Output Port\n");
        return -1;
    }

    CONFIG_VERSION_SIZE(aacparam);

    aacparam.nPortIndex   =  0;
    aacparam.nChannels    =  channels; //2 ; /* 1-> mono 2-> stereo*/
    aacparam.nBitRate     =  bitrate;
    aacparam.nSampleRate  =  samplerate;
    aacparam.eChannelMode =  OMX_AUDIO_ChannelModeStereo;
    aacparam.eAACStreamFormat    =  OMX_AUDIO_AACStreamFormatMP2ADTS;
    OMX_SetParameter(aac_enc_handle,OMX_IndexParamAudioAac,&aacparam);

    DEBUG_PRINT ("\nOMX_SendCommand Encoder -> IDLE\n");
    OMX_SendCommand(aac_enc_handle, OMX_CommandStateSet, OMX_StateIdle,0);
    /* wait_for_event(); should not wait here event complete status will
    not come until enough buffer are allocated */
    output_buf_cnt = outputportFmt.nBufferCountMin ;

    /* Allocate buffer on encoder's O/Pp port */
    error = Allocate_Buffer(aac_enc_handle, &pOutputBufHdrs, outputportFmt.nPortIndex,
        output_buf_cnt, outputportFmt.nBufferSize);
    if (error != OMX_ErrorNone) {
        DEBUG_PRINT ("\nOMX_AllocateBuffer Output buffer error\n");
        return -1;
    }
    else {
        DEBUG_PRINT ("\nOMX_AllocateBuffer Output buffer success\n");
    }

    wait_for_event();


    if (tunnel == 1)
    {
        DEBUG_PRINT ("\nOMX_SendCommand to enable TUNNEL MODE during IDLE\n");
        OMX_SendCommand(aac_enc_handle, OMX_CommandPortDisable,0,0); // disable input port
        wait_for_event();
    }

    DEBUG_PRINT ("\nOMX_SendCommand encoder -> Executing\n");
    OMX_SendCommand(aac_enc_handle, OMX_CommandStateSet, OMX_StateExecuting,0);
    wait_for_event();

    DEBUG_PRINT(" Start sending OMX_FILLthisbuffer\n");

    for(i=0; i < output_buf_cnt; i++) {
        DEBUG_PRINT ("\nOMX_FillThisBuffer on output buf no.%d\n",i);
        pOutputBufHdrs[i]->nOutputPortIndex = 1;
        pOutputBufHdrs[i]->nFlags &= ~OMX_BUFFERFLAG_EOS;
        ret = OMX_FillThisBuffer(aac_enc_handle, pOutputBufHdrs[i]);
        if (OMX_ErrorNone != ret) {
            DEBUG_PRINT("OMX_FillThisBuffer failed with result %d\n", ret);
        }
        else {
            DEBUG_PRINT("OMX_FillThisBuffer success!\n");
        }
    }

    return 0;
}


static OMX_ERRORTYPE Allocate_Buffer ( OMX_COMPONENTTYPE *avc_enc_handle,
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
        error = OMX_AllocateBuffer(aac_enc_handle, &((*pBufHdrs)[bufCnt]),
            nPortIndex, NULL, bufSize);
    }

    return error;
}

//In Encoder this Should Open a PCM or WAV file for input.

static int open_audio_file ()
{
    int error_code = 0;
    DEBUG_PRINT("Inside %s filename=%s\n", __FUNCTION__, out_filename);
    outputBufferFile = fopen (out_filename, "wb");
    if (outputBufferFile < 0) {
        DEBUG_PRINT("\ni/p file %s could NOT be opened\n",
            out_filename);
        error_code = -1;
    }

    return error_code;
}


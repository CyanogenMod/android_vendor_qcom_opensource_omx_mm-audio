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
/*============================================================================
  @file omx_adec_mp3.c
  This module contains the implementation of the OpenMAX MP3 core & component.

*//*========================================================================*/


//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////

#include<string.h>
#include <fcntl.h>
#include <omx_mp3_adec.h>
#include <sys/ioctl.h>


#define max(x,y) (x >= y?x:y)

#define AUDIO_IOCTL_MAGIC 'a'

#define AUDIO_START        _IOW(AUDIO_IOCTL_MAGIC, 0, unsigned)
#define AUDIO_STOP         _IOW(AUDIO_IOCTL_MAGIC, 1, unsigned)
#define AUDIO_FLUSH        _IOW(AUDIO_IOCTL_MAGIC, 2, unsigned)
#define AUDIO_GET_CONFIG   _IOR(AUDIO_IOCTL_MAGIC, 3, unsigned)
#define AUDIO_SET_CONFIG   _IOW(AUDIO_IOCTL_MAGIC, 4, unsigned)
#define AUDIO_GET_STATS    _IOR(AUDIO_IOCTL_MAGIC, 5, unsigned)
#define AUDIO_GET_PCM_CONFIG _IOR(AUDIO_IOCTL_MAGIC, 30, unsigned)
#define AUDIO_SET_PCM_CONFIG _IOW(AUDIO_IOCTL_MAGIC, 31, unsigned)

#define OMX_ADEC_VOLUME_STEP 0x147 /* 0x7FFF / 0x64 */
#define OMX_ADEC_MIN         0
#define OMX_ADEC_MAX         100


#define NON_TUNNEL  1
#define TUNNEL      0

struct msm_audio_config {
    uint32_t buffer_size;
    uint32_t buffer_count;
    uint32_t channel_count;
    uint32_t sample_rate;
    uint32_t type;
    uint32_t unused[3];
};

struct msm_audio_stats {
    uint32_t byte_count;
    uint32_t sample_count;
    uint32_t unused[2];
};

struct msm_audio_pcm_config {
    uint32_t pcm_feedback;    /* 0 - disable > 0 - enable */
    uint32_t buffer_count;    /* Number of buffers to allocate */
    uint32_t buffer_size;    /* Size of buffer for capturing of
                   PCM samples */
};
/*************************************************/

using namespace std;


// omx_cmd_queue destructor
omx_mp3_adec::omx_cmd_queue::~omx_cmd_queue()
{
  // Nothing to do
}

// omx cmd queue constructor
omx_mp3_adec::omx_cmd_queue::omx_cmd_queue(): m_read(0),m_write(0),m_size(0)
{
  memset(m_q,      0,sizeof(omx_event)*OMX_CORE_CONTROL_CMDQ_SIZE);
}

// omx cmd queue insert
bool omx_mp3_adec::omx_cmd_queue::insert_entry(unsigned p1, unsigned p2, unsigned id)
{
  bool ret = true;
  if(m_size < OMX_CORE_CONTROL_CMDQ_SIZE)
  {
    m_q[m_write].id       = id;
    m_q[m_write].param1   = p1;
    m_q[m_write].param2   = p2;
    m_write++;
    m_size ++;
    if(m_write >= OMX_CORE_CONTROL_CMDQ_SIZE)
    {
      m_write = 0;
    }
  }
  else
  {
    ret = false;
    DEBUG_PRINT_ERROR("ERROR!!! Command Queue Full");
  }
  return ret;
}

// omx cmd queue delete
bool omx_mp3_adec::omx_cmd_queue::delete_entry(unsigned *p1, unsigned *p2, unsigned *id)
{
  bool ret = true;
  if (m_size > 0)
  {
    *id = m_q[m_read].id;
    *p1 = m_q[m_read].param1;
    *p2 = m_q[m_read].param2;
    // Move the read pointer ahead
    ++m_read;
    --m_size;
    if(m_read >= OMX_CORE_CONTROL_CMDQ_SIZE)
    {
      m_read = 0;

    }
  }
  else
  {
    ret = false;
    DEBUG_PRINT_ERROR("ERROR Delete!!! Command Queue Full");
  }
  return ret;
}

// factory function executed by the core to create instances
void *get_omx_component_factory_fn(void)
{
  return (new omx_mp3_adec);
}


/* ======================================================================
FUNCTION
  omx_mp3_adec::omx_mp3_adec

DESCRIPTION
  Constructor

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
omx_mp3_adec::omx_mp3_adec(): m_state(OMX_StateInvalid),
                              m_app_data(NULL),
                              m_cmd_svr(NULL),
                              m_drv_fd(-1),
                              m_inp_buf_count(0),
                              m_flags(0),
                              output_buffer_size(OMX_MP3_OUTPUT_BUFFER_SIZE),
                              input_buffer_size(OMX_CORE_INPUT_BUFFER_SIZE)
{
  memset(&m_cmp,       0,     sizeof(m_cmp));
  memset(&m_cb,        0,      sizeof(m_cb));

  pthread_mutexattr_init(&m_lock_attr);
  pthread_mutex_init(&m_lock, &m_lock_attr);
  return;
}


/* ======================================================================
FUNCTION
  omx_mp3_adec::~omx_mp3_adec

DESCRIPTION
  Destructor

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
omx_mp3_adec::~omx_mp3_adec()
{
  pthread_mutexattr_destroy(&m_lock_attr);
  pthread_mutex_destroy(&m_lock);

  return;
}

/**
  @brief memory function for sending EmptyBufferDone event
   back to IL client

  @param bufHdr OMX buffer header to be passed back to IL client
  @return none
 */
void omx_mp3_adec::buffer_done_cb(OMX_BUFFERHEADERTYPE *bufHdr)
{

  if(m_cb.EmptyBufferDone)
  {
    PrintFrameHdr(bufHdr);

    m_cb.EmptyBufferDone(&m_cmp, m_app_data, bufHdr);
  }

  return;
}

void omx_mp3_adec::frame_done_cb(OMX_BUFFERHEADERTYPE *bufHdr)
{

  if(m_cb.FillBufferDone)
  {
    PrintFrameHdr(bufHdr);
    if (fcount == 0) {
        bufHdr->nTimeStamp = nTimestamp;
        DEBUG_PRINT(" frame_done_cb : time stamp of first output buffer = %d \n",bufHdr->nTimeStamp);
    }
    else
    {
        nTimestamp += frameDuration;
        bufHdr->nTimeStamp = nTimestamp;
        DEBUG_PRINT(" frame_done_cb : time stamp of output buffer = %d \n",bufHdr->nTimeStamp);
    }
    fcount++;

    m_cb.FillBufferDone(&m_cmp, m_app_data, bufHdr);
  }

  return;
}

/** ======================================================================
 @brief static member function for handling all commands from IL client

  IL client commands are processed and callbacks are generated
  through this routine. Audio Command Server provides the thread context
  for this routine.

  @param client_data pointer to decoder component
  @param id command identifier
  @return none
 */

void omx_mp3_adec::process_output_cb(void *client_data, unsigned char id)
{
  unsigned      p1; // Parameter - 1
  unsigned      p2; // Parameter - 2
  unsigned      ident;
  unsigned      qsize=0; // qsize
  static int    count = 0;
  omx_mp3_adec  *pThis              = (omx_mp3_adec *) client_data;
{

  DEBUG_PRINT("output callback OMXCntrlProessMsgCb[%x,%d] Enter:" , (unsigned) client_data,
              (unsigned)id);

  do
  {
    pthread_mutex_lock(&pThis->m_lock);

    qsize = pThis->m_output_q.m_size;

    if(qsize)
    {
      pThis->m_output_q.delete_entry(&p1,&p2,&ident);
    }
    if(qsize)
    {
      pThis->m_msg_cnt ++;
    }
    pthread_mutex_unlock(&pThis->m_lock);

    if(qsize > 0)
    {
      id = ident;
      DEBUG_PRINT("Process output callback ->%d[%d]ebd %d %x\n",pThis->m_state,ident, pThis->m_etb_cnt,
            pThis->m_flags >> 3);
      if(id == OMX_COMPONENT_GENERATE_FRAME_DONE)
      {
          DEBUG_PRINT(" processing OMX_COMPONENT_GENERATE_FRAME_DONE \n");
          pThis->frame_done_cb((OMX_BUFFERHEADERTYPE *)p2);
      }
      else if(id == OMX_COMPONENT_GENERATE_FTB)
      {
          DEBUG_PRINT(" processing OMX_COMPONENT_GENERATE_FTB \n");
          pThis->fill_this_buffer_proxy((OMX_HANDLETYPE)p1,(OMX_BUFFERHEADERTYPE *)p2);
      }
      else if(id == OMX_COMPONENT_GENERATE_EOS)
      {
          pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                     OMX_EventBufferFlag, 1, 1, NULL );
      }
      else
      {
        DEBUG_PRINT_ERROR("Error: ProcessMsgCb Ignored due to Invalid Identifier\n");
      }
      DEBUG_PRINT("OMXCntrlProessMsgCb[%x,%d] Exit: \n",
                  (unsigned)client_data,(unsigned)id);
    }
    else
    {
      DEBUG_PRINT("Error: ProcessMsgCb Ignored due to empty CmdQ\n");
    }


  } while(qsize>0);


  return;
}

}

void omx_mp3_adec::process_event_cb(void *client_data, unsigned char id)
{
  unsigned      p1; // Parameter - 1
  unsigned      p2; // Parameter - 2
  unsigned      ident;
  unsigned qsize=0; // qsize
  omx_mp3_adec  *pThis              = (omx_mp3_adec *) client_data;

  DEBUG_PRINT("OMXCntrlProessMsgCb[%x,%d] Enter:" , (unsigned) client_data,
              (unsigned)id);
  if(!pThis)
  {
    DEBUG_PRINT_ERROR("ERROR : ProcessMsgCb: Context is incorrect; bailing out\n");
    return;
  }

  // Protect the shared queue data structure
  do
  {
    pthread_mutex_lock(&pThis->m_lock);

    qsize = pThis->m_cmd_q.m_size;

    if(qsize)
    {
      pThis->m_cmd_q.delete_entry(&p1,&p2,&ident);
    } else {
      OMX_STATETYPE state;

      qsize = pThis->m_data_q.m_size;
      pThis->get_state(&pThis->m_cmp, &state);

      if ((qsize) && (state == OMX_StateExecuting))
      {
        pThis->m_data_q.delete_entry(&p1, &p2, &ident);
      } else
      {
        qsize = 0;
      }
    }

    if(qsize)
    {
      pThis->m_msg_cnt ++;
    }
    pthread_mutex_unlock(&pThis->m_lock);

    if(qsize > 0)
    {
      id = ident;
      DEBUG_PRINT("Process ->%d[%d]ebd %d %x\n",pThis->m_state,ident, pThis->m_etb_cnt,
            pThis->m_flags >> 3);
      if(id == OMX_COMPONENT_GENERATE_EVENT)
      {
        if (pThis->m_cb.EventHandler)
        {
          if (p1 == OMX_CommandStateSet)
          {
             pThis->m_state = (OMX_STATETYPE) p2;
             DEBUG_PRINT("Process -> state set to %d \n", pThis->m_state);
          }

          if (pThis->m_state == OMX_StateInvalid) {
            pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                     OMX_EventError, OMX_ErrorInvalidState,
                                     0, NULL );
          } else {
            pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                     OMX_EventCmdComplete, p1, p2, NULL );
          }

        }
        else
        {
          DEBUG_PRINT_ERROR("Error: ProcessMsgCb ignored due to NULL callbacks\n");
        }
      }
      else if(id == OMX_COMPONENT_GENERATE_BUFFER_DONE)
      {
        pThis->buffer_done_cb((OMX_BUFFERHEADERTYPE *)p2);
      }
      else if(id == OMX_COMPONENT_GENERATE_EOS)
      {
        pThis->m_cb.EventHandler(&pThis->m_cmp, pThis->m_app_data,
                                     OMX_EventBufferFlag, 1, 1, NULL );
      }
      /*else if(id == OMX_COMPONENT_GENERATE_FRAME_DONE)
      {
        frame_done_cb((struct vdec_context *)p1,(struct vdec_frame *)p2);
      }*/
      else if(id == OMX_COMPONENT_GENERATE_ETB)
      {
        pThis->empty_this_buffer_proxy((OMX_HANDLETYPE)p1,(OMX_BUFFERHEADERTYPE *)p2);
      }
      else if(id == OMX_COMPONENT_GENERATE_COMMAND)
      {
        pThis->send_command_proxy(&pThis->m_cmp,(OMX_COMMANDTYPE)p1,(OMX_U32)p2,(OMX_PTR)NULL);
      }
      else
      {
        DEBUG_PRINT_ERROR("Error: ProcessMsgCb Ignored due to Invalid Identifier\n");
      }
      DEBUG_PRINT("OMXCntrlProessMsgCb[%x,%d] Exit: \n",
                  (unsigned)client_data,(unsigned)id);
    }
    else
    {
      DEBUG_PRINT("Error: ProcessMsgCb Ignored due to empty CmdQ\n");
    }
    pthread_mutex_lock(&pThis->m_lock);
    qsize = pThis->m_cmd_q.m_size;
    pthread_mutex_unlock(&pThis->m_lock);
  } while(qsize>0);
  return;
}



/**
 @brief member function for performing component initialization

 @param role C string mandating role of this component
 @return Error status
 */
OMX_ERRORTYPE omx_mp3_adec::component_init(OMX_STRING role)
{

  OMX_ERRORTYPE eRet = OMX_ErrorNone;

  /* Ignore role */

  m_state                   = OMX_StateLoaded;
  /* DSP does not give information about the bitstream
     randomly assign the value right now. Query will result in
     incorrect param */
  memset(&m_adec_param, 0, sizeof(m_adec_param));
  m_adec_param.nSize = sizeof(m_adec_param);
  m_adec_param.nSampleRate = 44100;
  m_volume = 25; /* Close to unity gain */
  m_adec_param.nChannels = 2;

  /* default calculation of frame duration */
  frameDuration = (((4608)* 1000) / (44100 * 4));
  fcount = 0;
  nTimestamp = 0;
  DEBUG_PRINT(" Enabling Non-Tunnel mode \n");
  pcm_feedback = 1;    /* by default enable non-tunnel mode */
  ntotal_playtime = 0;

  DEBUG_PRINT(" component init: role = %s\n",role);

  if(!strcmp(role,"OMX.qcom.audio.decoder.mp3"))
  {
      pcm_feedback = 1;
      DEBUG_PRINT("\ncomponent_init: Component %s LOADED \n", role);
  }
  else if(!strcmp(role,"OMX.qcom.audio.decoder.tunneled.mp3"))
  {
      pcm_feedback = 0;
      DEBUG_PRINT("\ncomponent_init: Component %s LOADED \n", role);
  }
  else
  {
      DEBUG_PRINT("\ncomponent_init: Component %s LOADED is invalid\n", role);
  }

  return eRet;
}

/**

 @brief member function to retrieve version of component



 @param hComp handle to this component instance
 @param componentName name of component
 @param componentVersion  pointer to memory space which stores the
       version number
 @param specVersion pointer to memory sapce which stores version of
        openMax specification
 @param componentUUID
 @return Error status
 */
OMX_ERRORTYPE  omx_mp3_adec::get_component_version(OMX_IN OMX_HANDLETYPE               hComp,
                                                  OMX_OUT OMX_STRING          componentName,
                                                  OMX_OUT OMX_VERSIONTYPE* componentVersion,
                                                  OMX_OUT OMX_VERSIONTYPE*      specVersion,
                                                  OMX_OUT OMX_UUIDTYPE*       componentUUID)
{
  /* TBD -- Return the proper version */
  return OMX_ErrorNone;
}
/**
  @brief member function handles command from IL client

  This function simply queue up commands from IL client.
  Commands will be processed in command server thread context later

  @param hComp handle to component instance
  @param cmd type of command
  @param param1 parameters associated with the command type
  @param cmdData
  @return Error status
*/
OMX_ERRORTYPE  omx_mp3_adec::send_command(OMX_IN OMX_HANDLETYPE hComp,
                                          OMX_IN OMX_COMMANDTYPE  cmd,
                                          OMX_IN OMX_U32       param1,
                                          OMX_IN OMX_PTR      cmdData)
{
  if(!m_cmd_svr)
  {
    m_cmd_svr = adec_svr_start(process_event_cb, this);
    if(!m_cmd_svr)
    {
      m_cb.EventHandler(&m_cmp, m_app_data, OMX_EventError, cmd, param1, NULL );
      DEBUG_PRINT_ERROR("ERROR!!! comand server open failed\n");
      return OMX_ErrorHardware;
    }

  }

if(pcm_feedback)
{
  if(!m_cmd_cln)
  {
    m_cmd_cln = adec_cln_start(process_output_cb, this);
    if(!m_cmd_cln)
    {
      m_cb.EventHandler(&m_cmp, m_app_data, OMX_EventError, cmd, param1, NULL );
      DEBUG_PRINT_ERROR("ERROR!!! comand Client open failed\n");
      return OMX_ErrorHardware;
    }

  }
}

  post_event((unsigned)cmd,(unsigned)param1,OMX_COMPONENT_GENERATE_COMMAND, true);

  return OMX_ErrorNone;
}

/**
 @brief member function performs actual processing of commands excluding
  empty buffer call

 @param hComp handle to component
 @param cmd command type
 @param param1 parameter associated with the command
 @param cmdData

 @return error status
*/
OMX_ERRORTYPE  omx_mp3_adec::send_command_proxy(OMX_IN OMX_HANDLETYPE hComp,
                                          OMX_IN OMX_COMMANDTYPE  cmd,
                                          OMX_IN OMX_U32       param1,
                                          OMX_IN OMX_PTR      cmdData)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  //   Handle only IDLE and executing
  OMX_STATETYPE eState = (OMX_STATETYPE) param1;
  int bFlag = 1;

  if(cmd == OMX_CommandStateSet)
  {
    /***************************/
    /* Current State is Loaded */
    /***************************/
    if(m_state == OMX_StateLoaded)
    {
      if(eState == OMX_StateIdle)
      {
        m_drv_fd = open("/dev/msm_mp3", O_RDWR);

        if (m_drv_fd < 0) {
            DEBUG_PRINT_ERROR("OMXCORE-SM: device open fail Loaded -->Invalid\n");
            eState = OMX_StateInvalid;
        } else {
          if (allocate_done()) {
            DEBUG_PRINT("OMXCORE-SM: Loaded->Idle\n");
          } else {
            DEBUG_PRINT("OMXCORE-SM: Loaded-->Idle-Pending\n");
            BITMASK_SET(&m_flags, OMX_COMPONENT_IDLE_PENDING);
            bFlag = 0;
          }
        }
      }
      else
      {
        DEBUG_PRINT_ERROR("OMXCORE-SM: Loaded-->Invalid(%d Not Handled)\n",eState);
        eRet = OMX_ErrorBadParameter;
      }
    }

    /***************************/
    /* Current State is IDLE */
    /***************************/
    else if(m_state == OMX_StateIdle)
    {
      if(eState == OMX_StateLoaded)
      {
        if(release_done())
        {
      fsync(m_drv_fd);
          close(m_drv_fd);
          m_drv_fd = -1;
          DEBUG_PRINT("OMXCORE-SM: Idle-->Loaded\n");
        }
        else
        {
          DEBUG_PRINT("OMXCORE-SM: Idle-->Loaded-Pending\n");
          BITMASK_SET(&m_flags, OMX_COMPONENT_LOADING_PENDING);
          // Skip the event notification
          bFlag = 0;
        }
      }
      else if(eState == OMX_StateExecuting)
      {
        struct msm_audio_config drv_config;
        struct msm_audio_pcm_config  pcm_config;


        DEBUG_PRINT("configure Driver for MP3 playback sample rate = %d \n",m_adec_param.nSampleRate);
        ioctl(m_drv_fd, AUDIO_GET_CONFIG, &drv_config);
        drv_config.sample_rate = m_adec_param.nSampleRate;
        drv_config.channel_count = m_adec_param.nChannels;
        drv_config.type = 2; // mp3 decoding
        ioctl(m_drv_fd, AUDIO_SET_CONFIG, &drv_config);

        DEBUG_PRINT(" configure driver mode as %d \n",pcm_feedback);
        ioctl(m_drv_fd, AUDIO_GET_PCM_CONFIG, &pcm_config);
        pcm_config.pcm_feedback = pcm_feedback;
        pcm_config.buffer_size  = OMX_MP3_OUTPUT_BUFFER_SIZE ;
        ioctl(m_drv_fd, AUDIO_SET_PCM_CONFIG, &pcm_config);

        ioctl(m_drv_fd, AUDIO_START, 0);

        DEBUG_PRINT("OMXCORE-SM: Idle-->Executing\n");
      }
      else
      {
        DEBUG_PRINT_ERROR("OMXCORE-SM: Idle --> %d Not Handled\n",eState);
        eRet = OMX_ErrorBadParameter;
      }
    }

    /******************************/
    /* Current State is Executing */
    /******************************/
    else if(m_state == OMX_StateExecuting)
    {
       if(eState == OMX_StateIdle)
       {
         DEBUG_PRINT("OMXCORE-SM: Executing --> Idle \n");
         execute_omx_flush();
         ioctl(m_drv_fd, AUDIO_STOP, 0);

       }
       else if(eState == OMX_StatePause)
       {
         DEBUG_PRINT("OMXCORE-SM: Executing --> Paused \n");
       }
       else
       {
         DEBUG_PRINT_ERROR("OMXCORE-SM: Executing --> %d Not Handled\n",eState);
         eRet = OMX_ErrorBadParameter;
       }
    }
    /***************************/
    /* Current State is Pause  */
    /***************************/
    else if(m_state == OMX_StatePause)
    {
      if(eState == OMX_StateExecuting)
      {
        DEBUG_PRINT("OMXCORE-SM: Paused --> Executing \n");
      }
      else if(eState == OMX_StateIdle)
      {
        DEBUG_PRINT("OMXCORE-SM: Paused --> Idle \n");
        execute_omx_flush();
        ioctl(m_drv_fd, AUDIO_STOP, 0);

      }
      else
      {
        DEBUG_PRINT("OMXCORE-SM: Paused --> %d Not Handled\n",eState);
        eRet = OMX_ErrorBadParameter;
      }
    }
    else
    {
      DEBUG_PRINT_ERROR("OMXCORE-SM: %d --> %d(Not Handled)\n",m_state,eState);
      eRet = OMX_ErrorBadParameter;
    }
  }
  else if (cmd == OMX_CommandFlush)
  {
    execute_omx_flush();

    if ((param1 == OMX_CORE_INPUT_PORT_INDEX) ||
        (param1 == OMX_ALL))
    {
      post_event(OMX_CommandFlush,
                 OMX_CORE_INPUT_PORT_INDEX,OMX_COMPONENT_GENERATE_EVENT, true);
    } else
    {
      DEBUG_PRINT_ERROR("Flush wrong port ID");
    }

    bFlag = 0;
  }
  else if (cmd == OMX_CommandPortDisable) {

    if (param1 == OMX_CORE_OUTPUT_PORT_INDEX) {
      pcm_feedback = 0;    /* enable tunnel mode */
      DEBUG_PRINT(" Enabling Tunnel mode \n");
      post_event(OMX_CommandPortDisable,
                 OMX_CORE_OUTPUT_PORT_INDEX,OMX_COMPONENT_GENERATE_EVENT, true);
    }
    else
    {
      DEBUG_PRINT_ERROR("disable wrong port ID");
    }
    bFlag = 0;
  }
  else
  {
    DEBUG_PRINT_ERROR("Error: Invalid Command received other than StateSet (%d)\n",cmd);
    eRet = OMX_ErrorNotImplemented;
  }
  if(eRet == OMX_ErrorNone && bFlag)
  {
    post_event(cmd,eState,OMX_COMPONENT_GENERATE_EVENT, true);
  }
  return eRet;
}

/**
 @brief member function that flushes buffers that are pending to be written
  to driver

 @param none
 @return bool value indicating whether flushing is carried out successfully
*/
bool omx_mp3_adec::execute_omx_flush(void) /* Should flush be executed in order? */
{
  bool bRet = true;
  OMX_BUFFERHEADERTYPE *omx_buf;
  unsigned      p1; // Parameter - 1
  unsigned      p2; // Parameter - 2
  unsigned      ident;

  ioctl( m_drv_fd, AUDIO_FLUSH, 0);

  pthread_mutex_lock(&m_lock);

  DEBUG_PRINT("execute flush \n");
  while((m_data_q.delete_entry(&p1, &p2, &ident)) == true) {
    omx_buf = (OMX_BUFFERHEADERTYPE *) p2;

    DEBUG_PRINT("buf_addr=%x \n", omx_buf);
    post_event((unsigned) &m_cmp, (unsigned) omx_buf,
               OMX_COMPONENT_GENERATE_BUFFER_DONE, false);
  }

  pthread_mutex_unlock(&m_lock);

  return bRet;
}

/**
  @brief member function that posts command
  in the command queue

  @param p1 first paramter for the command
  @param p2 second parameter for the command
  @param id command ID
  @param lock self-locking mode
  @return bool indicating command being queued
 */
bool omx_mp3_adec::post_event(unsigned int p1,
                              unsigned int p2,
                              unsigned int id,
                              bool lock)
{
  bool bRet      =                      false;

  if (lock == true)
  {
    pthread_mutex_lock(&m_lock);
  }

  m_cmd_cnt ++;

  if (id == OMX_COMPONENT_GENERATE_ETB) {
    m_data_q.insert_entry(p1,p2,id);
  } else {
    m_cmd_q.insert_entry(p1,p2,id);
  }

  if(m_cmd_svr)
  {
    bRet = true;
    adec_svr_post_msg(m_cmd_svr, id);
  }

  if (lock == true) {
    pthread_mutex_unlock(&m_lock);
  }

  DEBUG_PRINT("Post -->%d[%d]ebd %d  %x \n",m_state,
          id, m_etb_cnt,
          m_flags >> 3);
  return bRet;
}


bool omx_mp3_adec::post_event_output(unsigned int p1,
                              unsigned int p2,
                              unsigned int id,
                              bool lock)
{
  bool bRet      =                      false;

  if (lock == true)
  {
    pthread_mutex_lock(&m_lock);
  }

  m_cmd_cnt ++;

  m_output_q.insert_entry(p1,p2,id);

  if(m_cmd_cln)
  {
    bRet = true;
    DEBUG_PRINT(" post_event_output ID = %d \n",id);
    adec_output_post_msg(m_cmd_cln, id);
  }

  if (lock == true) {
    pthread_mutex_unlock(&m_lock);
  }

  DEBUG_PRINT("Post -->%d[%d]ebd %d  %x \n",m_state,
          id, m_etb_cnt,
          m_flags >> 3);
  return bRet;
}
/**
  @brief member function that return parameters to IL client

  @param hComp handle to component instance
  @param paramIndex Parameter type
  @param paramData pointer to memory space which would hold the
        paramter
  @return error status
*/
OMX_ERRORTYPE  omx_mp3_adec::get_parameter(OMX_IN OMX_HANDLETYPE     hComp,
                                           OMX_IN OMX_INDEXTYPE paramIndex,
                                           OMX_INOUT OMX_PTR     paramData)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;

  switch(paramIndex)
  {
    case OMX_IndexParamPortDefinition:
    {
      OMX_PARAM_PORTDEFINITIONTYPE *portDefn;
      portDefn = (OMX_PARAM_PORTDEFINITIONTYPE *) paramData;

      DEBUG_PRINT("OMX_IndexParamPortDefinition portDefn->nPortIndex = %d\n",portDefn->nPortIndex);

        portDefn->nVersion.nVersion = OMX_SPEC_VERSION;
        portDefn->nSize = sizeof(portDefn);
        portDefn->bEnabled   = OMX_TRUE;
        portDefn->bPopulated = OMX_TRUE;
        portDefn->eDomain    = OMX_PortDomainAudio;

      if (0 == portDefn->nPortIndex)
      {
        portDefn->eDir =  OMX_DirInput;
        if (m_input_buf_hdrs.size() >= OMX_CORE_NUM_INPUT_BUFFERS) {
          portDefn->bPopulated = OMX_TRUE;
        } else {
          portDefn->bPopulated = OMX_FALSE;
        }
        /* What if the component does not restrict how many buffer to take */
        portDefn->nBufferCountActual = OMX_CORE_NUM_INPUT_BUFFERS;
        portDefn->nBufferCountMin    = OMX_CORE_NUM_INPUT_BUFFERS;
        portDefn->nBufferSize        = input_buffer_size;
        portDefn->format.audio.bFlagErrorConcealment = OMX_TRUE;
        if (portDefn->format.audio.cMIMEType != NULL) {
          memcpy(portDefn->format.audio.cMIMEType, "audio/mpeg", sizeof("audio/mpeg"));
        }
        portDefn->format.audio.eEncoding = OMX_AUDIO_CodingMP3;
        portDefn->format.audio.pNativeRender = 0;


      }
      else if (1 == portDefn->nPortIndex)
      {
        portDefn->eDir =  OMX_DirOutput;
        /* What if the component does not restrict how many buffer to take */
        portDefn->nBufferCountActual = 2;
        portDefn->nBufferCountMin    = 2;
        portDefn->nBufferSize = output_buffer_size;
        portDefn->format.audio.bFlagErrorConcealment = OMX_TRUE;
        portDefn->format.audio.eEncoding = OMX_AUDIO_CodingPCM;
        portDefn->format.audio.pNativeRender = 0;

      }
      else
      {
        portDefn->eDir =  OMX_DirMax;
        DEBUG_PRINT_ERROR("Bad Port idx %d\n", (int)portDefn->nPortIndex);
        eRet = OMX_ErrorBadPortIndex;
      }

      break;
    }

    case OMX_IndexParamAudioInit:
    {
      OMX_PORT_PARAM_TYPE *portParamType =
                              (OMX_PORT_PARAM_TYPE *) paramData;
      DEBUG_PRINT("OMX_IndexParamAudioInit\n");

      portParamType->nVersion.nVersion = OMX_SPEC_VERSION;
      portParamType->nSize = sizeof(portParamType);
      portParamType->nPorts           = 2;
      portParamType->nStartPortNumber = 0;
      break;
    }

    case OMX_IndexParamAudioPortFormat:
    {
      OMX_AUDIO_PARAM_PORTFORMATTYPE *portFormatType =
                                  (OMX_AUDIO_PARAM_PORTFORMATTYPE *) paramData;
      DEBUG_PRINT("OMX_IndexParamAudioPortFormat\n");
      portFormatType->nVersion.nVersion = OMX_SPEC_VERSION;
      portFormatType->nSize = sizeof(portFormatType);

      if (OMX_CORE_INPUT_PORT_INDEX == portFormatType->nPortIndex)
      {
        if (0 == portFormatType->nIndex) { /* What is the intention of nIndex */
          portFormatType->eEncoding = OMX_AUDIO_CodingMP3;
        }
      } else {
        eRet = OMX_ErrorBadPortIndex;
      }
      break;
    }

    case OMX_IndexParamAudioMp3:
    {
      OMX_AUDIO_PARAM_MP3TYPE *mp3Param = (OMX_AUDIO_PARAM_MP3TYPE *) paramData;
      DEBUG_PRINT("OMX_IndexParamAudioMp3\n");
      *mp3Param = m_adec_param;

      break;
    }

    default:
    {
      DEBUG_PRINT_ERROR("unknown param %08x\n", paramIndex);
      eRet = OMX_ErrorBadParameter;
    }

  }

  return eRet;

}

/**
 @brief member function that set paramter from IL client

 @param hComp handle to component instance
 @param paramIndex parameter type
 @param paramData pointer to memory space which holds the paramter
 @return error status
 */
OMX_ERRORTYPE  omx_mp3_adec::set_parameter(OMX_IN OMX_HANDLETYPE     hComp,
                                           OMX_IN OMX_INDEXTYPE paramIndex,
                                           OMX_IN OMX_PTR        paramData)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  int           i;

  switch(paramIndex)
  {

    case OMX_IndexParamAudioMp3:
    {
      DEBUG_PRINT("OMX_IndexParamAudioMP3");
      m_adec_param = *((OMX_AUDIO_PARAM_MP3TYPE *) paramData);

      if(m_adec_param.nChannels == 1)
        {
            frameDuration = (((OMX_MP3_OUTPUT_BUFFER_SIZE)* 1000) /(m_adec_param.nSampleRate * 2));
            DEBUG_PRINT("frame duration of mono config = %d sampling rate = %d \n",
                         frameDuration,m_adec_param.nSampleRate);
        }
        else if(m_adec_param.nChannels == 2)
        {
            frameDuration = (((OMX_MP3_OUTPUT_BUFFER_SIZE)* 1000) /(m_adec_param.nSampleRate * 4));
            DEBUG_PRINT("frame duration of stero config = %d sampling rate = %d \n",
                         frameDuration,m_adec_param.nSampleRate);

        }
      break;
    }

    default:
    {
      DEBUG_PRINT_ERROR("unknown param %d\n", paramIndex);
      eRet = OMX_ErrorUnsupportedIndex;
    }
  }

  return eRet;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::GetConfig

DESCRIPTION
  OMX Get Config Method implementation.

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if successful.

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::get_config(OMX_IN OMX_HANDLETYPE      hComp,
                                        OMX_IN OMX_INDEXTYPE configIndex,
                                        OMX_INOUT OMX_PTR     configData)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;


  switch(configIndex)
  {
  case OMX_IndexConfigAudioVolume:
    {
      OMX_AUDIO_CONFIG_VOLUMETYPE *volume = (OMX_AUDIO_CONFIG_VOLUMETYPE*) configData;

      if (volume->nPortIndex == OMX_CORE_INPUT_PORT_INDEX)
      {
        volume->nSize = sizeof(volume);
        volume->nVersion.nVersion = OMX_SPEC_VERSION;
        volume->bLinear = OMX_TRUE;
        volume->sVolume.nValue = m_volume;
        volume->sVolume.nMax   = OMX_ADEC_MAX;
        volume->sVolume.nMin   = OMX_ADEC_MIN;
      } else {
        eRet = OMX_ErrorBadPortIndex;
      }
    }
    break;

  case OMX_IndexConfigAudioMute:
    {
      OMX_AUDIO_CONFIG_MUTETYPE *mute = (OMX_AUDIO_CONFIG_MUTETYPE*) configData;

      if (mute->nPortIndex == OMX_CORE_INPUT_PORT_INDEX)
      {
        mute->nSize = sizeof(mute);
        mute->nVersion.nVersion = OMX_SPEC_VERSION;
        mute->bMute = (BITMASK_PRESENT(&m_flags, OMX_COMPONENT_MUTED)?OMX_TRUE:OMX_FALSE);
      } else {
        eRet = OMX_ErrorBadPortIndex;
      }
    }
    break;

  default:
    eRet = OMX_ErrorUnsupportedIndex;
    break;
  }
  return eRet;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::SetConfig

DESCRIPTION
  OMX Set Config method implementation

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if successful.
========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::set_config(OMX_IN OMX_HANDLETYPE      hComp,
                                        OMX_IN OMX_INDEXTYPE configIndex,
                                        OMX_IN OMX_PTR        configData)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;


  switch(configIndex)
  {
    case OMX_IndexConfigAudioVolume:
    {
      OMX_AUDIO_CONFIG_VOLUMETYPE *vol = (OMX_AUDIO_CONFIG_VOLUMETYPE*)
                                          configData;
      if (vol->nPortIndex == OMX_CORE_INPUT_PORT_INDEX)
      {
        if ((vol->sVolume.nValue <= OMX_ADEC_MAX) &&
            (vol->sVolume.nValue >= OMX_ADEC_MIN)) {
          m_volume = vol->sVolume.nValue;
          if (BITMASK_ABSENT(&m_flags, OMX_COMPONENT_MUTED))
          {
            /* ioctl(m_drv_fd, AUDIO_VOLUME, m_volume * OMX_ADEC_VOLUME_STEP); */
          }

        } else {
          eRet = OMX_ErrorBadParameter;
        }
      } else {
        eRet = OMX_ErrorBadPortIndex;
      }
    }
    break;

  case OMX_IndexConfigAudioMute:
    {
      OMX_AUDIO_CONFIG_MUTETYPE *mute = (OMX_AUDIO_CONFIG_MUTETYPE*)
                                                          configData;
      if (mute->nPortIndex == OMX_CORE_INPUT_PORT_INDEX)
      {
        if (mute->bMute == OMX_TRUE) {
          BITMASK_SET(&m_flags, OMX_COMPONENT_MUTED);
          /* ioctl(m_drv_fd, AUDIO_VOLUME, 0); */
        } else {
          BITMASK_CLEAR(&m_flags, OMX_COMPONENT_MUTED);
          /* ioctl(m_drv_fd, AUDIO_VOLUME, m_volume * OMX_ADEC_VOLUME_STEP); */
        }
      } else {
        eRet = OMX_ErrorBadPortIndex;
      }
    }
    break;

  default:
    eRet = OMX_ErrorUnsupportedIndex;
    break;
  }
  return eRet;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::GetExtensionIndex

DESCRIPTION
  OMX GetExtensionIndex method implementaion.  <TBD>

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::get_extension_index(OMX_IN OMX_HANDLETYPE      hComp,
                                                OMX_IN OMX_STRING      paramName,
                                                OMX_OUT OMX_INDEXTYPE* indexType)
{
  DEBUG_PRINT_ERROR("Error, Not implemented\n");
  return OMX_ErrorNotImplemented;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::GetState

DESCRIPTION
  Returns the state information back to the caller.<TBD>

PARAMETERS
  <TBD>.

RETURN VALUE
  Error None if everything is successful.
========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::get_state(OMX_IN OMX_HANDLETYPE  hComp,
                                       OMX_OUT OMX_STATETYPE* state)
{
  *state = m_state;
  DEBUG_PRINT("Returning the state %d\n",*state);
  return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::ComponentTunnelRequest

DESCRIPTION
  OMX Component Tunnel Request method implementation. <TBD>

PARAMETERS
  None.

RETURN VALUE
  OMX Error None if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::component_tunnel_request(OMX_IN OMX_HANDLETYPE                hComp,
                                                     OMX_IN OMX_U32                        port,
                                                     OMX_IN OMX_HANDLETYPE        peerComponent,
                                                     OMX_IN OMX_U32                    peerPort,
                                                     OMX_INOUT OMX_TUNNELSETUPTYPE* tunnelSetup)
{
  DEBUG_PRINT_ERROR("Error: component_tunnel_request Not Implemented\n");
  return OMX_ErrorNotImplemented;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::UseBuffer

DESCRIPTION
  OMX Use Buffer method implementation. <TBD>

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None , if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::use_buffer(OMX_IN OMX_HANDLETYPE                hComp,
                                        OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                                        OMX_IN OMX_U32                        port,
                                        OMX_IN OMX_PTR                     appData,
                                        OMX_IN OMX_U32                       bytes,
                                        OMX_IN OMX_U8*                      buffer)
{
  DEBUG_PRINT_ERROR("Error: use_buffer Not implemented\n");
  return OMX_ErrorNotImplemented;
}


/* ======================================================================
FUNCTION
  omx_mp3_adec::AllocateInputBuffer

DESCRIPTION
  Helper function for allocate buffer in the input pin

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::allocate_input_buffer(OMX_IN OMX_HANDLETYPE                hComp,
                                                  OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                                                  OMX_IN OMX_U32                        port,
                                                  OMX_IN OMX_PTR                     appData,
                                                  OMX_IN OMX_U32                       bytes)
{
  OMX_ERRORTYPE         eRet = OMX_ErrorNone;
  OMX_BUFFERHEADERTYPE            *bufHdr; // buffer header
  unsigned                   nBufSize = max(bytes, OMX_CORE_INPUT_BUFFER_SIZE);
  char                       *buf_ptr;

  buf_ptr = (char *) calloc( (nBufSize + sizeof(OMX_BUFFERHEADERTYPE) ) , 1);

  if (buf_ptr != NULL) {
    bufHdr = (OMX_BUFFERHEADERTYPE *) buf_ptr;
    *bufferHdr = bufHdr;
    memset(bufHdr,0,sizeof(OMX_BUFFERHEADERTYPE));


    bufHdr->pBuffer           = (OMX_U8 *)((buf_ptr) +
                                           sizeof(OMX_BUFFERHEADERTYPE));
    DEBUG_PRINT("bufHdr %x bufHdr->pBuffer %x", bufHdr, bufHdr->pBuffer);
    bufHdr->nSize             = sizeof(OMX_BUFFERHEADERTYPE);
    bufHdr->nVersion.nVersion = OMX_SPEC_VERSION;
    bufHdr->nAllocLen         = nBufSize;
    bufHdr->pAppPrivate       = appData;
    bufHdr->nInputPortIndex   = OMX_CORE_INPUT_PORT_INDEX;
    m_input_buf_hdrs.insert(bufHdr, NULL);
    m_inp_buf_count++;
  } else {
    DEBUG_PRINT("Input buffer memory allocation failed\n");
    eRet =  OMX_ErrorInsufficientResources;
  }

  return eRet;
}

OMX_ERRORTYPE  omx_mp3_adec::allocate_output_buffer(OMX_IN OMX_HANDLETYPE                hComp,
                                                  OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                                                  OMX_IN OMX_U32                        port,
                                                  OMX_IN OMX_PTR                     appData,
                                                  OMX_IN OMX_U32                       bytes)
{
  OMX_ERRORTYPE         eRet = OMX_ErrorNone;
  OMX_BUFFERHEADERTYPE            *bufHdr; // buffer header
  unsigned                   nBufSize = max(bytes, OMX_MP3_OUTPUT_BUFFER_SIZE);
  char                       *buf_ptr;

  buf_ptr = (char *) calloc( (nBufSize + sizeof(OMX_BUFFERHEADERTYPE) ) , 1);

  if (buf_ptr != NULL) {
    bufHdr = (OMX_BUFFERHEADERTYPE *) buf_ptr;
    *bufferHdr = bufHdr;
    memset(bufHdr,0,sizeof(OMX_BUFFERHEADERTYPE));

    bufHdr->pBuffer           = (OMX_U8 *)((buf_ptr) +
                                           sizeof(OMX_BUFFERHEADERTYPE));
    DEBUG_PRINT("bufHdr %x bufHdr->pBuffer %x", bufHdr, bufHdr->pBuffer);
    bufHdr->nSize             = sizeof(OMX_BUFFERHEADERTYPE);
    bufHdr->nVersion.nVersion = OMX_SPEC_VERSION;
    bufHdr->nAllocLen         = nBufSize;
    bufHdr->pAppPrivate       = appData;
    bufHdr->nInputPortIndex   = OMX_CORE_OUTPUT_PORT_INDEX;
    m_output_buf_hdrs.insert(bufHdr, NULL);
    m_out_buf_count++;
    if(m_out_buf_count == 1) {
        pFirstOutputBuf = (OMX_U8 *)bufHdr->pBuffer;
    }
    if(m_out_buf_count == 2) {
        pSecondOutputBuf = (OMX_U8 *)bufHdr->pBuffer;
    }

  } else {
    DEBUG_PRINT("Output buffer memory allocation failed\n");
    eRet =  OMX_ErrorInsufficientResources;
  }

  return eRet;
}


// AllocateBuffer  -- API Call
/* ======================================================================
FUNCTION
  omx_mp3_adec::AllocateBuffer

DESCRIPTION
  Returns zero if all the buffers released..

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::allocate_buffer(OMX_IN OMX_HANDLETYPE                hComp,
                                     OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                                     OMX_IN OMX_U32                        port,
                                     OMX_IN OMX_PTR                     appData,
                                     OMX_IN OMX_U32                       bytes)
{

    OMX_ERRORTYPE eRet = OMX_ErrorNone;


    // What if the client calls again.
    if(port == OMX_CORE_INPUT_PORT_INDEX)
    {
      eRet = allocate_input_buffer(hComp,bufferHdr,port,appData,bytes);
    }
    else if(port == OMX_CORE_OUTPUT_PORT_INDEX)
    {
      eRet = allocate_output_buffer(hComp,bufferHdr,port,appData,bytes);
    }
    else
    {
      DEBUG_PRINT_ERROR("Error: Invalid Port Index received %d\n",
                        (int)port);
      eRet = OMX_ErrorBadPortIndex;
    }

    if((eRet == OMX_ErrorNone) && (BITMASK_PRESENT(&m_flags,OMX_COMPONENT_IDLE_PENDING)))
    {
      DEBUG_PRINT("Checking for Output Allocate buffer Done");
      if(allocate_done())
      {
        // Send the callback now
        BITMASK_CLEAR(&m_flags, OMX_COMPONENT_IDLE_PENDING);
        post_event(OMX_CommandStateSet,OMX_StateIdle,
                   OMX_COMPONENT_GENERATE_EVENT, true);
      }
    }
    DEBUG_PRINT("Allocate Buffer exit with ret Code %d\n", eRet);
    return eRet;
}

/**
 @brief member function that searches for caller buffer

 @param buffer pointer to buffer header
 @return bool value indicating whether buffer is found
 */
bool omx_mp3_adec::search_input_bufhdr(OMX_BUFFERHEADERTYPE *buffer)
{
  bool eRet = false;
  OMX_BUFFERHEADERTYPE *temp;

  //access only in IL client context
  temp = m_input_buf_hdrs.find_ele(buffer);
  if(buffer && temp)
  {
      DEBUG_PRINT("found %x \n", buffer);
      eRet = true;
  }

  return eRet;
}

/**
 @brief member function that searches for caller buffer

 @param buffer pointer to buffer header
 @return bool value indicating whether buffer is found
 */
bool omx_mp3_adec::search_output_bufhdr(OMX_BUFFERHEADERTYPE *buffer)
{
  bool eRet = false;
  OMX_BUFFERHEADERTYPE *temp;


  //access only in IL client context
  temp = m_output_buf_hdrs.find_ele(buffer);
  if(buffer && temp)
  {
      DEBUG_PRINT("found %x \n", buffer);
      eRet = true;
  }

  return eRet;
}

// Free Buffer - API call
/**
  @brief member function that handles free buffer command from IL client

  This function is a block-call function that handles IL client request to
  freeing the buffer

  @param hComp handle to component instance
  @param port id of port which holds the buffer
  @param buffer buffer header
  @return Error status
*/
OMX_ERRORTYPE  omx_mp3_adec::free_buffer(OMX_IN OMX_HANDLETYPE         hComp,
                                      OMX_IN OMX_U32                 port,
                                      OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;

  DEBUG_PRINT("buf %x\n", buffer);


  if(port == OMX_CORE_INPUT_PORT_INDEX)
  {
    if(search_input_bufhdr(buffer) == true)
    {
      /* Buffer exist */
      //access only in IL client context
      m_input_buf_hdrs.erase(buffer);
      m_inp_buf_count--;
    } else {
      DEBUG_PRINT_ERROR("Error: free_buffer , invalid Input buffer header\n");
      eRet = OMX_ErrorBadParameter;
    }
  }
  else if(port == OMX_CORE_OUTPUT_PORT_INDEX)
  {
   if(search_output_bufhdr(buffer) == true)
    {
      /* Buffer exist */
      //access only in IL client context
      m_output_buf_hdrs.erase(buffer);
      m_out_buf_count--;
    } else {
      DEBUG_PRINT_ERROR("Error: free_buffer , invalid Output buffer header\n");
      eRet = OMX_ErrorBadParameter;
    }
  }
  else
  {
    eRet = OMX_ErrorBadPortIndex;
  }

  if((eRet == OMX_ErrorNone) &&
     (BITMASK_PRESENT(&m_flags ,OMX_COMPONENT_LOADING_PENDING)))
  {
    if(release_done())
    {
      // Send the callback now
      BITMASK_CLEAR((&m_flags),OMX_COMPONENT_LOADING_PENDING);
      post_event(OMX_CommandStateSet,
                 OMX_StateLoaded,OMX_COMPONENT_GENERATE_EVENT, false);
    }
  }

  return eRet;
}


/**
 @brief member function that that handles empty this buffer command

 This function meremly queue up the command and data would be consumed
 in command server thread context

 @param hComp handle to component instance
 @param buffer pointer to buffer header
 @return error status
 */
OMX_ERRORTYPE  omx_mp3_adec::empty_this_buffer(OMX_IN OMX_HANDLETYPE         hComp,
                                              OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  bool bPost = false;

  pthread_mutex_lock(&m_lock);
  if ((m_state != OMX_StateExecuting) &&
      (m_state != OMX_StatePause))
  {
    DEBUG_PRINT_ERROR("Invalid state\n");
    eRet = OMX_ErrorInvalidState;
  }
  pthread_mutex_unlock(&m_lock);

  if (eRet == OMX_ErrorNone) {
    if (search_input_bufhdr(buffer) == true) {
      post_event((unsigned)hComp,
                 (unsigned) buffer,OMX_COMPONENT_GENERATE_ETB, false);
    } else {
      DEBUG_PRINT_ERROR("Bad header %x \n", buffer);
      eRet = OMX_ErrorBadParameter;
    }
  }

  return eRet;
}
/**
  @brief member function that writes data to kernel driver

  @param hComp handle to component instance
  @param buffer pointer to buffer header
  @return error status
 */
OMX_ERRORTYPE  omx_mp3_adec::empty_this_buffer_proxy(OMX_IN OMX_HANDLETYPE         hComp,
                                                     OMX_BUFFERHEADERTYPE* buffer)
{

  /* Assume empty this buffer function has already checked
     validity of buffer */
  DEBUG_PRINT("Empty buffer %x to kernel driver\n", buffer);

   DEBUG_PRINT ("\n Before Write");
  if(buffer->nFlags & OMX_BUFFERFLAG_EOS)
  {
      DEBUG_PRINT("ETBP:EOS OCCURED \n");
      ntotal_playtime = buffer->nTimeStamp;

     if(buffer->nFilledLen)
     {
        // First send non-zero byte data w/o EOS flag set.
        buffer->nFlags &= ~OMX_BUFFERFLAG_EOS;
        write(m_drv_fd, buffer->pBuffer, buffer->nFilledLen);
     }
      // Now zero write buffer w EOS flags set.
      buffer->nFilledLen = 0;
      buffer->nFlags |= OMX_BUFFERFLAG_EOS;
      write(m_drv_fd, buffer->pBuffer, buffer->nFilledLen);
      if(pcm_feedback == 0)
      {
        fsync(m_drv_fd);
        post_event((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_GENERATE_EOS, true);
      }

  }
  else
  {
      DEBUG_PRINT("ETBP: before write nFlags[%d] len[%d]\n",buffer->nFlags, buffer->nFilledLen);
      write(m_drv_fd, buffer->pBuffer, buffer->nFilledLen);
      DEBUG_PRINT("ETBP:after write\n");
  }

  post_event((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_GENERATE_BUFFER_DONE, true);

  return OMX_ErrorNone;
}

OMX_ERRORTYPE  omx_mp3_adec::fill_this_buffer_proxy(OMX_IN OMX_HANDLETYPE         hComp,
                                                     OMX_BUFFERHEADERTYPE* buffer)
{
  static int count = 0;
  int nDatalen = 0;


  /* Assume fill this buffer function has already checked
     validity of buffer */
  DEBUG_PRINT("Fill buffer %x \n", buffer);

  if(search_output_bufhdr(buffer) == true)
  {
     int nRead = 0;
     int nReadbytes = 0;
     OMX_IN OMX_U8  *pBuf = NULL;
     pBuf = buffer->pBuffer;
     DEBUG_PRINT("FTBP: before read \n");
     nReadbytes = read(m_drv_fd, pBuf, buffer->nAllocLen);
     DEBUG_PRINT("FTBP: after read buffer %p  of size = %d\n",buffer->pBuffer, nReadbytes);

     if (nReadbytes < 0)
     {
        DEBUG_PRINT("FTBP: breaking read since nReadbytes is -1 ");
        buffer->nFilledLen = 0;
        post_event_output((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_GENERATE_FRAME_DONE, true);
        return OMX_ErrorNone;
     }
     else if(nReadbytes == 0)
     {
        buffer->nFlags = OMX_BUFFERFLAG_EOS;
     }
     nDatalen = nDatalen + nReadbytes;

     DEBUG_PRINT("FTBP: Total Number of bytes read %d", nDatalen);
     buffer->nFilledLen = nDatalen;
     if ( (nDatalen < 0) || (nDatalen > buffer->nAllocLen) )
     {
       buffer->nFilledLen = 0;
       DEBUG_PRINT("1.nflags[%d] nFilledLen[%d]",buffer->nFlags,buffer->nFilledLen);
       post_event_output((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_GENERATE_FRAME_DONE, true);
     }
     else
     {
        DEBUG_PRINT("2.nflags[%d] nFilledLen[%d]",buffer->nFlags,buffer->nFilledLen);

        if((buffer->nFlags == OMX_BUFFERFLAG_EOS) &&
             (nTimestamp >= ntotal_playtime))
        {

              DEBUG_PRINT("FTBP: END OF STREAM \n");
              DEBUG_PRINT("FTBP: buffer timestamp = %d Total playtime = %d \n",nTimestamp,ntotal_playtime);
              {
                post_event_output((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_GENERATE_EOS, true);

              }
              post_event_output((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_GENERATE_FRAME_DONE, true);
              return OMX_ErrorNone;
        }
        else
        {
             DEBUG_PRINT("FTBP: VALID DATA LENGTH datalen=%d nflags=%d ts[%ld] tot_time[%ld]\n",
                                buffer->nFilledLen, buffer->nFlags, nTimestamp, ntotal_playtime);
        }
     }
  }
  else
  {
      DEBUG_PRINT("\n Invalid buffer in FTB \n");
  }

  return OMX_ErrorNone;

}


/* ======================================================================
FUNCTION
  omx_mp3_adec::FillThisBuffer

DESCRIPTION
  IL client uses this method to release the frame buffer
  after displaying them.

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::fill_this_buffer(OMX_IN OMX_HANDLETYPE         hComp,
                                                  OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;

    DEBUG_PRINT(" inside fill_this_buffer \n");
    post_event_output((unsigned)hComp,
                 (unsigned) buffer,OMX_COMPONENT_GENERATE_FTB, false);
    return eRet; //OMX_ErrorNotImplemented;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::SetCallbacks

DESCRIPTION
  Set the callbacks.

PARAMETERS
  None.

RETURN VALUE
  OMX Error None if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::set_callbacks(OMX_IN OMX_HANDLETYPE        hComp,
                                           OMX_IN OMX_CALLBACKTYPE* callbacks,
                                           OMX_IN OMX_PTR             appData)
{
  m_cb       = *callbacks;
  m_app_data =    appData;

  return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::ComponentDeInit

DESCRIPTION
  Destroys the component and release memory allocated to the heap.

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::component_deinit(OMX_IN OMX_HANDLETYPE hComp)
{
  if (OMX_StateLoaded != m_state)
  {
      DEBUG_PRINT_ERROR("Warning: Received DeInit when not in LOADED state, cur_state %d\n",
                   m_state);
      return OMX_ErrorInvalidState;
  }

  if (m_cmd_svr != NULL) {
    adec_svr_stop(m_cmd_svr);
    m_cmd_svr = NULL;
  }
  if(pcm_feedback ==1)
  {
    if (m_cmd_cln != NULL) {
      adec_cln_stop(m_cmd_cln);
      m_cmd_cln = NULL;
    }
  }

  if (m_drv_fd >= 0) {
    close(m_drv_fd);
  }
  else
  {
    DEBUG_PRINT_ERROR(" mp3 device close failure \n");
  }

  return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::UseEGLImage

DESCRIPTION
  OMX Use EGL Image method implementation <TBD>.

PARAMETERS
  <TBD>.

RETURN VALUE
  Not Implemented error.

========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::use_EGL_image(OMX_IN OMX_HANDLETYPE                hComp,
                                          OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                                          OMX_IN OMX_U32                        port,
                                          OMX_IN OMX_PTR                     appData,
                                          OMX_IN void*                      eglImage)
{
    DEBUG_PRINT_ERROR("Error : use_EGL_image:  Not Implemented \n");
    return OMX_ErrorNotImplemented;
}

/* ======================================================================
FUNCTION
  omx_mp3_adec::ComponentRoleEnum

DESCRIPTION
  OMX Component Role Enum method implementation.

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if everything is successful.
========================================================================== */
OMX_ERRORTYPE  omx_mp3_adec::component_role_enum(OMX_IN OMX_HANDLETYPE hComp,
                                                OMX_OUT OMX_U8*        role,
                                                OMX_IN OMX_U32        index)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  const char *cmp_role = "audio_decoder.mp3";

  if(index == 0 && role)
  {
    memcpy(role, cmp_role, sizeof(cmp_role));
    *(((char *) role) + sizeof(cmp_role)) = '\0';
  }
  else
  {
    eRet = OMX_ErrorNoMore;
  }
  return eRet;
}




/* ======================================================================
FUNCTION
  omx_mp3_adec::AllocateDone

DESCRIPTION
  Checks if entire buffer pool is allocated by IL Client or not.
  Need this to move to IDLE state.

PARAMETERS
  None.

RETURN VALUE
  true/false.

========================================================================== */
bool omx_mp3_adec::allocate_done(void)
{
  return (m_inp_buf_count >= OMX_CORE_NUM_INPUT_BUFFERS?true:false);
}


/* ======================================================================
FUNCTION
  omx_mp3_adec::ReleaseDone

DESCRIPTION
  Checks if IL client has released all the buffers.

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
bool omx_mp3_adec::release_done(void)
{
  return (m_inp_buf_count == 0?true:false);

}


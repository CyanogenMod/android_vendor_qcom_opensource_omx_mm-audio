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

  @file omx_adec_aac.c
  This module contains the implementation of the OpenMAX Audio AAC component.

*//*========================================================================*/


//////////////////////////////////////////////////////////////////////////////
//                             Include Files
////////////////////////////////////////////////////////////////////////////

#include<string.h>
#include <fcntl.h>
#include <omx_aac_adec.h>
#include <sys/ioctl.h>

using namespace std;

OMX_U32 aac_frequency_index[16] = {
    96000,
    88200,
    64000,
    48000,
    44100,
    32000,
    24000,
    22050,
    16000,
    12000,
    11025,
    8000,
    7350,
    0x000,//invalid index
    0x000,//invalid index
    0x000 // no index, value provided is actual frequency
};

/********************************************************************/
//#if 0 to be deleted from here once msm_audio_aac.h is included here
#define AUDIO_IOCTL_MAGIC 'a'
#define AUDIO_MAX_COMMON_IOCTL_NUM 100

#define AUDIO_START        _IOW(AUDIO_IOCTL_MAGIC, 0, unsigned)
#define AUDIO_STOP         _IOW(AUDIO_IOCTL_MAGIC, 1, unsigned)
#define AUDIO_FLUSH        _IOW(AUDIO_IOCTL_MAGIC, 2, unsigned)
#define AUDIO_GET_CONFIG   _IOR(AUDIO_IOCTL_MAGIC, 3, unsigned)
#define AUDIO_SET_CONFIG   _IOW(AUDIO_IOCTL_MAGIC, 4, unsigned)
#define AUDIO_GET_STATS    _IOR(AUDIO_IOCTL_MAGIC, 5, unsigned)
#define AUDIO_GET_PCM_CONFIG _IOR(AUDIO_IOCTL_MAGIC, 30, unsigned)
#define AUDIO_SET_PCM_CONFIG _IOW(AUDIO_IOCTL_MAGIC, 31, unsigned)
#define AUDIO_PAUSE        _IOW(AUDIO_IOCTL_MAGIC, 11, unsigned)
#define AUDIO_SET_AAC_CONFIG  _IOW(AUDIO_IOCTL_MAGIC, \
  (AUDIO_MAX_COMMON_IOCTL_NUM+0), unsigned)
#define AUDIO_GET_AAC_CONFIG  _IOR(AUDIO_IOCTL_MAGIC, \
  (AUDIO_MAX_COMMON_IOCTL_NUM+1), unsigned)
//#endif to be deleted till here
/********************************************************************/

#define OMX_ADEC_VOLUME_STEP 0x147
#define OMX_ADEC_MIN         0
#define OMX_ADEC_MAX         100


#define NON_TUNNEL  1
#define TUNNEL      0

/*************************************************/
//#if 0 to be deleted from here once msm_audio_aac.h is included here

#define AUDIO_AAC_FORMAT_ADTS           -1
#define AUDIO_AAC_FORMAT_RAW            0x0000
#define AUDIO_AAC_FORMAT_PSUEDO_RAW     0x0001
#define AUDIO_AAC_FORMAT_LOAS       0x0002

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
    uint32_t buffer_size;    /* Size of buffer for capturing of PCM samples */
};

struct msm_audio_aac_config {
        signed   short format;
        unsigned short audio_object;
        unsigned short ep_config; /* 0 ~ 3 useful only obj = ERLC */
        unsigned short aac_section_data_resilience_flag;
        unsigned short aac_scalefactor_data_resilience_flag;
        unsigned short aac_spectral_data_resilience_flag;
        unsigned short sbr_on_flag;
        unsigned short sbr_ps_on_flag;
        unsigned short dual_mono_mode;
        unsigned short channel_configuration;
};
//#endif to be deleted till here

/*************************************************/
#if DONT_INCLUDE
struct msm_audio_config {
    uint32_t in_buf_count;
    uint32_t in_buf_size;   // output buffer size
    uint32_t channel_count;
    uint32_t sample_rate;
  uint8_t  pcm_feedback : 1;
  uint32_t buffer_size;    // input buffer size
  uint32_t buffer_count;
    uint32_t unused[4];
};
#endif

/*************************************************/
// omx_cmd_queue destructor
omx_aac_adec::omx_cmd_queue::~omx_cmd_queue()
{
  // Nothing to do
}

// omx cmd queue constructor
omx_aac_adec::omx_cmd_queue::omx_cmd_queue(): m_read(0),m_write(0),m_size(0)
{
  memset(m_q,      0,sizeof(omx_event)*OMX_CORE_CONTROL_CMDQ_SIZE);
}

// omx cmd queue insert
bool omx_aac_adec::omx_cmd_queue::insert_entry(unsigned p1, unsigned p2, unsigned id)
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
    DEBUG_PRINT_ERROR("xxERROR!!! Command Queue Full");
  }
  return ret;
}

// omx cmd queue delete
bool omx_aac_adec::omx_cmd_queue::pop_entry(unsigned *p1, unsigned *p2, unsigned *id)
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
    DEBUG_PRINT_ERROR("ERROR Delete!!! Command Queue Empty");
  }
  return ret;
}

// factory function executed by the core to create instances
void *get_omx_component_factory_fn(void)
{
  return (new omx_aac_adec);
}

void omx_aac_adec::wait_for_event()
{
    pthread_mutex_lock(&m_event_lock);
    while(m_is_event_done == 0)
    {
        pthread_cond_wait(&cond, &m_event_lock);
    }
    m_is_event_done = 0;
    pthread_mutex_unlock(&m_event_lock);
}

void omx_aac_adec::event_complete()
{
    pthread_mutex_lock(&m_event_lock);
    if(m_is_event_done == 0) {
        m_is_event_done = 1;
        pthread_cond_signal(&cond);
    }
    pthread_mutex_unlock(&m_event_lock);
}

// All this non-sense because of a single aac object
void omx_aac_adec::in_th_goto_sleep()
{
    pthread_mutex_lock(&m_in_th_lock);
    while (m_is_in_th_sleep == 0)
    {
        pthread_cond_wait(&in_cond, &m_in_th_lock);
    }
    m_is_in_th_sleep = 0;
    pthread_mutex_unlock(&m_in_th_lock);
}

void omx_aac_adec::in_th_wakeup()
{
    pthread_mutex_lock(&m_in_th_lock);
    if (m_is_in_th_sleep == 0) {
        m_is_in_th_sleep = 1;
        pthread_cond_signal(&in_cond);
    }
    pthread_mutex_unlock(&m_in_th_lock);
}

void omx_aac_adec::out_th_goto_sleep()
{

    pthread_mutex_lock(&m_out_th_lock);
    while (m_is_out_th_sleep == 0)
    {
        pthread_cond_wait(&out_cond, &m_out_th_lock);
    }
    m_is_out_th_sleep = 0;
    pthread_mutex_unlock(&m_out_th_lock);
}

void omx_aac_adec::out_th_wakeup()
{
    pthread_mutex_lock(&m_out_th_lock);
    if (m_is_out_th_sleep == 0) {
        m_is_out_th_sleep = 1;
        pthread_cond_signal(&out_cond);
    }
    pthread_mutex_unlock(&m_out_th_lock);
}

/* ======================================================================
FUNCTION
  omx_aac_adec::omx_aac_adec

DESCRIPTION
  Constructor

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
omx_aac_adec::omx_aac_adec(): m_flush_cnt(255),
                              m_state(OMX_StateInvalid),
                              m_app_data(NULL),
                              m_ipc_to_in_th(NULL),
                              m_ipc_to_out_th(NULL),
                              m_ipc_to_cmd_th(NULL),
                              m_drv_fd(-1),
                              m_inp_buf_count(0),
                              m_flags(0),
                              output_buffer_size(OMX_AAC_OUTPUT_BUFFER_SIZE),
                              input_buffer_size(OMX_CORE_INPUT_BUFFER_SIZE),
                              m_aac_hdr_bit_index(0),
                              m_is_event_done(0),
                              m_first_aac_header(0),
                              is_in_th_sleep(false),
                              is_out_th_sleep(false)
{
    memset(&m_cmp,       0,     sizeof(m_cmp));
    memset(&m_cb,        0,      sizeof(m_cb));

    pthread_mutexattr_init(&m_lock_attr);
    pthread_mutex_init(&m_lock, &m_lock_attr);

    pthread_mutexattr_init(&m_commandlock_attr);
    pthread_mutex_init(&m_commandlock, &m_commandlock_attr);

    pthread_mutexattr_init(&m_outputlock_attr);
    pthread_mutex_init(&m_outputlock, &m_outputlock_attr);

    pthread_mutexattr_init(&m_state_attr);
    pthread_mutex_init(&m_state_lock, &m_state_attr);

    pthread_mutexattr_init(&m_event_attr);
    pthread_mutex_init(&m_event_lock, &m_event_attr);

    pthread_mutexattr_init(&m_flush_attr);
    pthread_mutex_init(&m_flush_lock, &m_flush_attr);

    pthread_mutexattr_init(&m_event_attr);
    pthread_mutex_init(&m_event_lock, &m_event_attr);

    pthread_mutexattr_init(&m_in_th_attr);
    pthread_mutex_init(&m_in_th_lock, &m_in_th_attr);

    pthread_mutexattr_init(&m_out_th_attr);
    pthread_mutex_init(&m_out_th_lock, &m_out_th_attr);

    pthread_mutexattr_init(&m_in_th_attr_1);
    pthread_mutex_init(&m_in_th_lock_1, &m_in_th_attr_1);

    pthread_mutexattr_init(&m_out_th_attr_1);
    pthread_mutex_init(&m_out_th_lock_1, &m_out_th_attr_1);

    return;
}


/* ======================================================================
FUNCTION
  omx_aac_adec::~omx_aac_adec

DESCRIPTION
  Destructor

PARAMETERS
  None

RETURN VALUE
  None.
========================================================================== */
omx_aac_adec::~omx_aac_adec()
{
  pthread_mutexattr_destroy(&m_lock_attr);
  pthread_mutexattr_destroy(&m_commandlock_attr);
  pthread_mutexattr_destroy(&m_outputlock_attr);
  pthread_mutexattr_destroy(&m_state_attr);
  pthread_mutexattr_destroy(&m_flush_attr);
  pthread_mutexattr_destroy(&m_in_th_attr_1);
  pthread_mutexattr_destroy(&m_out_th_attr_1);
  pthread_mutexattr_destroy(&m_event_attr);
  pthread_mutexattr_destroy(&m_in_th_attr);
  pthread_mutexattr_destroy(&m_out_th_attr);

  pthread_mutex_destroy(&m_lock);
  pthread_mutex_destroy(&m_commandlock);
  pthread_mutex_destroy(&m_outputlock);
  return;
}

/**
  @brief memory function for sending EmptyBufferDone event
   back to IL client

  @param bufHdr OMX buffer header to be passed back to IL client
  @return none
 */
void omx_aac_adec::buffer_done_cb(OMX_BUFFERHEADERTYPE *bufHdr)
{
  if(m_cb.EmptyBufferDone)
  {
    PrintFrameHdr(bufHdr);

    m_cb.EmptyBufferDone(&m_cmp, m_app_data, bufHdr);
    nNumInputBuf--;
    DEBUG_PRINT("EBD:bufHdr[0x%x] NumInBuf[%d] TS[%ld]\n",\
                             bufHdr,nNumInputBuf,bufHdr->nTimeStamp);
  }

  return;
}

void omx_aac_adec::flush_ack()
{
    // Decrement the FLUSH ACK count and notify the waiting recepients
    pthread_mutex_lock(&m_flush_lock);
    --m_flush_cnt;
    if( m_flush_cnt == 0 )
    {
        event_complete();
    }
    DEBUG_PRINT("Rxed FLUSH ACK cnt=%d\n",m_flush_cnt);
    pthread_mutex_unlock(&m_flush_lock);
}


void omx_aac_adec::frame_done_cb(OMX_BUFFERHEADERTYPE *bufHdr)
{
  if(m_cb.FillBufferDone)
  {
    PrintFrameHdr(bufHdr);
    if (fbd_cnt == 0) {
        bufHdr->nTimeStamp = nTimestamp;
        DEBUG_PRINT("FBD:bufHdr[0x%x] TS-1[%d] Len[%d] NumOutBuf[%d]\n",\
                             bufHdr, bufHdr->nTimeStamp,bufHdr->nFilledLen,
                             (nNumOutputBuf-1));
    }
    else
    {
        nTimestamp += frameDuration;
        bufHdr->nTimeStamp = nTimestamp;
        DEBUG_PRINT("FBD:bufHdr[0x%x] TS[%d] Len[%d] NumOutBuf[%d]\n",\
                             bufHdr, bufHdr->nTimeStamp,bufHdr->nFilledLen,
                             (nNumOutputBuf-1));
    }
    fbd_cnt++;

    nNumOutputBuf--;
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

void omx_aac_adec::process_out_port_msg(void *client_data, unsigned char id)
{
    unsigned      p1; // Parameter - 1
    unsigned      p2; // Parameter - 2
    unsigned      ident;
    unsigned      qsize     = 0; // qsize
    unsigned      tot_qsize = 0;
    omx_aac_adec  *pThis    = (omx_aac_adec *) client_data;
    OMX_STATETYPE state;

    pthread_mutex_lock(&pThis->m_outputlock);

    pthread_mutex_lock(&pThis->m_state_lock);
    pThis->get_state(&pThis->m_cmp, &state);
    pthread_mutex_unlock(&pThis->m_state_lock);

    qsize = pThis->m_output_ctrl_cmd_q.m_size;

    tot_qsize = qsize;
    tot_qsize += pThis->m_output_ctrl_fbd_q.m_size;
    tot_qsize += pThis->m_output_q.m_size;

    if((!qsize && !pThis->bOutputPortReEnabled))
    {
        // case where no port reconfig and nothing in the flush q
        DEBUG_DETAIL("No flush/port reconfig qsize=%d tot_qsize=%d",qsize,tot_qsize);
        pthread_mutex_unlock(&pThis->m_outputlock);
        return;
    }

    DEBUG_DETAIL("OUT-->QSIZE-flush=%d,fbd=%d QSIZE=%d state=%d\n",\
                                      pThis->m_output_ctrl_cmd_q.m_size,
                                      pThis->m_output_ctrl_fbd_q.m_size,
                                      pThis->m_output_q.m_size,state);

    if(tot_qsize ==0) {
        pthread_mutex_unlock(&pThis->m_outputlock);
        DEBUG_DETAIL("OUT-->BREAK FROM LOOP...%d\n",tot_qsize);
        return;
    }

    if (qsize)
    {
        // process FLUSH message
        pThis->m_output_ctrl_cmd_q.pop_entry(&p1,&p2,&ident);
    }
    else if((qsize = pThis->m_output_ctrl_fbd_q.m_size) &&
            (pThis->bOutputPortReEnabled) && (state == OMX_StateExecuting))
    {
        // then process EBD's
        pThis->m_output_ctrl_fbd_q.pop_entry(&p1,&p2,&ident);
    }
    else if((qsize = pThis->m_output_q.m_size) &&
            (pThis->bOutputPortReEnabled) && (state == OMX_StateExecuting))
    {
        // if no FLUSH and FBD's then process FTB's
        pThis->m_output_q.pop_entry(&p1,&p2,&ident);
    }
    else
    {
        qsize = 0;
    }
    pthread_mutex_unlock(&pThis->m_outputlock);

    if(qsize > 0)
    {
        pThis->m_msg_cnt ++;

        id = ident;
        DEBUG_DETAIL("OUT->state[%d]ident[%d]flushq[%d]fbd[%d]dataq[%d]\n",\
                                  pThis->m_state,
                                  ident,
                                  pThis->m_output_ctrl_cmd_q.m_size,
                                  pThis->m_output_ctrl_fbd_q.m_size,
                                  pThis->m_output_q.m_size);

        if(id == OMX_COMPONENT_GENERATE_FRAME_DONE)
        {
                pThis->frame_done_cb((OMX_BUFFERHEADERTYPE *)p2);
        }
        else if(id == OMX_COMPONENT_GENERATE_FTB)
        {
                pThis->fill_this_buffer_proxy((OMX_HANDLETYPE)p1,
                                              (OMX_BUFFERHEADERTYPE *)p2);
        }
        else if(id == OMX_COMPONENT_GENERATE_EOS)
        {
                pThis->m_cb.EventHandler(&pThis->m_cmp,
                                          pThis->m_app_data,
                                          OMX_EventBufferFlag,
                                          1, 1, NULL );
        }
        else if(id == OMX_COMPONENT_GENERATE_COMMAND)
        {
            // Execute FLUSH command
            if(p1 == OMX_CommandFlush)
            {
                DEBUG_DETAIL("Executing FLUSH command on Output port\n");
                pThis->execute_output_omx_flush();
            }
            else
            {
                DEBUG_DETAIL("Invalid command[%d]\n",p1);
            }
        }
        else
        {
            DEBUG_PRINT_ERROR("ERROR:OUT-->Invalid Id[%d]\n",id);
        }
    }
    else
    {
        DEBUG_DETAIL("ERROR: OUT--> Empty OUTPUTQ\n");
    }
    pthread_mutex_lock(&pThis->m_state_lock);
    pThis->get_state(&pThis->m_cmp, &state);
    pthread_mutex_unlock(&pThis->m_state_lock);
    if(state != OMX_StateExecuting)
    {
        DEBUG_DETAIL("SLEEPING OUT THREAD\n");
        pthread_mutex_lock(&pThis->m_out_th_lock_1);
        pThis->is_out_th_sleep = true;
        pthread_mutex_unlock(&pThis->m_out_th_lock_1);
        pThis->out_th_goto_sleep();
    }
  return;
}


void omx_aac_adec::process_command_msg(void *client_data, unsigned char id)
{
    unsigned     p1;         // Parameter - 1
    unsigned     p2;         // Parameter - 2
    unsigned     ident;
    unsigned     qsize  = 0;
    omx_aac_adec *pThis = (omx_aac_adec*)client_data;

    pthread_mutex_lock(&pThis->m_commandlock);

    qsize = pThis->m_command_q.m_size;
    DEBUG_DETAIL("CMD-->QSIZE=%d state=%d\n",pThis->m_command_q.m_size,
                                             pThis->m_state);

    if(!qsize )
    {
        DEBUG_DETAIL("CMD-->BREAKING FROM LOOP\n");
        pthread_mutex_unlock(&pThis->m_commandlock);
        return;
    }
    else
    {
        pThis->m_command_q.pop_entry(&p1,&p2,&ident);
        pThis->m_msg_cnt ++;
    }
    pthread_mutex_unlock(&pThis->m_commandlock);

    id = ident;
    DEBUG_DETAIL("CMD->state[%d]id[%d]cmdq[%d]n",\
                                      pThis->m_state,ident, \
                                      pThis->m_command_q.m_size);

    if(id == OMX_COMPONENT_GENERATE_EVENT)
    {
        if (pThis->m_cb.EventHandler)
        {
            if (p1 == OMX_CommandStateSet)
            {
                pthread_mutex_lock(&pThis->m_state_lock);
                pThis->m_state = (OMX_STATETYPE) p2;
                DEBUG_PRINT("****** m_state set to %d ********\n",pThis->m_state);
                pthread_mutex_unlock(&pThis->m_state_lock);

                if(pThis->m_state == OMX_StateExecuting)
                {
                    pthread_mutex_lock(&pThis->m_in_th_lock_1);
                    if(pThis->is_in_th_sleep)
                    {
                        DEBUG_DETAIL("WAKING UP IN THREADS\n");
                        pThis->in_th_wakeup();
                        pThis->is_in_th_sleep = false;
                    }
                    pthread_mutex_unlock(&pThis->m_in_th_lock_1);

                    pthread_mutex_lock(&pThis->m_out_th_lock_1);
                    if(pThis->is_out_th_sleep)
                    {
                        DEBUG_DETAIL("WAKING UP OUT THREADS\n");
                        pThis->out_th_wakeup();
                        pThis->is_out_th_sleep = false;
                    }
                    pthread_mutex_unlock(&pThis->m_out_th_lock_1);
                }
                DEBUG_PRINT("Process->state set to %d \n", pThis->m_state);
            }
            if (pThis->m_state == OMX_StateInvalid)
            {
                pThis->m_cb.EventHandler(&pThis->m_cmp,
                                         pThis->m_app_data,
                                         OMX_EventError,
                                         OMX_ErrorInvalidState,
                                         0, NULL );
            }
            else
            {
                pThis->m_cb.EventHandler(&pThis->m_cmp,
                                         pThis->m_app_data,
                                         OMX_EventCmdComplete,
                                         p1, p2, NULL );
            }
        }
        else
        {
            DEBUG_PRINT_ERROR("ERROR:CMD-->EventHandler NULL \n");
        }
    }
    else if(id == OMX_COMPONENT_GENERATE_COMMAND)
    {
        pThis->send_command_proxy(&pThis->m_cmp,
                                  (OMX_COMMANDTYPE)p1,
                                  (OMX_U32)p2,(OMX_PTR)NULL);
    }
    else if(id == OMX_COMPONENT_PORTSETTINGS_CHANGED)
    {
        DEBUG_DETAIL("CMD-->RXED PORTSETTINGS_CHANGED");
        pThis->m_cb.EventHandler(&pThis->m_cmp,
                                 pThis->m_app_data,
                                 OMX_EventPortSettingsChanged,
                                 1, 1, NULL );
    }
    else
    {
        DEBUG_PRINT_ERROR("ERROR:CMD-->incorrect event[%d]\n",id);
    }
    return;
}

void omx_aac_adec::process_in_port_msg(void *client_data, unsigned char id)
{
    unsigned      p1;       // Parameter - 1
    unsigned      p2;       // Parameter - 2
    unsigned      ident;
    unsigned      qsize     = 0;
    unsigned      tot_qsize = 0;
    omx_aac_adec  *pThis    = (omx_aac_adec *) client_data;
    OMX_STATETYPE state;

    if(!pThis)
    {
        DEBUG_PRINT_ERROR("ERROR:IN--> Invalid Obj \n");
        return;
    }

    // Protect the shared queue data structure
    pthread_mutex_lock(&pThis->m_lock);

    pthread_mutex_lock(&pThis->m_state_lock);
    pThis->get_state(&pThis->m_cmp, &state);
    pthread_mutex_unlock(&pThis->m_state_lock);

    qsize = pThis->m_input_ctrl_cmd_q.m_size;
    tot_qsize = qsize;
    tot_qsize += pThis->m_input_ctrl_ebd_q.m_size;
    tot_qsize += pThis->m_input_q.m_size;

    DEBUG_DETAIL("Input-->QSIZE-flush=%d,ebd=%d QSIZE=%d state=%d\n",\
                                   pThis->m_input_ctrl_cmd_q.m_size,
                                   pThis->m_input_ctrl_ebd_q.m_size,
                                   pThis->m_input_q.m_size, state);


    if(tot_qsize ==0) {
        DEBUG_DETAIL("IN-->BREAKING FROM IN LOOP");
        pthread_mutex_unlock(&pThis->m_lock);
        return;
    }

    if(qsize)
    {
        // process FLUSH message
        pThis->m_input_ctrl_cmd_q.pop_entry(&p1,&p2,&ident);
    }
    else if((qsize = pThis->m_input_ctrl_ebd_q.m_size) &&
            (state == OMX_StateExecuting))
    {
        // then process EBD's
        pThis->m_input_ctrl_ebd_q.pop_entry(&p1,&p2,&ident);
    }
    else if((qsize = pThis->m_input_q.m_size) &&
            (state == OMX_StateExecuting))
    {
        // if no FLUSH and EBD's then process ETB's
        pThis->m_input_q.pop_entry(&p1, &p2, &ident);
    }
    else
    {
        qsize = 0;
    }
    pthread_mutex_unlock(&pThis->m_lock);

    if(qsize > 0)
    {
        pThis->m_msg_cnt ++;
        id = ident;
        DEBUG_DETAIL("Input->state[%d]id[%d]flushq[%d]ebdq[%d]dataq[%d]\n",\
                                             pThis->m_state,
                                             ident,
                                             pThis->m_input_ctrl_cmd_q.m_size,
                                             pThis->m_input_ctrl_ebd_q.m_size,
                                             pThis->m_input_q.m_size);
        if(id == OMX_COMPONENT_GENERATE_BUFFER_DONE)
        {
            pThis->buffer_done_cb((OMX_BUFFERHEADERTYPE *)p2);
        }
        else if(id == OMX_COMPONENT_GENERATE_ETB)
        {
            pThis->empty_this_buffer_proxy((OMX_HANDLETYPE)p1,
                                           (OMX_BUFFERHEADERTYPE *)p2);
        }
        else if(id == OMX_COMPONENT_GENERATE_COMMAND)
        {
            // Execute FLUSH command
            if(p1 == OMX_CommandFlush)
            {
                DEBUG_DETAIL(" Executing FLUSH command on Input port\n");
                pThis->execute_input_omx_flush();
            }
            else
            {
                DEBUG_DETAIL("Invalid command[%d]\n",p1);
            }
        }
        else
        {
            DEBUG_PRINT_ERROR("ERROR:IN-->Invalid Id[%d]\n",id);
        }
    }
    else
    {
        DEBUG_DETAIL("ERROR:IN-->Empty INPUT Q\n");
    }
    pthread_mutex_lock(&pThis->m_state_lock);
    pThis->get_state(&pThis->m_cmp, &state);
    pthread_mutex_unlock(&pThis->m_state_lock);
    if(state != OMX_StateExecuting)
    {
        DEBUG_DETAIL("SLEEPING IN THREAD\n");
        pthread_mutex_lock(&pThis->m_in_th_lock_1);
        pThis->is_in_th_sleep = true;
        pthread_mutex_unlock(&pThis->m_in_th_lock_1);
        pThis->in_th_goto_sleep();
    }
    return;
}

/**
 @brief member function for performing component initialization

 @param role C string mandating role of this component
 @return Error status
 */
OMX_ERRORTYPE omx_aac_adec::component_init(OMX_STRING role)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;

  /* Ignore role */
  DEBUG_PRINT("******* setting to OMX_StateLoaded *******\n");
  m_state                   = OMX_StateLoaded;

  /* DSP does not give information about the bitstream
     randomly assign the value right now. Query will result in
     incorrect param */
  memset(&m_adec_param, 0, sizeof(m_adec_param));
  m_adec_param.nSize = sizeof(m_adec_param);
  m_adec_param.nSampleRate = 0; //48000;
  m_volume = 25; /* Close to unity gain */
  m_adec_param.nChannels = 2;
  m_adec_param.nFrameLength = 1024; /*Setting the nFrameLength to default*/

  /* default calculation of frame duration */
  //frameDuration = (((9216)* 1000) / (44100 * 4));
  frameDuration = 0;
  fbd_cnt = 0;
  nTimestamp = 0;
  DEBUG_PRINT(" Enabling Non-Tunnel mode \n");
  pcm_feedback = 1;    /* by default enable non-tunnel mode */
  ntotal_playtime = 0;

  m_first_aac_header=0;
  nNumInputBuf = 0;
  nNumOutputBuf = 0;
  bEOSSent = 0;
  m_inp_buf_count = 0; // making number of input buffer count to 0
  m_out_buf_count = 0; // making number of output buffer count to 0
  m_ipc_to_in_th = NULL;  // Command server instance
  m_ipc_to_out_th = NULL;  // Client server instance
  m_ipc_to_cmd_th = NULL;  // command instance
  bOutputPortReEnabled = 0;

  DEBUG_PRINT(" component init: role = %s\n",role);

  if(!strcmp(role,"OMX.qcom.audio.decoder.aac"))
  {
      pcm_feedback = 1;
      DEBUG_PRINT("\ncomponent_init: Component %s LOADED \n", role);
  }
  else if(!strcmp(role,"OMX.qcom.audio.decoder.tunneled.aac"))
  {
      pcm_feedback = 0;
      DEBUG_PRINT("\ncomponent_init: Component %s LOADED \n", role);
  }
  else
  {
      DEBUG_PRINT("\ncomponent_init: Component %s LOADED is invalid\n", role);
  }

  if(!m_ipc_to_in_th)
  {
    m_ipc_to_in_th = omx_aac_thread_create(process_in_port_msg, this,"INPUT_THREAD");
    if(!m_ipc_to_in_th)
    {
      DEBUG_PRINT_ERROR("ERROR!!! Failed to start Input port thread\n");
      return OMX_ErrorInsufficientResources;
    }

  }

  if(!m_ipc_to_cmd_th)
  {
    m_ipc_to_cmd_th = omx_aac_thread_create(process_command_msg, this,"CMD_THREAD");
    if(!m_ipc_to_cmd_th)
    {
      DEBUG_PRINT_ERROR("ERROR!!!Failed to start command message thread\n");
      return OMX_ErrorInsufficientResources;
    }

  }

  if(pcm_feedback)
  {
    if(!m_ipc_to_out_th)
    {
      m_ipc_to_out_th = omx_aac_thread_create(process_out_port_msg, this,"OUTPUT_THREAD");
      if(!m_ipc_to_out_th)
      {
        DEBUG_PRINT_ERROR("ERROR!!! Failed to start output port thread\n");
        return OMX_ErrorInsufficientResources;
      }
    }
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
OMX_ERRORTYPE  omx_aac_adec::get_component_version(OMX_IN OMX_HANDLETYPE               hComp,
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
OMX_ERRORTYPE  omx_aac_adec::send_command(OMX_IN OMX_HANDLETYPE hComp,
                                          OMX_IN OMX_COMMANDTYPE  cmd,
                                          OMX_IN OMX_U32       param1,
                                          OMX_IN OMX_PTR      cmdData)
{
  post_command((unsigned)cmd,(unsigned)param1,OMX_COMPONENT_GENERATE_COMMAND);

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
OMX_ERRORTYPE  omx_aac_adec::send_command_proxy(OMX_IN OMX_HANDLETYPE hComp,
                                          OMX_IN OMX_COMMANDTYPE  cmd,
                                          OMX_IN OMX_U32       param1,
                                          OMX_IN OMX_PTR      cmdData)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;
    OMX_BUFFERHEADERTYPE* buffer;

    OMX_STATETYPE eState = (OMX_STATETYPE) param1;
    int bFlag = 1;

    if(cmd == OMX_CommandStateSet)
    {
        DEBUG_PRINT("omx_aac_adec::send_command_proxy(): OMX_CommandStateSet current %d ==> %d\n",
                                        m_state,eState);
        /***************************/
        /* Current State is Loaded */
        /***************************/
        if(m_state == OMX_StateLoaded)
        {
            if(eState == OMX_StateIdle)
            {
                m_drv_fd = open("/dev/msm_aac", O_RDWR);

                if (m_drv_fd < 0)
                {
                    DEBUG_PRINT_ERROR("SCP-->Dev Open Failed[%d]",m_drv_fd);
                    eState = OMX_StateInvalid;
                }
                else
                {
                    if (allocate_done())
                    {
                        DEBUG_PRINT("SCP-->Loaded->Idle\n");
                    }
                    else
                    {
                        DEBUG_PRINT("SCP-->Loaded to Idle-Pending\n");
                        BITMASK_SET(&m_flags, OMX_COMPONENT_IDLE_PENDING);
                        bFlag = 0;
                    }
                }
            }
            else
            {
                DEBUG_PRINT_ERROR("SCP-->Loaded to Invalid(%d))\n",eState);
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

                    ioctl(m_drv_fd, AUDIO_STOP, 0);
                    close(m_drv_fd);
                    m_drv_fd = -1;
                    DEBUG_PRINT("SCP-->Idle to Loaded\n");
                }
                else
                {
                    DEBUG_PRINT("SCP--> Idle to Loaded-Pending\n");
                    BITMASK_SET(&m_flags, OMX_COMPONENT_LOADING_PENDING);
                    // Skip the event notification
                    bFlag = 0;
                }
            }
            else if(eState == OMX_StateExecuting)
            {
                struct msm_audio_pcm_config  pcm_config;
                DEBUG_PRINT("SCP-->Driver mode  %d \n",pcm_feedback);
                ioctl(m_drv_fd, AUDIO_GET_PCM_CONFIG, &pcm_config);
                pcm_config.pcm_feedback = pcm_feedback;
                ioctl(m_drv_fd, AUDIO_SET_PCM_CONFIG, &pcm_config);
                DEBUG_PRINT("SCP-->Idle to Executing\n");
            }
            else
            {
                DEBUG_PRINT_ERROR("SCP--> Idle to %d Not Handled\n",eState);
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
                DEBUG_PRINT("SCP-->Executing to Idle \n");
                execute_omx_flush(-1,false); // Flush all ports
            }
            else if(eState == OMX_StatePause)
            {
                DEBUG_DETAIL("*************************\n");
                DEBUG_PRINT("SCP-->RXED PAUSE STATE\n");
                DEBUG_DETAIL("*************************\n");
            }
            else
            {
                DEBUG_PRINT_ERROR("SCP--> Executing to %d Not Handled\n",eState);
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
                DEBUG_PRINT("SCP-->Paused to Executing \n");
                if (1 == bEOSSent)
                {
                  endofstream = 0;
                  bEOSSent = 0;
                  DEBUG_PRINT("SCP--> Resetting the EOS flags endofstream");
                }
            }
            else if(eState == OMX_StateIdle)
            {
                DEBUG_PRINT("SCP-->Paused to Idle \n");
                pthread_mutex_lock(&m_flush_lock);
                m_flush_cnt = 2;
                pthread_mutex_unlock(&m_flush_lock);

                execute_omx_flush(-1,false); // Flush all ports
            }
            else
            {
                DEBUG_PRINT("SCP-->Paused to %d Not Handled\n",eState);
                eRet = OMX_ErrorBadParameter;
            }
        }
        else
        {
            DEBUG_PRINT_ERROR("SCP--> %d to %d(Not Handled)\n",m_state,eState);
            eRet = OMX_ErrorBadParameter;
        }
    }
    else if (cmd == OMX_CommandFlush)
    {
        DEBUG_DETAIL("*************************\n");
        DEBUG_PRINT("SCP-->RXED FLUSH COMMAND port=%d\n",param1);
        DEBUG_DETAIL("*************************\n");

        execute_omx_flush(param1);
        bFlag = 0;
    }
    else if (cmd == OMX_CommandPortDisable)
    {
        if(param1 == OMX_CORE_OUTPUT_PORT_INDEX)
        {
            pcm_feedback = 0;    /* enable tunnel mode */
            DEBUG_PRINT("SCP-->Disabling Tunnel mode \n");

            m_cb.EventHandler(&m_cmp,
                               m_app_data,
                               OMX_EventCmdComplete,
                               OMX_CommandPortDisable,
                               OMX_CORE_OUTPUT_PORT_INDEX, NULL );
        }
        else
        {
            DEBUG_PRINT_ERROR("SCP-->Disabling invalid port ID[%d]",param1);
        }
        bFlag = 0;
    }
    else if (cmd == OMX_CommandPortEnable)
    {
        if (param1 == OMX_CORE_OUTPUT_PORT_INDEX)
        {
            pcm_feedback = 1;    /* enable non-tunnel mode */
            DEBUG_PRINT("SCP-->Enabling Non-Tunnel mode \n");
            m_cb.EventHandler(&m_cmp,
                               m_app_data,
                               OMX_EventCmdComplete,
                               OMX_CommandPortEnable,
                               OMX_CORE_OUTPUT_PORT_INDEX, NULL );
            bOutputPortReEnabled = 1;
        }
        else
        {
            DEBUG_PRINT_ERROR("SCP-->Enabling invalid port ID[%d]",param1);
        }
        bFlag = 0;
    }
    else
    {
        DEBUG_PRINT_ERROR("SCP-->ERROR: Invali Command [%d]\n",cmd);
        eRet = OMX_ErrorNotImplemented;
    }
    if(eRet == OMX_ErrorNone && bFlag)
    {
        post_command(cmd,eState,OMX_COMPONENT_GENERATE_EVENT);
    }
  return eRet;
}

/**
 @brief member function that flushes buffers that are pending to be written
  to driver

 @param none
 @return bool value indicating whether flushing is carried out successfully
*/
bool omx_aac_adec::execute_omx_flush(OMX_IN OMX_U32 param1, bool cmd_cmpl)
{
    bool bRet = true;
    OMX_BUFFERHEADERTYPE *omx_buf;
    unsigned      p1; // Parameter - 1
    unsigned      p2; // Parameter - 2
    unsigned      ident;
    bool          bqueStatus = 0;

    DEBUG_PRINT("Execute_omx_flush Port[%d]", param1);

    if (param1 == -1)
    {
        DEBUG_PRINT("Execute flush for both I/p O/p port\n");
        pthread_mutex_lock(&m_flush_lock);
        m_flush_cnt = 2;
        pthread_mutex_unlock(&m_flush_lock);

        // Send Flush commands to input and output threads
        post_input(OMX_CommandFlush,
                  OMX_CORE_INPUT_PORT_INDEX,OMX_COMPONENT_GENERATE_COMMAND);
        post_output(OMX_CommandFlush,
                  OMX_CORE_OUTPUT_PORT_INDEX,OMX_COMPONENT_GENERATE_COMMAND);
        // Send Flush to the kernel so that the in and out buffers are released
        ioctl( m_drv_fd, AUDIO_FLUSH, 0);
        DEBUG_PRINT("AUDIO_FLUSH = 0x%08X\n",AUDIO_FLUSH);

        DEBUG_DETAIL("****************************************");
        DEBUG_DETAIL("is_in_th_sleep=%d is_out_th_sleep=%d\n",is_in_th_sleep,is_out_th_sleep);
        DEBUG_DETAIL("****************************************");
        pthread_mutex_lock(&m_in_th_lock_1);
        if(is_in_th_sleep)
        {
            DEBUG_DETAIL("For FLUSH-->WAKING UP IN THREADS\n");
            in_th_wakeup();
            is_in_th_sleep = false;
        }
        pthread_mutex_unlock(&m_in_th_lock_1);

        pthread_mutex_lock(&m_out_th_lock_1);
        if(is_out_th_sleep)
        {
            DEBUG_DETAIL("For FLUSH-->WAKING UP OUT THREADS\n");
            out_th_wakeup();
            is_out_th_sleep = false;
        }
        pthread_mutex_unlock(&m_out_th_lock_1);

        // sleep till the FLUSH ACK are done by both the input and output threads
        DEBUG_DETAIL("WAITING FOR FLUSH ACK's param1=%d",param1);
        wait_for_event();

        DEBUG_DETAIL("RECIEVED BOTH FLUSH ACK's param1=%d",param1);

        // If not going to idle state, Send FLUSH complete message to the Client,
        // now that FLUSH ACK's have been recieved.
        if(cmd_cmpl)
        {
            m_cb.EventHandler(&m_cmp, m_app_data, OMX_EventCmdComplete,
                          OMX_CommandFlush, OMX_CORE_INPUT_PORT_INDEX, NULL );
            m_cb.EventHandler(&m_cmp, m_app_data, OMX_EventCmdComplete,
                          OMX_CommandFlush, OMX_CORE_OUTPUT_PORT_INDEX, NULL );
        }
    }
    else if (param1 == OMX_CORE_INPUT_PORT_INDEX)
    {
        DEBUG_PRINT("Execute FLUSH for I/p port\n");
        pthread_mutex_lock(&m_flush_lock);
        m_flush_cnt = 1;
        pthread_mutex_unlock(&m_flush_lock);
        post_input(OMX_CommandFlush,
                  OMX_CORE_INPUT_PORT_INDEX,OMX_COMPONENT_GENERATE_COMMAND);
        // Send Flush to the kernel so that the in and out buffers are released
        // sleep till the FLUSH ACK are done by both the input and output threads
        DEBUG_DETAIL("Executing FLUSH for I/p port\n");
        DEBUG_DETAIL("WAITING FOR FLUSH ACK's param1=%d",param1);
        wait_for_event();
        DEBUG_DETAIL(" RECIEVED FLUSH ACK FOR I/P PORT param1=%d",param1);

        // Send FLUSH complete message to the Client,
        // now that FLUSH ACK's have been recieved.
        m_cb.EventHandler(&m_cmp, m_app_data, OMX_EventCmdComplete,
                          OMX_CommandFlush, OMX_CORE_INPUT_PORT_INDEX, NULL );
    }
    else if (param1 == OMX_CORE_OUTPUT_PORT_INDEX)
    {
        DEBUG_PRINT("Executing FLUSH for O/p port\n");
        pthread_mutex_lock(&m_flush_lock);
        m_flush_cnt = 1;
        pthread_mutex_unlock(&m_flush_lock);
        DEBUG_DETAIL("Executing FLUSH for O/p port\n");
        DEBUG_DETAIL("WAITING FOR FLUSH ACK's param1=%d",param1);

        post_output(OMX_CommandFlush,
                    OMX_CORE_OUTPUT_PORT_INDEX,OMX_COMPONENT_GENERATE_COMMAND);
        // sleep till the FLUSH ACK are done by both the input and output threads
        wait_for_event();
        // Send FLUSH complete message to the Client,
        // now that FLUSH ACK's have been recieved.
        m_cb.EventHandler(&m_cmp, m_app_data, OMX_EventCmdComplete,
                          OMX_CommandFlush, OMX_CORE_OUTPUT_PORT_INDEX, NULL );

        DEBUG_DETAIL("RECIEVED FLUSH ACK FOR O/P PORT param1=%d",param1);
    }
    else
    {
      DEBUG_PRINT("Invalid Port ID[%d]",param1);
    }
    return bRet;
}

/**
 @brief member function that flushes buffers that are pending to be written
  to driver

 @param none
 @return bool value indicating whether flushing is carried out successfully
*/
bool omx_aac_adec::execute_input_omx_flush()
{
    OMX_BUFFERHEADERTYPE *omx_buf;
    unsigned      p1; // Parameter - 1
    unsigned      p2; // Parameter - 2
    unsigned      ident;
    unsigned      qsize=0; // qsize
    unsigned      tot_qsize=0; // qsize

    DEBUG_PRINT("Execute_omx_flush on input port");

    pthread_mutex_lock(&m_lock);
    do
    {
        qsize = m_input_q.m_size;
        tot_qsize = qsize;
        tot_qsize += m_input_ctrl_ebd_q.m_size;

        DEBUG_DETAIL("Input FLUSH-->flushq[%d] ebd[%d]dataq[%d]",
                     m_input_ctrl_cmd_q.m_size,m_input_ctrl_ebd_q.m_size,qsize);
        if(!tot_qsize)
        {
            DEBUG_DETAIL("Input-->BREAKING FROM execute_input_flush LOOP");
            pthread_mutex_unlock(&m_lock);
            break;
        }
        if (qsize)
        {
            m_input_q.pop_entry(&p1, &p2, &ident);
            if ((ident == OMX_COMPONENT_GENERATE_ETB) ||
                (ident == OMX_COMPONENT_GENERATE_BUFFER_DONE))
            {
                omx_buf = (OMX_BUFFERHEADERTYPE *) p2;
                DEBUG_DETAIL("Input Buf_Addr=%x \n", omx_buf);
                buffer_done_cb((OMX_BUFFERHEADERTYPE *)omx_buf);
            }
        }
        else if((qsize = m_input_ctrl_ebd_q.m_size))
        {
            m_input_ctrl_ebd_q.pop_entry(&p1, &p2, &ident);
            if(ident == OMX_COMPONENT_GENERATE_BUFFER_DONE)
            {
                buffer_done_cb((OMX_BUFFERHEADERTYPE *)omx_buf);
            }
        }
        else{}
    }while(qsize>0);
    DEBUG_DETAIL("*************************\n");
    DEBUG_DETAIL("IN-->CALLING FLUSH EH\n");
    DEBUG_DETAIL("*************************\n");
    flush_ack();
    pthread_mutex_unlock(&m_lock);
  return true;
}

/**
 @brief member function that flushes buffers that are pending to be written
  to driver

 @param none
 @return bool value indicating whether flushing is carried out successfully
*/
bool omx_aac_adec::execute_output_omx_flush()
{
    OMX_BUFFERHEADERTYPE *omx_buf;
    unsigned      p1; // Parameter - 1
    unsigned      p2; // Parameter - 2
    unsigned      ident;
    unsigned      qsize=0; // qsize
    unsigned      tot_qsize=0; // qsize

    DEBUG_PRINT("Execute_omx_flush on output port");

    pthread_mutex_lock(&m_outputlock);
    do
    {
        qsize = m_output_q.m_size;
        DEBUG_DETAIL("OUT FLUSH-->flushq[%d] fbd[%d]dataq[%d]",\
                                                m_output_ctrl_cmd_q.m_size,
                                                m_output_ctrl_fbd_q.m_size,qsize);
        tot_qsize = qsize;
        tot_qsize += m_output_ctrl_fbd_q.m_size;
        if(!tot_qsize)
        {
            DEBUG_DETAIL("OUT-->BREAKING FROM execute_input_flush LOOP");
            pthread_mutex_unlock(&m_outputlock);
            break;
        }
        if (qsize)
        {
            m_output_q.pop_entry(&p1,&p2,&ident);
            if ( (ident == OMX_COMPONENT_GENERATE_FTB) ||
                 (ident == OMX_COMPONENT_GENERATE_FRAME_DONE))
            {
                omx_buf = (OMX_BUFFERHEADERTYPE *) p2;
                DEBUG_DETAIL("Ouput Buf_Addr=%x \n", omx_buf);
                omx_buf->nFilledLen = 0;
                frame_done_cb((OMX_BUFFERHEADERTYPE *)omx_buf);
                DEBUG_DETAIL("CALLING FBD FROM FLUSH");
            }
        }
        else if((qsize = m_output_ctrl_fbd_q.m_size))
        {
            m_output_ctrl_fbd_q.pop_entry(&p1, &p2, &ident);
            if(ident == OMX_COMPONENT_GENERATE_FRAME_DONE)
            {
                omx_buf->nFilledLen = 0;
                frame_done_cb((OMX_BUFFERHEADERTYPE *)omx_buf);
                DEBUG_DETAIL("CALLING FROM CTRL-FBDQ FROM FLUSH");
            }
        }
    }while(qsize>0);
    DEBUG_DETAIL("*************************\n");
    DEBUG_DETAIL("OUT-->BEFORE CALLING FLUSH ACK\n");
    DEBUG_DETAIL("*************************\n");
    flush_ack();
    pthread_mutex_unlock(&m_outputlock);

  return true;
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
bool omx_aac_adec::post_input(unsigned int p1,
                              unsigned int p2,
                              unsigned int id,
                              bool lock)
{
  bool bRet = false;
  unsigned prev_qsize=0;
  unsigned qsize=0;
  if (lock == true)
  {
    pthread_mutex_lock(&m_lock);
  }

  m_cmd_cnt ++;

  prev_qsize = m_input_ctrl_cmd_q.m_size;
  prev_qsize += m_input_ctrl_ebd_q.m_size;
  prev_qsize += m_input_q.m_size;

  if((OMX_COMPONENT_GENERATE_COMMAND == id))
  {
      // insert flush message and ebd
      m_input_ctrl_cmd_q.insert_entry(p1,p2,id);
  }
  else if((OMX_COMPONENT_GENERATE_BUFFER_DONE == id))
  {
      // insert ebd
      m_input_ctrl_ebd_q.insert_entry(p1,p2,id);
  }
  else
  {
      // ETBS in this queue
      m_input_q.insert_entry(p1,p2,id);
  }
  qsize = m_input_ctrl_cmd_q.m_size;
  qsize += m_input_ctrl_ebd_q.m_size;
  qsize += m_input_q.m_size;

  if(m_ipc_to_in_th)
  {
    bRet = true;
    omx_aac_post_msg(m_ipc_to_in_th, id);
  }

  DEBUG_DETAIL("PostInput-->state[%d]id[%d]flushq[%d]ebdq[%d]dataq[%d] \n",\
                                                       m_state,
                                                       id,
                                                       m_input_ctrl_cmd_q.m_size,
                                                       m_input_ctrl_ebd_q.m_size,
                                                       m_input_q.m_size);

  if (lock == true)
  {
    pthread_mutex_unlock(&m_lock);
  }

  return bRet;
}


bool omx_aac_adec::post_command(unsigned int p1,
                              unsigned int p2,
                              unsigned int id,
                              bool lock)
{
  bool bRet  = false;

  if (lock == true)
  {
    pthread_mutex_lock(&m_commandlock);
  }

  m_cmd_cnt ++;

  m_command_q.insert_entry(p1,p2,id);


  if(m_ipc_to_cmd_th)
  {
    bRet = true;
    omx_aac_post_msg(m_ipc_to_cmd_th, id);
  }

  DEBUG_DETAIL("PostCmd-->state[%d]id[%d]cmdq[%d]flags[%x]\n",\
                                                            m_state,
                                                            id,
                                                            m_command_q.m_size,
                                                            m_flags >> 3);

  if (lock == true) {
    pthread_mutex_unlock(&m_commandlock);
  }

  return bRet;
}



bool omx_aac_adec::post_output(unsigned int p1,
                              unsigned int p2,
                              unsigned int id,
                              bool lock)
{
  bool bRet = false;
  unsigned qsize = 0;
  unsigned prev_qsize = 0;
  if (lock == true)
  {
    pthread_mutex_lock(&m_outputlock);
  }

  m_cmd_cnt ++;

  prev_qsize = m_output_ctrl_cmd_q.m_size;
  prev_qsize += m_output_ctrl_fbd_q.m_size;
  prev_qsize += m_output_q.m_size;

  if((OMX_COMPONENT_GENERATE_COMMAND == id) )
  {
      // insert flush message and fbd
      m_output_ctrl_cmd_q.insert_entry(p1,p2,id);
  }
  else if((OMX_COMPONENT_GENERATE_FRAME_DONE == id) )
  {
      // insert flush message and fbd
      m_output_ctrl_fbd_q.insert_entry(p1,p2,id);
  }
  else
  {
      m_output_q.insert_entry(p1,p2,id);
  }
  qsize = m_output_ctrl_cmd_q.m_size;
  qsize += m_output_ctrl_fbd_q.m_size;
  qsize += m_output_q.m_size;


  if(m_ipc_to_out_th)
  {
    bRet = true;
    omx_aac_post_msg(m_ipc_to_out_th, id);
  }
  DEBUG_DETAIL("PostOutput-->state[%d]id[%d]flushq[%d]ebdq[%d]dataq[%d]\n",\
                                                        m_state,
                                                        id,
                                                        m_output_ctrl_cmd_q.m_size,
                                                        m_output_ctrl_fbd_q.m_size,
                                                        m_output_q.m_size);

  if (lock == true) {
    pthread_mutex_unlock(&m_outputlock);
  }

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
OMX_ERRORTYPE  omx_aac_adec::get_parameter(OMX_IN OMX_HANDLETYPE     hComp,
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
        portDefn->format.audio.eEncoding = OMX_AUDIO_CodingAAC;
        portDefn->format.audio.pNativeRender = 0;


      }
      else if (1 == portDefn->nPortIndex)
      {
        portDefn->eDir =  OMX_DirOutput;
        portDefn->nBufferCountActual = 2; /* What if the component does not restrict how many buffer to take */
        portDefn->nBufferCountMin    = 2;
        portDefn->nBufferSize        = output_buffer_size;
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
          portFormatType->eEncoding = OMX_AUDIO_CodingAAC;
        }
      } else {
        eRet = OMX_ErrorBadPortIndex;
      }
      break;
    }

    case OMX_IndexParamAudioAac:
    {
      OMX_AUDIO_PARAM_AACPROFILETYPE *aacParam = (OMX_AUDIO_PARAM_AACPROFILETYPE *) paramData;
      DEBUG_PRINT("OMX_IndexParamAudioAac\n");
      *aacParam = m_adec_param;

      break;
    }

    case OMX_IndexParamAudioPcm:
    {
      OMX_AUDIO_PARAM_PCMMODETYPE *pcmparam = (OMX_AUDIO_PARAM_PCMMODETYPE *) paramData;

      pcmparam->nSamplingRate = m_adec_param.nSampleRate;
      pcmparam->nChannels = m_adec_param.nChannels;
      DEBUG_PRINT("get_parameter: Sampling rate %d", pcmparam->nSamplingRate);
      DEBUG_PRINT("get_parameter: Number of channels %d", pcmparam->nChannels);

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
OMX_ERRORTYPE  omx_aac_adec::set_parameter(OMX_IN OMX_HANDLETYPE     hComp,
                                           OMX_IN OMX_INDEXTYPE paramIndex,
                                           OMX_IN OMX_PTR        paramData)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  int           i;
  struct msm_audio_config drv_config;
  struct msm_audio_pcm_config  pcm_config;

  switch(paramIndex)
  {

    case OMX_IndexParamAudioAac:
    {
      DEBUG_PRINT("OMX_IndexParamAudioAAC");
      m_adec_param = *((OMX_AUDIO_PARAM_AACPROFILETYPE *) paramData);

      pSamplerate = m_adec_param.nSampleRate;
      pChannels = m_adec_param.nChannels;
      pBitrate = m_adec_param.nBitRate;

      if(m_adec_param.nChannels == 1)
        {
            frameDuration = (((OMX_AAC_OUTPUT_BUFFER_SIZE)* 1000) /(m_adec_param.nSampleRate * 2));
            DEBUG_PRINT("frame duration of mono config = %d sampling rate = %d \n",
                         frameDuration,m_adec_param.nSampleRate);
        }
        else if(m_adec_param.nChannels == 2)
        {
            frameDuration = (((OMX_AAC_OUTPUT_BUFFER_SIZE)* 1000) /(m_adec_param.nSampleRate  * 4));
            DEBUG_PRINT("frame duration of stero config = %d sampling rate = %d \n",
                         frameDuration,m_adec_param.nSampleRate);

        }
      break;
    }
    case OMX_IndexParamPortDefinition:
    {
      OMX_PARAM_PORTDEFINITIONTYPE *portDefn;
      portDefn = (OMX_PARAM_PORTDEFINITIONTYPE *) paramData;

      DEBUG_PRINT("OMX_IndexParamPortDefinition portDefn->nPortIndex = %d\n",portDefn->nPortIndex);
      if (0 == portDefn->nPortIndex)
      {

        DEBUG_PRINT("SET_PARAMETER: configuring input port \n");
        ioctl(m_drv_fd, AUDIO_GET_CONFIG, &drv_config);
        drv_config.buffer_size = portDefn->nBufferSize;
        drv_config.buffer_count = portDefn->nBufferCountActual;
        input_buffer_size = drv_config.buffer_size;
        ioctl(m_drv_fd, AUDIO_SET_CONFIG, &drv_config);

      }
      else if (1 == portDefn->nPortIndex)
      {

        DEBUG_PRINT("SET_PARAMETER: configuring output port \n");
        ioctl(m_drv_fd, AUDIO_GET_PCM_CONFIG, &pcm_config);
        pcm_config.buffer_count = portDefn->nBufferCountActual;
        pcm_config.buffer_size  = portDefn->nBufferSize;
        output_buffer_size = pcm_config.buffer_size;
        ioctl(m_drv_fd, AUDIO_SET_PCM_CONFIG, &pcm_config);

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
  omx_aac_adec::GetConfig

DESCRIPTION
  OMX Get Config Method implementation.

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if successful.

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::get_config(OMX_IN OMX_HANDLETYPE      hComp,
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
  omx_aac_adec::SetConfig

DESCRIPTION
  OMX Set Config method implementation

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if successful.
========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::set_config(OMX_IN OMX_HANDLETYPE      hComp,
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
  omx_aac_adec::GetExtensionIndex

DESCRIPTION
  OMX GetExtensionIndex method implementaion.  <TBD>

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::get_extension_index(OMX_IN OMX_HANDLETYPE      hComp,
                                                OMX_IN OMX_STRING      paramName,
                                                OMX_OUT OMX_INDEXTYPE* indexType)
{
  DEBUG_PRINT_ERROR("Error, Not implemented\n");
  return OMX_ErrorNotImplemented;
}

/* ======================================================================
FUNCTION
  omx_aac_adec::GetState

DESCRIPTION
  Returns the state information back to the caller.<TBD>

PARAMETERS
  <TBD>.

RETURN VALUE
  Error None if everything is successful.
========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::get_state(OMX_IN OMX_HANDLETYPE  hComp,
                                       OMX_OUT OMX_STATETYPE* state)
{
    DEBUG_PRINT("omx_aac_adec::get_state()\n");
    *state = m_state;
    DEBUG_PRINT("omx_aac_adec::get_state() 2\n");
    return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  omx_aac_adec::ComponentTunnelRequest

DESCRIPTION
  OMX Component Tunnel Request method implementation. <TBD>

PARAMETERS
  None.

RETURN VALUE
  OMX Error None if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::component_tunnel_request(OMX_IN OMX_HANDLETYPE                hComp,
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
  omx_aac_adec::AllocateInputBuffer

DESCRIPTION
  Helper function for allocate buffer in the input pin

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::allocate_input_buffer(OMX_IN OMX_HANDLETYPE                hComp,
                                                  OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                                                  OMX_IN OMX_U32                        port,
                                                  OMX_IN OMX_PTR                     appData,
                                                  OMX_IN OMX_U32                       bytes)
{
  OMX_ERRORTYPE         eRet = OMX_ErrorNone;
  OMX_BUFFERHEADERTYPE            *bufHdr;
  unsigned                   nBufSize = MAX(bytes, input_buffer_size);
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

OMX_ERRORTYPE  omx_aac_adec::allocate_output_buffer(OMX_IN OMX_HANDLETYPE                hComp,
                                                  OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                                                  OMX_IN OMX_U32                        port,
                                                  OMX_IN OMX_PTR                     appData,
                                                  OMX_IN OMX_U32                       bytes)
{
  OMX_ERRORTYPE         eRet = OMX_ErrorNone;
  OMX_BUFFERHEADERTYPE            *bufHdr;
  unsigned                   nBufSize = MAX(bytes,output_buffer_size);
  char                       *buf_ptr;

  buf_ptr = (char *) calloc( (nBufSize + sizeof(OMX_BUFFERHEADERTYPE) ) , 1);

  if (buf_ptr != NULL) {
    bufHdr = (OMX_BUFFERHEADERTYPE *) buf_ptr;
    *bufferHdr = bufHdr;
    memset(bufHdr,0,sizeof(OMX_BUFFERHEADERTYPE));

    bufHdr->pBuffer           = (OMX_U8 *)((buf_ptr) +
                                           sizeof(OMX_BUFFERHEADERTYPE));
    DEBUG_PRINT("AOB::bufHdr %x bufHdr->pBuffer %x", bufHdr, bufHdr->pBuffer);
    bufHdr->nSize             = sizeof(OMX_BUFFERHEADERTYPE);
    bufHdr->nVersion.nVersion = OMX_SPEC_VERSION;
    bufHdr->nAllocLen         = nBufSize;
    bufHdr->pAppPrivate       = appData;
    bufHdr->nOutputPortIndex   = OMX_CORE_OUTPUT_PORT_INDEX;
    m_output_buf_hdrs.insert(bufHdr, NULL);
    m_out_buf_count++;

    }
    else
    {
        DEBUG_PRINT("Output buffer memory allocation failed\n");
        eRet =  OMX_ErrorInsufficientResources;
    }
  return eRet;
}


// AllocateBuffer  -- API Call
/* ======================================================================
FUNCTION
  omx_aac_adec::AllocateBuffer

DESCRIPTION
  Returns zero if all the buffers released..

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::allocate_buffer(OMX_IN OMX_HANDLETYPE                hComp,
                                     OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                                     OMX_IN OMX_U32                        port,
                                     OMX_IN OMX_PTR                     appData,
                                     OMX_IN OMX_U32                       bytes)
{

    OMX_ERRORTYPE eRet = OMX_ErrorNone; // OMX return type


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
        post_command(OMX_CommandStateSet,OMX_StateIdle,
                   OMX_COMPONENT_GENERATE_EVENT);
      }
    }
    DEBUG_PRINT("Allocate Buffer exit with ret Code %d\n", eRet);
    return eRet;
}

/* ======================================================================
FUNCTION
  omx_aac_adec::UseBuffer

DESCRIPTION
  OMX Use Buffer method implementation.

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None , if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::use_buffer(
                         OMX_IN OMX_HANDLETYPE            hComp,
                         OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                         OMX_IN OMX_U32                   port,
                         OMX_IN OMX_PTR                   appData,
                         OMX_IN OMX_U32                   bytes,
                         OMX_IN OMX_U8*                   buffer)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;

    if(port == OMX_CORE_INPUT_PORT_INDEX)
    {
      eRet = use_input_buffer(hComp,bufferHdr,port,appData,bytes,buffer);
    }
    else if(port == OMX_CORE_OUTPUT_PORT_INDEX)
    {
      eRet = use_output_buffer(hComp,bufferHdr,port,appData,bytes,buffer);
    }
    else
    {
      DEBUG_PRINT_ERROR("Error: Invalid Port Index received %d\n",(int)port);
      eRet = OMX_ErrorBadPortIndex;
    }

    if((eRet == OMX_ErrorNone) && (BITMASK_PRESENT(&m_flags,OMX_COMPONENT_IDLE_PENDING)))
    {
      DEBUG_PRINT("Checking for Output Allocate buffer Done");
      if(allocate_done())
      {
        DEBUG_PRINT("omx_aac_adec::use_buffer::Sending OMX_StateIdle");
        // Send the callback now
        BITMASK_CLEAR(&m_flags, OMX_COMPONENT_IDLE_PENDING);
        post_command(OMX_CommandStateSet,OMX_StateIdle,
                   OMX_COMPONENT_GENERATE_EVENT);
      }
    }

    return eRet;
}
/* ======================================================================
FUNCTION
  omx_aac_adec::UseInputBuffer

DESCRIPTION
  Helper function for Use buffer in the input pin

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::use_input_buffer(
                         OMX_IN OMX_HANDLETYPE            hComp,
                         OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                         OMX_IN OMX_U32                   port,
                         OMX_IN OMX_PTR                   appData,
                         OMX_IN OMX_U32                   bytes,
                         OMX_IN OMX_U8*                   buffer)
{
  OMX_ERRORTYPE         eRet = OMX_ErrorNone;
  OMX_BUFFERHEADERTYPE            *bufHdr;
  unsigned                   nBufSize = MAX(bytes, input_buffer_size);
  char                       *buf_ptr;

  DEBUG_PRINT("Inside omx_aac_adec::use_input_buffer");

  buf_ptr = (char *) calloc(sizeof(OMX_BUFFERHEADERTYPE), 1);

  if (buf_ptr != NULL) {
    bufHdr = (OMX_BUFFERHEADERTYPE *) buf_ptr;
    *bufferHdr = bufHdr;
    memset(bufHdr,0,sizeof(OMX_BUFFERHEADERTYPE));


    bufHdr->pBuffer           = (OMX_U8 *)(buffer);
    DEBUG_PRINT("bufHdr %x bufHdr->pBuffer %x", bufHdr, bufHdr->pBuffer);
    bufHdr->nSize             = sizeof(OMX_BUFFERHEADERTYPE);
    bufHdr->nVersion.nVersion = OMX_SPEC_VERSION;
    bufHdr->nAllocLen         = nBufSize;
    input_buffer_size         = nBufSize;
    bufHdr->pAppPrivate       = appData;
    bufHdr->nInputPortIndex   = OMX_CORE_INPUT_PORT_INDEX;
    m_input_buf_hdrs.insert(bufHdr, NULL);
    m_inp_buf_count++;
  }
  else
  {
    DEBUG_PRINT("Input buffer memory allocation failed\n");
    eRet =  OMX_ErrorInsufficientResources;
  }
  return eRet;


}

/* ======================================================================
FUNCTION
  omx_aac_adec::UseOutputBuffer

DESCRIPTION
  Helper function for Use buffer in the input pin

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::use_output_buffer(
                         OMX_IN OMX_HANDLETYPE            hComp,
                         OMX_INOUT OMX_BUFFERHEADERTYPE** bufferHdr,
                         OMX_IN OMX_U32                   port,
                         OMX_IN OMX_PTR                   appData,
                         OMX_IN OMX_U32                   bytes,
                         OMX_IN OMX_U8*                   buffer)
{
  OMX_ERRORTYPE         eRet = OMX_ErrorNone;
  OMX_BUFFERHEADERTYPE            *bufHdr;
  unsigned                   nBufSize = MAX(bytes,output_buffer_size);
  char                       *buf_ptr;

  DEBUG_PRINT("Inside omx_aac_adec::use_output_buffer");

  buf_ptr = (char *) calloc(sizeof(OMX_BUFFERHEADERTYPE), 1);

  if (buf_ptr != NULL) {
    bufHdr = (OMX_BUFFERHEADERTYPE *) buf_ptr;
    DEBUG_PRINT("BufHdr=%p buffer=%p\n",bufHdr,buffer);
    *bufferHdr = bufHdr;
    memset(bufHdr,0,sizeof(OMX_BUFFERHEADERTYPE));

    bufHdr->pBuffer           = (OMX_U8 *)(buffer);
    DEBUG_PRINT("bufHdr %x bufHdr->pBuffer %x", bufHdr, bufHdr->pBuffer);
    bufHdr->nSize             = sizeof(OMX_BUFFERHEADERTYPE);
    bufHdr->nVersion.nVersion = OMX_SPEC_VERSION;
    bufHdr->nAllocLen         = nBufSize;
    output_buffer_size        = nBufSize;
    bufHdr->pAppPrivate       = appData;
    bufHdr->nOutputPortIndex   = OMX_CORE_OUTPUT_PORT_INDEX;
    m_output_buf_hdrs.insert(bufHdr, NULL);
    m_out_buf_count++;
  }
  else
  {
    DEBUG_PRINT("Output buffer memory allocation failed\n");
    eRet =  OMX_ErrorInsufficientResources;

  }

  return eRet;

}

/**
 @brief member function that searches for caller buffer

 @param buffer pointer to buffer header
 @return bool value indicating whether buffer is found
 */
bool omx_aac_adec::search_input_bufhdr(OMX_BUFFERHEADERTYPE *buffer)
{
  bool eRet = false;
  OMX_BUFFERHEADERTYPE *temp = NULL;

  //access only in IL client context

  temp = m_input_buf_hdrs.find_ele(buffer);
  if(buffer && temp)
  {
      DEBUG_DETAIL("search_input_bufhdr %x \n", buffer);
      eRet = true;
  }

  return eRet;
}

/**
 @brief member function that searches for caller buffer

 @param buffer pointer to buffer header
 @return bool value indicating whether buffer is found
 */
bool omx_aac_adec::search_output_bufhdr(OMX_BUFFERHEADERTYPE *buffer)
{
  bool eRet = false;
  OMX_BUFFERHEADERTYPE *temp = NULL;


  //access only in IL client context
  temp = m_output_buf_hdrs.find_ele(buffer);
  if(buffer && temp)
  {
      DEBUG_DETAIL("search_output_bufhdr %x \n", buffer);
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
OMX_ERRORTYPE  omx_aac_adec::free_buffer(OMX_IN OMX_HANDLETYPE         hComp,
                                      OMX_IN OMX_U32                 port,
                                      OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;

  DEBUG_PRINT("Free_Buffer buf %x\n", buffer);


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
  if(port == OMX_CORE_OUTPUT_PORT_INDEX)
  {
   if(search_output_bufhdr(buffer) == true)
    {
      /* Buffer exist */
      //access only in IL client context
      DEBUG_PRINT_ERROR(": free_buffer , Output buffer headerbuffer=%p\n",buffer);
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
      post_command(OMX_CommandStateSet,
                 OMX_StateLoaded,OMX_COMPONENT_GENERATE_EVENT);
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
OMX_ERRORTYPE  omx_aac_adec::empty_this_buffer(OMX_IN OMX_HANDLETYPE         hComp,
                                              OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  bool bPost = false;

  DEBUG_PRINT("ETB:Buf:%x Len %d TS %ld numInBuf=%d\n", buffer, buffer->nFilledLen, buffer->nTimeStamp, (nNumInputBuf+1));

  if ((m_state != OMX_StateExecuting) &&
      (m_state != OMX_StatePause))
  {
    DEBUG_PRINT_ERROR("Invalid state\n");
    eRet = OMX_ErrorInvalidState;
  }

  if (eRet == OMX_ErrorNone) {
    if (search_input_bufhdr(buffer) == true) {
      post_input((unsigned)hComp,
                 (unsigned) buffer,OMX_COMPONENT_GENERATE_ETB);
    } else {
      DEBUG_PRINT_ERROR("Bad header %x \n", buffer);
      eRet = OMX_ErrorBadParameter;
    }
  }
  nNumInputBuf++;

  return eRet;
}
/**
  @brief member function that configures the kernel driver
  @return error status
 */
OMX_ERRORTYPE  omx_aac_adec::config_AAC()
{
    struct msm_audio_aac_config aac_config;
    if (ioctl(m_drv_fd, AUDIO_GET_AAC_CONFIG, &aac_config)) {
        DEBUG_PRINT("omx_aac_adec::config_AAC():AUDIO_GET_AAC_CONFIG failed");
        close(m_drv_fd);
                return OMX_ErrorInvalidComponent;
    }
    DEBUG_PRINT("omx_aac_adec::config_AAC():AUDIO_GET_AAC_CONFIG DONE");

    switch(m_adec_param.eAACStreamFormat)
    {
        case OMX_AUDIO_AACStreamFormatADIF:
        case OMX_AUDIO_AACStreamFormatRAW:
        default:
              aac_config.format = AUDIO_AAC_FORMAT_RAW;
              break;
        case OMX_AUDIO_AACStreamFormatMP4LOAS:
             aac_config.format = AUDIO_AAC_FORMAT_LOAS;
             break;
        case OMX_AUDIO_AACStreamFormatMP4ADTS:
             aac_config.format = AUDIO_AAC_FORMAT_ADTS;
             break;
    }

    switch(m_adec_param.eAACProfile)
    {
        case  OMX_AUDIO_AACObjectLC:
        default:
            aac_config.sbr_on_flag = 0;
            aac_config.sbr_ps_on_flag = 0;
            break;
        case  OMX_AUDIO_AACObjectHE:
            aac_config.sbr_on_flag = 1;
            aac_config.sbr_ps_on_flag = 0;
            break;
        case  OMX_AUDIO_AACObjectHE_PS:
            aac_config.sbr_on_flag = 1;
            aac_config.sbr_ps_on_flag = 1;
            break;
    }


   aac_config.channel_configuration = m_adec_param.nChannels;
   if (ioctl(m_drv_fd, AUDIO_SET_AAC_CONFIG, &aac_config)) {
        DEBUG_PRINT("omx_aac_adec::config_AAC():AUDIO_SET_AAC_CONFIG failed");
        close(m_drv_fd);
        return OMX_ErrorInvalidComponent;
   }
   return OMX_ErrorNone;
}
/**
  @brief member function that writes data to kernel driver

  @param hComp handle to component instance
  @param buffer pointer to buffer header
  @return error status
 */

OMX_ERRORTYPE  omx_aac_adec::empty_this_buffer_proxy(OMX_IN OMX_HANDLETYPE         hComp,
                                                     OMX_BUFFERHEADERTYPE* buffer)
{
  struct aac_header aac_header_info;
  int res = 0;
  struct msm_audio_config drv_config;
  struct msm_audio_aac_config aac_config;

  static int fwrite_count = 0;
  OMX_STATETYPE state;

  /* Assume empty this buffer function has already checked
     validity of buffer */
  DEBUG_PRINT("ETBProxy-->buffer[%x] len[%d] TS[%ld] flags[%d]\n", \
                                                           buffer,
                                                           buffer->nFilledLen,
                                                           buffer->nTimeStamp,
                                                           buffer->nFlags);

  bool flg= false;

  if(m_first_aac_header == 0)
  {
      DEBUG_PRINT("empty_this_buffer_proxy: Callin aac frame header parsing\n");
      res = aac_frameheader_parser( (OMX_BUFFERHEADERTYPE*)buffer,&aac_header_info);
      if(res ==0)
      {
          op_settings_changed = 1;
          m_first_aac_header = 1; /* No more parsing after first frame*/
          ioctl(m_drv_fd, AUDIO_GET_CONFIG, &drv_config);
          ioctl(m_drv_fd, AUDIO_GET_AAC_CONFIG, &aac_config);
          DEBUG_PRINT("GET_AAC_CONFIG::format=%d ch_cfg=%d audio_obj=%d\n",
              aac_config.format,aac_config.channel_configuration,aac_config.audio_object);
          if(aac_header_info.input_format == FORMAT_ADIF)
          {
              // sampling frequency
              drv_config.sample_rate = aac_header_info.head.adif.sample_rate;
              drv_config.channel_count =
                                   aac_header_info.head.adif.channel_config;
              m_adec_param.nSampleRate = aac_header_info.head.adif.sample_rate;
              m_adec_param.nChannels = aac_header_info.head.adif.channel_config;
              m_adec_param.eAACStreamFormat = OMX_AUDIO_AACStreamFormatADIF;
          }
          else if(aac_header_info.input_format == FORMAT_ADTS)
          {
              // sampling frequency
              drv_config.sample_rate =
                 aac_frequency_index[aac_header_info.head.adts.fixed.sampling_frequency_index];
              drv_config.channel_count =
                                   aac_header_info.head.adts.fixed.channel_configuration;
              m_adec_param.nSampleRate =
                  aac_frequency_index[aac_header_info.head.adts.fixed.sampling_frequency_index];
              m_adec_param.nChannels =
                                    aac_header_info.head.adts.fixed.channel_configuration;
              m_adec_param.eAACStreamFormat = OMX_AUDIO_AACStreamFormatMP4ADTS;
              aac_config.channel_configuration = drv_config.channel_count;

          }
          else if(aac_header_info.input_format == FORMAT_LOAS)
          {
              m_adec_param.eAACStreamFormat = OMX_AUDIO_AACStreamFormatMP4LOAS;
              drv_config.sample_rate   = aac_frequency_index[aac_header_info.head.loas.freq_index];
              drv_config.channel_count = aac_header_info.head.loas.channel_config;
              m_adec_param.nSampleRate = aac_frequency_index[aac_header_info.head.loas.freq_index];
              m_adec_param.nChannels   = aac_header_info.head.loas.channel_config;
          }
          else
          {
              DEBUG_PRINT("RAW AAC FORMAT\n");
              // sampling frequency
              drv_config.sample_rate = (aac_frequency_index[aac_header_info.head.raw.freq_index]);
              drv_config.channel_count = aac_header_info.head.raw.channel_config;
              m_adec_param.nSampleRate = (aac_frequency_index[aac_header_info.head.raw.freq_index]);
              m_adec_param.nChannels = aac_header_info.head.raw.channel_config;
              DEBUG_PRINT("sample_rate=%d ch_cnt=%d\n",drv_config.sample_rate,drv_config.channel_count);
              flg = true;
              aac_config.format = AUDIO_AAC_FORMAT_RAW;
              aac_config.channel_configuration = drv_config.channel_count;

          }
          drv_config.type = 2; // aac decoding ??
          ioctl(m_drv_fd, AUDIO_SET_CONFIG, &drv_config);
          config_AAC();
          DEBUG_PRINT("SET_AAC_CONFIG::format=%d ch_cfg=%d audio_obj=%d\n",
                   aac_config.format,aac_config.channel_configuration,aac_config.audio_object);

          ioctl(m_drv_fd, AUDIO_START, 0);

          DEBUG_PRINT("POSTING TO CMD--> OMX_COMPONENT_PORTSETTINGS_CHANGED");

          post_command((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_PORTSETTINGS_CHANGED);
      }
      else
      {
        DEBUG_PRINT("configure Driver for AAC playback sample rate = %d \n",m_adec_param.nSampleRate);
        ioctl(m_drv_fd, AUDIO_GET_CONFIG, &drv_config);
        drv_config.sample_rate = m_adec_param.nSampleRate;
        drv_config.channel_count = m_adec_param.nChannels;
        drv_config.type = 2; // aac decoding
        ioctl(m_drv_fd, AUDIO_SET_CONFIG, &drv_config);
        config_AAC();
        ioctl(m_drv_fd, AUDIO_START, 0);
      }
  }

      if(!flg)
          write(m_drv_fd, buffer->pBuffer, buffer->nFilledLen);

  nTimestamp = buffer->nTimeStamp;

  if(buffer->nFlags & 0x01)
  {
      DEBUG_PRINT("EOS OCCURED \n");
      fsync(m_drv_fd);
      endofstream = 1;
      ntotal_playtime = buffer->nTimeStamp;
  }
  pthread_mutex_lock(&m_state_lock);
  get_state(&m_cmp, &state);
  pthread_mutex_unlock(&m_state_lock);

  if (state == OMX_StateExecuting)
  {
      DEBUG_DETAIL("In Exe state, EBD CB");
      buffer_done_cb((OMX_BUFFERHEADERTYPE *)buffer);
  }
  else
  {
    // post the event to input thread.
    DEBUG_DETAIL("Not in exe state, post EBD to input thread thread");
    post_input((unsigned) & hComp,(unsigned) buffer,
                                 OMX_COMPONENT_GENERATE_BUFFER_DONE);
  }
  return OMX_ErrorNone;
}

OMX_ERRORTYPE  omx_aac_adec::aac_frameheader_parser(OMX_BUFFERHEADERTYPE* bufHdr,struct aac_header *header)
{
    OMX_U8 *temp;
    OMX_U8 *buf = bufHdr->pBuffer;
    OMX_U8 cnt = 0;

    if( (buf[0] == 0x41) &&  (buf[1] == 0x44) &&
        (buf[2] == 0x49) &&  (buf[3] == 0x46) )
       /*check "ADIF" */
    {
        // format is ADIF
        // parser to parse ADIF
        header->input_format = FORMAT_ADIF;
        audaac_extract_adif_header(bufHdr->pBuffer,header);
    }
    else if ((buf[0] == 0xFF) && ((buf[1] & 0xF6) == 0xF0) )
    {
       OMX_U32 index=0;

        /* extract Sync word */
       //format is ADTS
       //extract MPEG "ID" bit from the header
       if (buf[1] & 0x08)
       {
           //format is ADTS mpeg2
       }
       else
       {
           //format is ADTS mpeg4
       }
       header->input_format = FORMAT_ADTS;
       // Get the frequency index from bits 19,20,21,22
       header->head.adts.fixed.sampling_frequency_index =
                                                     (buf[2] & 0x3C) >> 2;
       DEBUG_PRINT("ADTS-->freq_index=0x%x\n",header->head.adts.fixed.sampling_frequency_index);

       // Get the channel configuration bits 24,25,26
       index = (buf[2] & 0x01) << 0x02 ;
       index |= (buf[3] & 0xC0) >> 0x06;
       header->head.adts.fixed.channel_configuration = index;
       DEBUG_PRINT("ADTS-->ch cfg=0x%x\n",header->head.adts.fixed.channel_configuration);

    }
    /*else if(LOAS or LATM*/
    else if ((buf[0] == 0x56) && ((buf[1] & 0xE0) == 0xE0) )
    {
        header->input_format = FORMAT_LOAS;
        audaac_extract_loas_header(bufHdr->pBuffer,header);
    }
    else
    {
        // RAW AAC-ADTS PARSER
        header->input_format = FORMAT_RAW;

        // Get Audio object type[5bits]
        header->head.raw.aud_obj_type = (buf[0] & 0xF8) >> 0x03;
        DEBUG_PRINT("RAW-->aud_obj_type=0x%x\n",header->head.raw.aud_obj_type);

        // Get frequency index..bits 6,7,8,9
        cnt = buf[1] & 0x80;
        cnt = cnt >> 0x07;
        header->head.raw.freq_index = ((buf[0] & 0x07) << 1 );
        header->head.raw.freq_index |= cnt;
        DEBUG_PRINT("RAW-->freq_index=0x%x\n",header->head.raw.freq_index);

        // Channel config..bits 10,11,12,13
        header->head.raw.channel_config = (buf[1] & 0x78) >> 3;
        DEBUG_PRINT("RAW-->ch cfg=0x%x\n",header->head.raw.channel_config);
        if(header->head.raw.channel_config == 7)
        {
            header->head.raw.channel_config = 8;
            /* ch_config  num of channels
               ---------  ---------------
                   0  ----> custom config
                   1  ----> 1
                   2  ----> 2
                   3  ----> 3
                   4  ----> 4
                   5  ----> 5
                   6  ----> 6
                   7  ----> 8
            */
        }
    }
    return OMX_ErrorNone;
}

OMX_ERRORTYPE  omx_aac_adec::fill_this_buffer_proxy(OMX_IN OMX_HANDLETYPE         hComp,
                                                     OMX_BUFFERHEADERTYPE* buffer)
{
  static int count = 0;
  int nDatalen = 0;
  static int fwrite_count = 0;
  static int set_pcm_config = 0;
  OMX_STATETYPE state;

  DEBUG_PRINT("FTBP-->bufHdr=%p buffer=%p alloclen=%d",buffer, buffer->pBuffer,buffer->nAllocLen);


  if (0 == set_pcm_config)
  {
    struct msm_audio_pcm_config  pcm_config;

    DEBUG_PRINT("FTBP-->configure driver mode as %d \n",pcm_feedback);
    ioctl(m_drv_fd, AUDIO_GET_PCM_CONFIG, &pcm_config);
    pcm_config.buffer_size  = output_buffer_size;
    ioctl(m_drv_fd, AUDIO_SET_PCM_CONFIG, &pcm_config);
    set_pcm_config++;
    if(m_adec_param.nChannels == 1)
    {
        frameDuration = (((output_buffer_size)* 1000) /(m_adec_param.nSampleRate * 2));
        DEBUG_PRINT("frame duration of mono config = %d sampling rate = %d \n",frameDuration,m_adec_param.nSampleRate);
    }
    else if(m_adec_param.nChannels == 2)
    {
        frameDuration = (((output_buffer_size)* 1000) /(m_adec_param.nSampleRate * 4));
        DEBUG_PRINT("frame duration of stero config = %d sampling rate = %d \n",frameDuration,m_adec_param.nSampleRate);
    }
  }

  /* Assume fill this buffer function has already checked
     validity of buffer */

  if(search_output_bufhdr(buffer) == true)
  {
      if (0 == bEOSSent)
      {
        int ncount = buffer->nAllocLen/OMX_AAC_OUTPUT_BUFFER_SIZE;
        int nRead = 0;
        int nReadbytes = 0;
        OMX_IN OMX_U8  *pBuf = NULL;


        pBuf = buffer->pBuffer;

        for (nRead = 0; nRead < ncount; nRead++)
        {
          pBuf += nReadbytes;

          nReadbytes = read(m_drv_fd, pBuf,OMX_AAC_OUTPUT_BUFFER_SIZE);
          DEBUG_DETAIL("FTBP->Al_len[%d]buf[%p]Loop[%d]size[%d]numOutBuf[%d]\n",\
                                      buffer->nAllocLen, buffer->pBuffer,
                                      nRead, nReadbytes,nNumOutputBuf);

          if ((-1 == nReadbytes) || (endofstream == 1))
          {
            DEBUG_PRINT("FTBP: breaking read since nReadbytes is -1 or EOS is reached");
            break;
          }
          // just reading msize, should it be guarded by mutex ?
          else if(m_output_ctrl_cmd_q.m_size)
          {
              DEBUG_DETAIL("FTBP-->: FLUSH CMD IN Q, STOP READING");
              nDatalen += nReadbytes;
              break;
          }
          nDatalen = nDatalen + nReadbytes;
        }
        buffer->nFilledLen = nDatalen;
      }
      else
      {
        buffer->nFilledLen = 0;
        buffer->nFlags = 1;
        frame_done_cb((OMX_BUFFERHEADERTYPE *)buffer);
        return OMX_ErrorNone;
      }

      if((nDatalen < 0) || (nDatalen > output_buffer_size))
      {
        buffer->nFilledLen = 0;
        frame_done_cb((OMX_BUFFERHEADERTYPE *)buffer);
          DEBUG_PRINT("FTBP-->: Invalid data length read \n");
      }
      else
      {
          if((endofstream == 1)  && (nTimestamp >= ntotal_playtime))
          {
              DEBUG_PRINT("FTBP-->EOS TS[%d] Total playtime[%d] \n",\
                                                            nTimestamp,ntotal_playtime);
              buffer->nFilledLen = 0;
              buffer->nFlags = 1;


              if (0 == bEOSSent)
              {
                post_output((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_GENERATE_EOS);
                bEOSSent = 1;
              }

              endofstream = 0;
              DEBUG_PRINT("FTBP--> EOS valid data length read = %d\n",buffer->nFilledLen);
               frame_done_cb((OMX_BUFFERHEADERTYPE *)buffer);
              return OMX_ErrorNone;

          }
          else
          {
              DEBUG_DETAIL("FTBP-->DATA LENGTH[%d]\n",buffer->nFilledLen);
              buffer->nFilledLen = nDatalen;
          }

          pthread_mutex_lock(&m_state_lock);
          get_state(&m_cmp, &state);
          pthread_mutex_unlock(&m_state_lock);

          if (state == OMX_StatePause)
          {
            // post the event to output thread.
            post_output((unsigned) & hComp,(unsigned) buffer,OMX_COMPONENT_GENERATE_FRAME_DONE);
          }
          else
          {
              frame_done_cb((OMX_BUFFERHEADERTYPE *)buffer);
          }
      }
  }
  else
  {
      DEBUG_PRINT("\n FTBP-->Invalid buffer in FTB \n");

  }

  return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  omx_aac_adec::FillThisBuffer

DESCRIPTION
  IL client uses this method to release the frame buffer
  after displaying them.

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::fill_this_buffer(OMX_IN OMX_HANDLETYPE         hComp,
                                              OMX_IN OMX_BUFFERHEADERTYPE* buffer)
{
    OMX_ERRORTYPE eRet = OMX_ErrorNone;

    nNumOutputBuf++;
    DEBUG_DETAIL("FTB:nNumOutputBuf is %d", nNumOutputBuf);


    post_output((unsigned)hComp,
                 (unsigned) buffer,OMX_COMPONENT_GENERATE_FTB);
    return eRet;
}

/* ======================================================================
FUNCTION
  omx_aac_adec::SetCallbacks

DESCRIPTION
  Set the callbacks.

PARAMETERS
  None.

RETURN VALUE
  OMX Error None if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::set_callbacks(OMX_IN OMX_HANDLETYPE        hComp,
                                           OMX_IN OMX_CALLBACKTYPE* callbacks,
                                           OMX_IN OMX_PTR             appData)
{
  m_cb       = *callbacks;
  m_app_data =    appData;

  return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  omx_aac_adec::ComponentDeInit

DESCRIPTION
  Destroys the component and release memory allocated to the heap.

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if everything successful.

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::component_deinit(OMX_IN OMX_HANDLETYPE hComp)
{
  if (OMX_StateLoaded != m_state)
  {
      DEBUG_PRINT_ERROR("Warning: Received DeInit when not in LOADED state, cur_state %d\n",
                   m_state);
      DEBUG_PRINT("OMX_StateLoaded == %d\n",OMX_StateLoaded);
      return OMX_ErrorInvalidState;
  }

  if(m_drv_fd >=0)
      ioctl(m_drv_fd, AUDIO_STOP, 0);

  if (m_ipc_to_in_th != NULL) {
    omx_aac_thread_stop(m_ipc_to_in_th);
    m_ipc_to_in_th = NULL;
  }

  if (m_ipc_to_cmd_th != NULL) {
    omx_aac_thread_stop(m_ipc_to_cmd_th);
    m_ipc_to_cmd_th = NULL;
  }

  if(pcm_feedback ==1)
  {
    if (m_ipc_to_out_th != NULL) {
      omx_aac_thread_stop(m_ipc_to_out_th);
      m_ipc_to_out_th = NULL;
    }
  }

  endofstream = 0;
  bEOSSent = 0;
  bOutputPortReEnabled = 0;
  nNumInputBuf = 0;
  nNumOutputBuf = 0;
  m_first_aac_header=0;

  if (m_drv_fd >= 0) {
    close(m_drv_fd);
  }
  else
  {
    DEBUG_PRINT(" aac device close failure \n");
  }

  return OMX_ErrorNone;
}

/* ======================================================================
FUNCTION
  omx_aac_adec::UseEGLImage

DESCRIPTION
  OMX Use EGL Image method implementation <TBD>.

PARAMETERS
  <TBD>.

RETURN VALUE
  Not Implemented error.

========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::use_EGL_image(OMX_IN OMX_HANDLETYPE                hComp,
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
  omx_aac_adec::ComponentRoleEnum

DESCRIPTION
  OMX Component Role Enum method implementation.

PARAMETERS
  <TBD>.

RETURN VALUE
  OMX Error None if everything is successful.
========================================================================== */
OMX_ERRORTYPE  omx_aac_adec::component_role_enum(OMX_IN OMX_HANDLETYPE hComp,
                                                OMX_OUT OMX_U8*        role,
                                                OMX_IN OMX_U32        index)
{
  OMX_ERRORTYPE eRet = OMX_ErrorNone;
  const char *cmp_role = "audio_decoder.aac";

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
  omx_aac_adec::AllocateDone

DESCRIPTION
  Checks if entire buffer pool is allocated by IL Client or not.
  Need this to move to IDLE state.

PARAMETERS
  None.

RETURN VALUE
  true/false.

========================================================================== */
bool omx_aac_adec::allocate_done(void)
{
  return (m_inp_buf_count >= OMX_CORE_NUM_INPUT_BUFFERS?true:false);
}


/* ======================================================================
FUNCTION
  omx_aac_adec::ReleaseDone

DESCRIPTION
  Checks if IL client has released all the buffers.

PARAMETERS
  None.

RETURN VALUE
  true/false

========================================================================== */
bool omx_aac_adec::release_done(void)
{
  DEBUG_PRINT("Inside omx_aac_adec::release_done");
  endofstream = 0;
  bEOSSent = 0;
  bOutputPortReEnabled = 0;
#ifdef _ANDROID_
  return true;
#else
  return (m_inp_buf_count == 0?true:false);
#endif

}

void omx_aac_adec::audaac_extract_adif_header(
                                        OMX_U8 *data,
                                        struct aac_header *aac_header_info)
{
  OMX_U8  buf8;
  OMX_U32 buf32;
  OMX_U8  num_pfe, num_fce, num_sce, num_bce, num_lfe, num_ade, num_vce;
  OMX_U8  pfe_index;
  OMX_U8  i;
  OMX_U32 temp;

  /* We already parsed the 'ADIF' keyword. Skip it. */
  m_aac_hdr_bit_index = 32;


  audaac_extract_bits(data, AAC_COPYRIGHT_PRESENT_SIZE, &buf8);

  if(buf8) {
    /* Copyright present; Just discard it for now */
    m_aac_hdr_bit_index += 72;
  }

  audaac_extract_bits(data, AAC_ORIGINAL_COPY_SIZE,
                      &temp);

  audaac_extract_bits(data, AAC_HOME_SIZE, &temp);

  audaac_extract_bits(data, AAC_BITSTREAM_TYPE_SIZE,
                      &aac_header_info->head.adif.variable_bit_rate);

  audaac_extract_bits(data, AAC_BITRATE_SIZE, &temp);

  audaac_extract_bits(data, AAC_NUM_PFE_SIZE, &num_pfe);

  for(pfe_index=0; pfe_index<num_pfe+1; pfe_index++) {
    if(!aac_header_info->head.adif.variable_bit_rate) {
      /* Discard */
      audaac_extract_bits(data, AAC_BUFFER_FULLNESS_SIZE, &buf32);
    }

    /* Extract Program Config Element */

    /* Discard element instance tag */
    audaac_extract_bits(data, AAC_ELEMENT_INSTANCE_TAG_SIZE, &buf8);

    audaac_extract_bits(data, AAC_PROFILE_SIZE, &aac_header_info->head.adif.aud_obj_type);

    buf8 = 0;
    audaac_extract_bits(data, AAC_SAMPLING_FREQ_INDEX_SIZE, &buf8);
    aac_header_info->head.adif.freq_index = buf8;
    aac_header_info->head.adif.sample_rate = aac_frequency_index[buf8];

    audaac_extract_bits(data, AAC_NUM_FRONT_CHANNEL_ELEMENTS_SIZE, &num_fce);

    audaac_extract_bits(data, AAC_NUM_SIDE_CHANNEL_ELEMENTS_SIZE, &num_sce);

    audaac_extract_bits(data, AAC_NUM_BACK_CHANNEL_ELEMENTS_SIZE, &num_bce);

    audaac_extract_bits(data, AAC_NUM_LFE_CHANNEL_ELEMENTS_SIZE, &num_lfe);

    audaac_extract_bits(data, AAC_NUM_ASSOC_DATA_ELEMENTS_SIZE, &num_ade);

    audaac_extract_bits(data, AAC_NUM_VALID_CC_ELEMENTS_SIZE, &num_vce);

    audaac_extract_bits(data, AAC_MONO_MIXDOWN_PRESENT_SIZE, &buf8);
    if(buf8) {
      audaac_extract_bits(data, AAC_MONO_MIXDOWN_ELEMENT_SIZE, &buf8);
    }

    audaac_extract_bits(data, AAC_STEREO_MIXDOWN_PRESENT_SIZE, &buf8);
    if(buf8) {
      audaac_extract_bits(data, AAC_STEREO_MIXDOWN_ELEMENT_SIZE, &buf8);
    }

    audaac_extract_bits(data, AAC_MATRIX_MIXDOWN_PRESENT_SIZE, &buf8);
    if(buf8) {
      audaac_extract_bits(data, AAC_MATRIX_MIXDOWN_SIZE, &buf8);
    }

    for(i=0; i<num_fce; i++) {
      audaac_extract_bits(data, AAC_FCE_SIZE, &buf8);
    }

    for(i=0; i<num_sce; i++) {
      audaac_extract_bits(data, AAC_SCE_SIZE, &buf8);
    }

    for(i=0; i<num_bce; i++) {
      audaac_extract_bits(data, AAC_BCE_SIZE, &buf8);
    }

    for(i=0; i<num_lfe; i++) {
      audaac_extract_bits(data, AAC_LFE_SIZE, &buf8);
    }

    for(i=0; i<num_ade; i++) {
      audaac_extract_bits(data, AAC_ADE_SIZE, &buf8);
    }

    for(i=0; i<num_vce; i++) {
      audaac_extract_bits(data, AAC_VCE_SIZE, &buf8);
    }

    /* byte_alignment() */
    buf8 = (OMX_U8)(m_aac_hdr_bit_index % 8);
    if(buf8) {
      m_aac_hdr_bit_index += 8 - buf8;
    }

    /* get comment_field_bytes */
    buf8 = data[m_aac_hdr_bit_index/8];

    m_aac_hdr_bit_index += AAC_COMMENT_FIELD_BYTES_SIZE;

    /* Skip the comment */
    m_aac_hdr_bit_index += buf8 * AAC_COMMENT_FIELD_DATA_SIZE;
  }

  /* byte_alignment() */
  buf8 = (OMX_U8)(m_aac_hdr_bit_index % 8);
  if(buf8) {
    m_aac_hdr_bit_index += 8 - buf8;
  }

  aac_header_info->head.adif.channel_config =
               (num_fce + num_sce + num_bce +
                                           num_lfe + num_ade + num_vce) ;
}

void omx_aac_adec::audaac_extract_bits(
                                      OMX_U8  *input,
                                      OMX_U8  num_bits_reqd,
                                      void    *out_buf
)
{
  OMX_U32 output = 0;
  OMX_U32 value = 0;
  OMX_U32 byte_index;
  OMX_U8   bit_index;
  OMX_U8   bits_avail_in_byte;
  OMX_U8   num_to_copy;
  OMX_U8   mask;
  OMX_U8   num_remaining = num_bits_reqd;

  while(num_remaining) {
    byte_index = m_aac_hdr_bit_index / 8;
    bit_index  = m_aac_hdr_bit_index % 8;

    bits_avail_in_byte = 8 - bit_index;
    num_to_copy = MIN(bits_avail_in_byte, num_remaining);

    mask = ~(0xff << bits_avail_in_byte);

    value = input[byte_index] & mask;
    value = value >> (bits_avail_in_byte - num_to_copy);

    m_aac_hdr_bit_index += num_to_copy;
    num_remaining -= num_to_copy;

    output = (output << num_to_copy) | value;
  }

  if(num_bits_reqd <= 8) {
    OMX_U8 *buf8 = (OMX_U8*)out_buf;
    *buf8 = (OMX_U8)output;
  }
  else if(num_bits_reqd <= 16) {
    OMX_U16 *buf16 = (OMX_U16*)out_buf;
    *buf16 = (OMX_U16)output;
  }
  else if(num_bits_reqd <= 32) {
    OMX_U32 *buf32 = (OMX_U32*)out_buf;
    *buf32 = (OMX_U32)output;
  }
}

void omx_aac_adec::audaac_extract_loas_header(
    OMX_U8 *data,
    struct aac_header *aac_header_info)
{
  OMX_U16      aac_frame_length = 0;
  OMX_U32      value    = 0;
  OMX_U8       obj_type = 0;
  OMX_U8       ext_flag = 0;

  OMX_U8       num_fc   = 0;
  OMX_U8       num_sc   = 0;
  OMX_U8       num_bc   = 0;
  OMX_U8       num_lfe  = 0;
  OMX_U8       num_ade  = 0;
  OMX_U8       num_vce  = 0;

  OMX_U16      num_bits_to_skip = 0;

  /* Length is in the bits 11 to 23 from left in the bitstream */

  m_aac_hdr_bit_index = 11;
  audaac_extract_bits(data, 13, &aac_frame_length);

  /* useSameStreamMux */
  audaac_extract_bits(data, 1, &value);

  if(value)
  {
    return;
  }
  /* Audio mux version */
  audaac_extract_bits(data, 1, &value);
  if (value)
  {
    /* Unsupported format */
    return;
  }
  /* allStreamsSameTimeFraming */
  audaac_extract_bits(data, 1, &value);
  if (!value)
  {
    /* Unsupported format */
    return;
  }
  /* numSubFrames */
  audaac_extract_bits(data, 6, &value);
  if (value)
  {
    /* Unsupported format */
    return;
  }
  /* numProgram */
  audaac_extract_bits(data, 4, &value);
  if (value)
  {
    /* Unsupported format */
    return;
  }
  /* numLayer */
  audaac_extract_bits(data, 3, &value);
  if (value)
  {
    /* Unsupported format */
    return;
  }

  /* Audio specific config */
  /* audioObjectType */
  audaac_extract_bits(data, 5, &value);
  if (!LOAS_IS_AUD_OBJ_SUPPORTED(value))
  {
    /* Unsupported format */
    return;
  }
  aac_header_info->head.loas.aud_obj_type = value;

  /* SFI */
  audaac_extract_bits(data, 4, &value);
  if (!LOAS_IS_SFI_SUPPORTED(value))
  {
    /* Unsupported format */
    return;
  }
  aac_header_info->head.loas.freq_index = value;
  /* Channel config */
  audaac_extract_bits(data, 4, &value);
  if (!LOAS_IS_CHANNEL_CONFIG_SUPPORTED(value, aac_header_info->head.loas.aud_obj_type))
  {
    /* Unsupported format */
    return;
  }

  aac_header_info->head.loas.channel_config = value;

  if (aac_header_info->head.loas.aud_obj_type == 5)
  {
    /* Extension SFI */
    audaac_extract_bits(data, 4, &value);
    if (!LOAS_IS_EXT_SFI_SUPPORTED(value))
    {
      /* Unsupported format */
      return;
    }
    /* Audio object_type */
    audaac_extract_bits(data, 5, &value);
    if (!LOAS_IS_AUD_OBJ_SUPPORTED(value))
    {
      /* Unsupported format */
      return;
    }
    aac_header_info->head.loas.aud_obj_type = value;
  }
  obj_type = aac_header_info->head.loas.aud_obj_type;

  if (LOAS_GA_SPECIFIC_CONFIG(obj_type))
  {
    /* framelengthFlag */
    audaac_extract_bits(data, 1, &value);
    if (value)
    {
      /* Unsupported format */
      return;
    }

    /* dependsOnCoreCoder */
    audaac_extract_bits(data, 1, &value);
    if (value)
    {
      /* Unsupported format */
      return;
    }

    /* extensionFlag */
    audaac_extract_bits(data, 1, &ext_flag);
    if (!LOAS_IS_EXT_FLG_SUPPORTED(ext_flag,obj_type) )
    {
      /* Unsupported format */
      return;
    }
    if (!aac_header_info->head.loas.channel_config)
    {
      /* Skip 10 bits */
      audaac_extract_bits(data, 10, &value);

      audaac_extract_bits(data, 4, &num_fc);
      audaac_extract_bits(data, 4, &num_sc);
      audaac_extract_bits(data, 4, &num_bc);
      audaac_extract_bits(data, 2, &num_lfe);
      audaac_extract_bits(data, 3, &num_ade);
      audaac_extract_bits(data, 4, &num_vce);

      /* mono_mixdown_present */
      audaac_extract_bits(data, 1, &value);
      if (value) {
        /* mono_mixdown_element_number */
        audaac_extract_bits(data, 4, &value);
      }

      /* stereo_mixdown_present */
      audaac_extract_bits(data, 1, &value);
      if (value) {
        /* stereo_mixdown_element_number */
        audaac_extract_bits(data, 4, &value);
      }

      /* matrix_mixdown_idx_present */
      audaac_extract_bits(data, 1, &value);
      if (value) {
        /* matrix_mixdown_idx and presudo_surround_enable */
        audaac_extract_bits(data, 3, &value);
      }
      num_bits_to_skip = (num_fc * 5) + (num_sc * 5) + (num_bc * 5) +
                         (num_lfe * 4) + (num_ade * 4) + (num_vce * 5);
      while (num_bits_to_skip != 0){
        if (num_bits_to_skip > 32) {
          audaac_extract_bits(data, 32, &value);
          num_bits_to_skip -= 32;
        } else {
          audaac_extract_bits(data, num_bits_to_skip, &value);
          num_bits_to_skip = 0;
        }
      }

     if (m_aac_hdr_bit_index & 0x07)
     {
       m_aac_hdr_bit_index +=  (8 - (m_aac_hdr_bit_index & 0x07));
     }
      /* comment_field_bytes */
      audaac_extract_bits(data, 8, &value);

      num_bits_to_skip = value * 8;

      while (num_bits_to_skip != 0){
        if (num_bits_to_skip > 32) {
          audaac_extract_bits(data, 32, &value);
          num_bits_to_skip -= 32;
        } else {
          audaac_extract_bits(data, num_bits_to_skip, &value);
          num_bits_to_skip = 0;
        }
      }
    }

    if (ext_flag)
    {
      if (((obj_type == 17) || (obj_type == 19) ||
           (obj_type == 20) || (obj_type == 23)))
      {
        audaac_extract_bits(data, 1, &value);
        //aac_info.config.aac_section_data_resilience_flag = value;

        audaac_extract_bits(data, 1, &value);
        //aac_info.config.aac_scalefactor_data_resilience_flag = value;

        audaac_extract_bits(data, 1, &value);
        //aac_info.config.aac_spectral_data_resilience_flag = value;
      }
      /* extensionFlag3 */
      audaac_extract_bits(data, 1, &value);
    }
  }
  if ((obj_type != 18) && (obj_type >= 17) && (obj_type <= 27))
  {
    /* epConfig */
    audaac_extract_bits(data, 2, &value);
    if (value)
    {
      /* Unsupported format */
      return;
    }
  }
  /* Back in StreamMuxConfig */
  /* framelengthType */
  audaac_extract_bits(data, 3, &value);
  if (value)
  {
    /* Unsupported format */
    return;
  }

}

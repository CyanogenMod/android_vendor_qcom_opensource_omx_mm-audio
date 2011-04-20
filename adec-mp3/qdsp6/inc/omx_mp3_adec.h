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

  @file comx_mp3_adec.h
  This module contains the class definition for openMAX MP3 decoder component.

*//*========================================================================*/


//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////

#include<stdlib.h>

#define LOG_TAG "QC Audio MP3 Decoder"

#include <stdio.h>
#include <pthread.h>
#include <inttypes.h>
#include <unistd.h>

#include "OMX_Core.h"
#include "OMX_Audio.h"
#include "adec_svr.h"
#include "qc_omx_component.h"
#include "Map.h"

extern "C" {
  void * get_omx_component_factory_fn(void);
}

//#ifdef _DEBUG

#define DEBUG_PRINT(args...) printf("%s:%d ", __FUNCTION__, __LINE__); \
                             printf(args)

#define DEBUG_PRINT_ERROR(args...) printf("%s:%d ", __FUNCTION__, __LINE__); \
                       printf(args)

/*#else

#define DEBUG_PRINT
#define DEBUG_PRINT_ERROR

#endif*/ /* _DEBUG */

//////////////////////////////////////////////////////////////////////////////
//                       Module specific globals
//////////////////////////////////////////////////////////////////////////////



#define OMX_SPEC_VERSION  0x00000101



//////////////////////////////////////////////////////////////////////////////
//               Macros
//////////////////////////////////////////////////////////////////////////////
//
#define PrintFrameHdr(bufHdr) DEBUG_PRINT("bufHdr %x buf %x size %d TS %d\n",     \
                           (unsigned) bufHdr,                                     \
                           (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->pBuffer,   \
                           (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->nFilledLen,\
                           (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->nTimeStamp)

// BitMask Management logic
#define BITS_PER_BYTE 8
#define BITMASK_SIZE(mIndex)            (((mIndex) + BITS_PER_BYTE - 1)/BITS_PER_BYTE)
#define BITMASK_OFFSET(mIndex)          ((mIndex)/BITS_PER_BYTE)
#define BITMASK_FLAG(mIndex)            (1 << ((mIndex) % BITS_PER_BYTE))
#define BITMASK_CLEAR(mArray,mIndex)    (mArray)[BITMASK_OFFSET(mIndex)] &=  ~(BITMASK_FLAG(mIndex))
#define BITMASK_SET(mArray,mIndex)      (mArray)[BITMASK_OFFSET(mIndex)] |=  BITMASK_FLAG(mIndex)
#define BITMASK_PRESENT(mArray,mIndex)  ((mArray)[BITMASK_OFFSET(mIndex)] & BITMASK_FLAG(mIndex)  )
#define BITMASK_ABSENT(mArray,mIndex)   (((mArray)[BITMASK_OFFSET(mIndex)] & BITMASK_FLAG(mIndex)  ) == 0x0)



// OMX mo3 audio decoder class
class omx_mp3_adec: public qc_omx_component
{
  public:
           omx_mp3_adec();  // constructor
  virtual ~omx_mp3_adec();  // destructor

  OMX_ERRORTYPE allocate_buffer(OMX_HANDLETYPE             hComp,
                 OMX_BUFFERHEADERTYPE **bufferHdr,
                 OMX_U32                     port,
                 OMX_PTR                  appData,
                 OMX_U32                    bytes);

  OMX_ERRORTYPE allocate_output_buffer(OMX_HANDLETYPE             hComp,
                                     OMX_BUFFERHEADERTYPE **bufferHdr,
                                     OMX_U32 port,OMX_PTR     appData,
                                     OMX_U32                    bytes);

  OMX_ERRORTYPE component_deinit(OMX_HANDLETYPE hComp);

  OMX_ERRORTYPE component_init(OMX_STRING role);

  OMX_ERRORTYPE component_role_enum(OMX_HANDLETYPE hComp,
                                  OMX_U8         *role,
                                  OMX_U32        index);

  OMX_ERRORTYPE component_tunnel_request(OMX_HANDLETYPE             hComp,
                                       OMX_U32                     port,
                                       OMX_HANDLETYPE     peerComponent,
                                       OMX_U32                 peerPort,
                                       OMX_TUNNELSETUPTYPE *tunnelSetup);

  OMX_ERRORTYPE empty_this_buffer(OMX_HANDLETYPE         hComp,
                                OMX_BUFFERHEADERTYPE *buffer);


  OMX_ERRORTYPE empty_this_buffer_proxy(OMX_HANDLETYPE         hComp,
                                OMX_BUFFERHEADERTYPE *buffer);


  OMX_ERRORTYPE fill_this_buffer(OMX_HANDLETYPE         hComp,
                               OMX_BUFFERHEADERTYPE *buffer);

  OMX_ERRORTYPE fill_this_buffer_proxy(OMX_HANDLETYPE         hComp,
                               OMX_BUFFERHEADERTYPE *buffer);

  OMX_ERRORTYPE free_buffer(OMX_HANDLETYPE         hComp,
                           OMX_U32                 port,
                           OMX_BUFFERHEADERTYPE *buffer);

  OMX_ERRORTYPE get_component_version(OMX_HANDLETYPE              hComp,
                                    OMX_STRING          componentName,
                                    OMX_VERSIONTYPE *componentVersion,
                                    OMX_VERSIONTYPE *     specVersion,
                                    OMX_UUIDTYPE       *componentUUID);

  OMX_ERRORTYPE get_config(OMX_HANDLETYPE      hComp,
                          OMX_INDEXTYPE configIndex,
                          OMX_PTR        configData);

  OMX_ERRORTYPE get_extension_index(OMX_HANDLETYPE     hComp,
                                  OMX_STRING     paramName,
                                  OMX_INDEXTYPE *indexType);

  OMX_ERRORTYPE get_parameter(OMX_HANDLETYPE hComp,
                             OMX_INDEXTYPE paramIndex,
                             OMX_PTR paramData);

  OMX_ERRORTYPE get_state(OMX_HANDLETYPE hComp,
                         OMX_STATETYPE *state);

  void buffer_done_cb(OMX_BUFFERHEADERTYPE *bufHdr);

  void frame_done_cb(OMX_BUFFERHEADERTYPE *bufHdr);

  static void process_event_cb(void *client_data,
                               unsigned char id);

  static void process_output_cb(void *client_data,
                               unsigned char id);


  OMX_ERRORTYPE send_command_proxy(OMX_HANDLETYPE hComp,
                            OMX_COMMANDTYPE  cmd,
                            OMX_U32       param1,
                            OMX_PTR      cmdData);
  OMX_ERRORTYPE send_command(OMX_HANDLETYPE hComp,
                            OMX_COMMANDTYPE  cmd,
                            OMX_U32       param1,
                            OMX_PTR      cmdData);

  bool post_event(unsigned int p1,
                  unsigned int p2,
                  unsigned int id,
                  bool lock);

  bool post_event_output(unsigned int p1,
                  unsigned int p2,
                  unsigned int id,
                  bool lock);

  OMX_ERRORTYPE set_callbacks(OMX_HANDLETYPE hComp,
                             OMX_CALLBACKTYPE *callbacks,
                             OMX_PTR appData);

  OMX_ERRORTYPE set_config(OMX_HANDLETYPE hComp,
                          OMX_INDEXTYPE configIndex,
                          OMX_PTR configData);

  OMX_ERRORTYPE set_parameter(OMX_HANDLETYPE hComp,
                             OMX_INDEXTYPE paramIndex,
                             OMX_PTR paramData);

  OMX_ERRORTYPE use_buffer(OMX_HANDLETYPE             hComp,
                          OMX_BUFFERHEADERTYPE **bufferHdr,
                          OMX_U32                     port,
                          OMX_PTR                  appData,
                          OMX_U32                    bytes,
                          OMX_U8                  *buffer);

  OMX_ERRORTYPE use_EGL_image(OMX_HANDLETYPE             hComp,
                            OMX_BUFFERHEADERTYPE **bufferHdr,
                            OMX_U32                     port,
                            OMX_PTR                  appData,
                            void *                  eglImage);

private:


// Bit Positions
enum flags_bit_positions
{
  // Defer transition to IDLE
  OMX_COMPONENT_IDLE_PENDING            =0x1,
  // Defer transition to LOADING
  OMX_COMPONENT_LOADING_PENDING         =0x2,

  OMX_COMPONENT_MUTED                   =0x3
};

// Deferred callback identifiers
enum
{
  //Event Callbacks from the component thread context
  OMX_COMPONENT_GENERATE_EVENT       = 0x1,

  //Buffer Done callbacks from component thread context
  OMX_COMPONENT_GENERATE_BUFFER_DONE = 0x2,

  OMX_COMPONENT_GENERATE_ETB         = 0x3,
  //Command
  OMX_COMPONENT_GENERATE_COMMAND     = 0x4,

  OMX_COMPONENT_GENERATE_FRAME_DONE,

  OMX_COMPONENT_GENERATE_FTB,
  OMX_COMPONENT_GENERATE_EOS

};

static const unsigned OMX_CORE_NUM_INPUT_BUFFERS  =   2;

static const unsigned OMX_CORE_INPUT_BUFFER_SIZE  = 8192; /* 8K */
static const unsigned OMX_MP3_OUTPUT_BUFFER_SIZE  = 4608*2; //9792; //4608; //4800; /* 8K */

static const unsigned OMX_CORE_CONTROL_CMDQ_SIZE  = 100; //5000;  //100;

typedef Map<OMX_BUFFERHEADERTYPE*, OMX_BUFFERHEADERTYPE*>
                                                   input_buffer_map;

typedef Map<OMX_BUFFERHEADERTYPE*, OMX_BUFFERHEADERTYPE*>
                                                   output_buffer_map;

enum port_indexes
{
  OMX_CORE_INPUT_PORT_INDEX        =0,
  OMX_CORE_OUTPUT_PORT_INDEX       =1
};

struct omx_event
{
  unsigned param1;
  unsigned param2;
  unsigned id;
};

struct omx_cmd_queue
{
  omx_event m_q[OMX_CORE_CONTROL_CMDQ_SIZE];
  unsigned m_read;
  unsigned m_write;
  unsigned m_size;

  omx_cmd_queue();
  ~omx_cmd_queue();
  bool insert_entry(unsigned p1, unsigned p2, unsigned id);
  bool delete_entry(unsigned *p1,unsigned *p2, unsigned *id);
};

  OMX_STATETYPE                m_state;    // OMX State
  OMX_PTR                      m_app_data;    // Application data
  OMX_CALLBACKTYPE             m_cb;       // Application callbacks
  OMX_AUDIO_PARAM_MP3TYPE      m_adec_param; // Cache mp3 decoder parameter
  OMX_S32                      m_volume;   //Unit to be determined
  struct adec_cmd_svr          *m_cmd_svr;  // Command server instance
  struct adec_cmd_cln          *m_cmd_cln;  // Client server instance
  int                          m_drv_fd;   // Kernel device node file handle
  unsigned int                 frameDuration; // holds the duration of each frame
  OMX_U8*                      pFirstOutputBuf; // holds the address of the first outputbuf
  OMX_U8*                      pSecondOutputBuf; // holds the address of the second outputbuf
  input_buffer_map             m_input_buf_hdrs; //Input buffer header list
  output_buffer_map            m_output_buf_hdrs; //Output buffer header list

  omx_cmd_queue                m_data_q;  // Data command Q
  omx_cmd_queue                m_cmd_q;   // Command Q for rest of the events
  omx_cmd_queue                m_output_q;   // Command Q for rest of the events
  unsigned int                 m_inp_buf_count;   // Number of Input Buffers
  unsigned int                 m_out_buf_count;   // Number of Output Buffers

  unsigned int                 m_flags;   // encapsulate the waiting states.
  unsigned int                 fcount;
  unsigned int                 nTimestamp;
  unsigned int                 pcm_feedback;    // enable tunnel or non-tunnel mode
  unsigned int                 ntotal_playtime;
  unsigned int                 output_buffer_size;
  unsigned int                 input_buffer_size;
  pthread_mutex_t              m_lock;
  pthread_mutexattr_t          m_lock_attr;
  unsigned                     m_msg_cnt; // message count
  unsigned                     m_cmd_cnt; // command count
  unsigned                     m_etb_cnt; // Empty This Buffer count
  unsigned                     m_ebd_cnt; // Empty Buffer Done Count

  /* Private function */
  OMX_ERRORTYPE allocate_input_buffer(OMX_HANDLETYPE             hComp,
                                      OMX_BUFFERHEADERTYPE **bufferHdr,
                                      OMX_U32                     port,
                                      OMX_PTR                  appData,
                                      OMX_U32                    bytes);
  bool allocate_done(void);
  bool release_done();
  bool execute_omx_flush(void);
  bool search_input_bufhdr(OMX_BUFFERHEADERTYPE *buffer);
  bool search_output_bufhdr(OMX_BUFFERHEADERTYPE *buffer);
};







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
#ifndef  _OMX_AAC_DEC_H_
#define  _OMX_AAC_DEC_H_
/*=========================================================================
                                Audio AAC Decoder

@file comx_aac_adec.h
This module contains the class definition for openMAX AAC decoder component.

*//*=====================================================================*/

//////////////////////////////////////////////////////////////////////////////
//                             Include Files
//////////////////////////////////////////////////////////////////////////////

#include<stdlib.h>
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

#ifdef _ANDROID_
    #include <utils/Log.h>
    #define LOG_TAG "OMX_AAC_DEC"
    #define DEBUG_PRINT_ERROR LOGE
    #define DEBUG_PRINT LOGI
    #define DEBUG_DETAIL
#else
    #define DEBUG_PRINT_ERROR printf
    #define DEBUG_PRINT printf
    #define DEBUG_DETAIL
#endif

//////////////////////////////////////////////////////////////////////////////
//                       Module specific globals
////////////////////////////////////////////////////////////////////////////
#define OMX_SPEC_VERSION  0x00000101
#define MIN(x,y) (((x) < (y)) ? (x) : (y))
#define MAX(x,y) (x >= y?x:y)

//////////////////////////////////////////////////////////////////////////////
//               Macros
////////////////////////////////////////////////////////////////////////////

#define PrintFrameHdr(bufHdr) \
                   DEBUG_PRINT("bufHdr[%x]buf[%x]size[%d]TS[%d]\n",\
                   (unsigned) bufHdr,\
                   (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->pBuffer,\
                   (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->nFilledLen,\
                   (unsigned)((OMX_BUFFERHEADERTYPE *)bufHdr)->nTimeStamp)

// BitMask Management logic

#define BITMASK_SIZE(mIndex)            (((mIndex) + BITS_PER_BYTE - 1)/BITS_PER_BYTE)
#define BITMASK_OFFSET(mIndex)          ((mIndex)/BITS_PER_BYTE)
#define BITMASK_FLAG(mIndex)            (1 << ((mIndex) % BITS_PER_BYTE))
#define BITMASK_CLEAR(mArray,mIndex)    (mArray)[BITMASK_OFFSET(mIndex)] &=  ~(BITMASK_FLAG(mIndex))
#define BITMASK_SET(mArray,mIndex)      (mArray)[BITMASK_OFFSET(mIndex)] |=  BITMASK_FLAG(mIndex)
#define BITMASK_PRESENT(mArray,mIndex)  ((mArray)[BITMASK_OFFSET(mIndex)] & BITMASK_FLAG(mIndex)  )
#define BITMASK_ABSENT(mArray,mIndex)   (((mArray)[BITMASK_OFFSET(mIndex)] & BITMASK_FLAG(mIndex)  ) == 0x0)

#define BITS_PER_BYTE                8
#define OMX_CORE_NUM_INPUT_BUFFERS   2
#define OMX_CORE_INPUT_BUFFER_SIZE   8192
#define OMX_AAC_OUTPUT_BUFFER_SIZE   9216
#define OMX_CORE_CONTROL_CMDQ_SIZE   100
#define OMX_ADEC_VOLUME_STEP         0x147
#define OMX_ADEC_MIN                 0
#define OMX_ADEC_MAX                 100
#define NON_TUNNEL                   1
#define TUNNEL                       0

/* The following defines are used to extract all data from the AAC header.
** Each is divided into
**  the byte offset into the header
**  the byte mask for the bits
**  the number of bits to right-shift to extract a 0-based value
*/
#define AAC_SAMPLING_FREQ_INDEX_SIZE          4
#define AAC_ORIGINAL_COPY_SIZE                1
#define AAC_HOME_SIZE                         1
#define AAC_COPYRIGHT_PRESENT_SIZE            1
#define AAC_PROFILE_SIZE                      2
#define AAC_BITSTREAM_TYPE_SIZE               1
#define AAC_BITRATE_SIZE                      23
#define AAC_NUM_PFE_SIZE                      4
#define AAC_BUFFER_FULLNESS_SIZE              20
#define AAC_ELEMENT_INSTANCE_TAG_SIZE         4
#define AAC_NUM_FRONT_CHANNEL_ELEMENTS_SIZE   4
#define AAC_NUM_SIDE_CHANNEL_ELEMENTS_SIZE    4
#define AAC_NUM_BACK_CHANNEL_ELEMENTS_SIZE    4
#define AAC_NUM_LFE_CHANNEL_ELEMENTS_SIZE     2
#define AAC_NUM_ASSOC_DATA_ELEMENTS_SIZE      3
#define AAC_NUM_VALID_CC_ELEMENTS_SIZE        4
#define AAC_MONO_MIXDOWN_PRESENT_SIZE         1
#define AAC_MONO_MIXDOWN_ELEMENT_SIZE         4
#define AAC_STEREO_MIXDOWN_PRESENT_SIZE       1
#define AAC_STEREO_MIXDOWN_ELEMENT_SIZE       4
#define AAC_MATRIX_MIXDOWN_PRESENT_SIZE       1
#define AAC_MATRIX_MIXDOWN_SIZE               3
#define AAC_FCE_SIZE                          5
#define AAC_SCE_SIZE                          5
#define AAC_BCE_SIZE                          5
#define AAC_LFE_SIZE                          4
#define AAC_ADE_SIZE                          4
#define AAC_VCE_SIZE                          5
#define AAC_COMMENT_FIELD_BYTES_SIZE          8
#define AAC_COMMENT_FIELD_DATA_SIZE           8

#define LOAS_GA_SPECIFIC_CONFIG(o) \
        (((o != 5) && (o >=1 ) && (o <= 7)) || \
         ((o != 18) && (o >= 17) && (o <= 23)))

#define LOAS_IS_AUD_OBJ_SUPPORTED(x) \
        ((x == 2) || (x == 4) || (x == 5) || (x == 17))

#define LOAS_IS_SFI_SUPPORTED(x) ((x >= 3) && (x <= 0x0B))

/* c is channel config and o is Audio object type */
#define LOAS_IS_CHANNEL_CONFIG_SUPPORTED(c, o) \
        (((c <= 2) && ((o == 2) || (o == 4) || (o == 5))) || \
         (((c == 1) || (c == 2)) && (o == 17)))

#define LOAS_IS_EXT_SFI_SUPPORTED(x)  ((x >= 0x03) && (x <= 0x08))

/* Extension Flag is e and Audio object type is o */
#define LOAS_IS_EXT_FLG_SUPPORTED(e,o) \
        ((((o == 2) || (o == 4) || (o == 5)) && (e == 0)) || \
         ((o == 17) && (e == 1)))

//////////////////////////////////////////////////////////
//                    OMX AAC CLASS
//////////////////////////////////////////////////////////

class omx_aac_adec: public qc_omx_component
{
public:
           omx_aac_adec();
    virtual ~omx_aac_adec();

    OMX_ERRORTYPE allocate_buffer(OMX_HANDLETYPE       hComp,
                                OMX_BUFFERHEADERTYPE **bufferHdr,
                                OMX_U32              port,
                                OMX_PTR              appData,
                                OMX_U32              bytes);

    OMX_ERRORTYPE component_deinit(OMX_HANDLETYPE hComp);

    OMX_ERRORTYPE component_init(OMX_STRING role);

    OMX_ERRORTYPE component_role_enum(OMX_HANDLETYPE hComp,
                                    OMX_U8         *role,
                                    OMX_U32        index);

    OMX_ERRORTYPE component_tunnel_request(OMX_HANDLETYPE      hComp,
                                         OMX_U32             port,
                                         OMX_HANDLETYPE      peerComponent,
                                         OMX_U32             peerPort,
                                         OMX_TUNNELSETUPTYPE *tunnelSetup);

    OMX_ERRORTYPE empty_this_buffer(OMX_HANDLETYPE       hComp,
                                    OMX_BUFFERHEADERTYPE *buffer);


    OMX_ERRORTYPE empty_this_buffer_proxy(OMX_HANDLETYPE       hComp,
                                        OMX_BUFFERHEADERTYPE *buffer);


    OMX_ERRORTYPE fill_this_buffer(OMX_HANDLETYPE       hComp,
                                   OMX_BUFFERHEADERTYPE *buffer);


    OMX_ERRORTYPE free_buffer(OMX_HANDLETYPE       hComp,
                              OMX_U32              port,
                              OMX_BUFFERHEADERTYPE *buffer);

    OMX_ERRORTYPE get_component_version(OMX_HANDLETYPE  hComp,
                                      OMX_STRING      componentName,
                                      OMX_VERSIONTYPE *componentVersion,
                                      OMX_VERSIONTYPE *specVersion,
                                      OMX_UUIDTYPE    *componentUUID);

    OMX_ERRORTYPE get_config(OMX_HANDLETYPE hComp,
                           OMX_INDEXTYPE  configIndex,
                           OMX_PTR        configData);

    OMX_ERRORTYPE get_extension_index(OMX_HANDLETYPE hComp,
                                    OMX_STRING     paramName,
                                    OMX_INDEXTYPE  *indexType);

    OMX_ERRORTYPE get_parameter(OMX_HANDLETYPE hComp,
                              OMX_INDEXTYPE  paramIndex,
                              OMX_PTR        paramData);

    OMX_ERRORTYPE get_state(OMX_HANDLETYPE hComp,
                          OMX_STATETYPE  *state);


    static void process_in_port_msg(void          *client_data,
                                  unsigned char id);

    static void process_out_port_msg(void          *client_data,
                                   unsigned char id);

    static void process_command_msg(void          *client_data,
                                   unsigned char id);

    OMX_ERRORTYPE set_callbacks(OMX_HANDLETYPE   hComp,
                              OMX_CALLBACKTYPE *callbacks,
                              OMX_PTR          appData);

    OMX_ERRORTYPE set_config(OMX_HANDLETYPE hComp,
                           OMX_INDEXTYPE configIndex,
                           OMX_PTR configData);

    OMX_ERRORTYPE set_parameter(OMX_HANDLETYPE hComp,
                              OMX_INDEXTYPE  paramIndex,
                              OMX_PTR        paramData);

    OMX_ERRORTYPE use_buffer(OMX_HANDLETYPE       hComp,
                           OMX_BUFFERHEADERTYPE **bufferHdr,
                           OMX_U32              port,
                           OMX_PTR              appData,
                           OMX_U32              bytes,
                           OMX_U8               *buffer);

    OMX_ERRORTYPE use_EGL_image(OMX_HANDLETYPE      hComp,
                             OMX_BUFFERHEADERTYPE **bufferHdr,
                             OMX_U32              port,
                             OMX_PTR              appData,
                             void *               eglImage);

private:


    ///////////////////////////////////////////////////////////
    // Type definitions
    ///////////////////////////////////////////////////////////
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
        OMX_COMPONENT_GENERATE_FRAME_DONE  = 0x05,
        OMX_COMPONENT_GENERATE_FTB         = 0x06,
        OMX_COMPONENT_GENERATE_EOS         = 0x07,
        OMX_COMPONENT_PORTSETTINGS_CHANGED = 0x08,
    };

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
        bool pop_entry(unsigned *p1,unsigned *p2, unsigned *id);
        bool get_msg_with_id(unsigned *p1,unsigned *p2, unsigned id);
    };

    enum in_format{
        FORMAT_ADTS = 0,
        FORMAT_ADIF = 1,
        FORMAT_LOAS = 2,
        FORMAT_RAW  = 3,
    };
    struct adts_fixed_header
    {
        OMX_U16 sync_word;
        OMX_U8  id;
        OMX_U8  layer;
        OMX_U8  protection_absent;
        OMX_U8  profile;
        OMX_U8  sampling_frequency_index;
        OMX_U8  private_bit;
        OMX_U8  channel_configuration;
        OMX_U8  original_copy;
        OMX_U8  home;
        OMX_U8  emphasis;
    };
    struct adts_var_header
    {
        OMX_U8  copyright_id_bit;
        OMX_U8  copyright_id_start;
        OMX_U16 aac_frame_length;
        OMX_U8  adts_buffer_fullness;
        OMX_U8  no_raw_data_blocks_in_frame;
    };
    struct adts_header
    {
        struct adts_fixed_header fixed;
        struct adts_var_header   var;
    };
    struct aac_raw
    {
        OMX_U8 aud_obj_type;
        OMX_U8 freq_index;
        OMX_U8 channel_config;
    };
    struct adif_header
    {
        OMX_U8 variable_bit_rate;
        OMX_U8 aud_obj_type;
        OMX_U8 freq_index;
        OMX_U8 channel_config;
        OMX_U32 sample_rate;
    };
    struct loas_header
    {
        OMX_U8 aud_obj_type;
        OMX_U8 freq_index;
        OMX_U8 channel_config;
    };
    struct aac_header
    {
        in_format input_format;
        union
        {
            struct adts_header adts;
            struct adif_header adif;
            struct loas_header loas;
            struct aac_raw     raw;
        }head;
    };
    typedef enum {
        AAC_CHANNEL_UNKNOWN = 0,
        AAC_CHANNEL_MONO,       /* Single channel (mono) data*/
        AAC_CHANNEL_DUAL,       /* Stereo data*/
        AAC_CHANNEL_TRIPLE,     /* 3 channels: 1+2 (UNSUPPORTED)*/
        AAC_CHANNEL_QUAD,       /* 4 channels: 1+2+1 (UNSUPPORTED)*/
        AAC_CHANNEL_QUINTUPLE,  /* 5 channels: 1+2+2 (UNSUPPORTED)*/
        AAC_CHANNEL_SEXTUPLE,   /* 5+1 channels: 1+2+2+1 (UNSUPPORTED)*/
        AAC_CHANNEL_OCTUPLE,    /* 7+1 channels: 1+2+2+2+1 (UNSUPPORTED)*/
        AAC_CHANNEL_DUAL_MONO,  /* Dual Mono: 1+1 (Two SCEs)*/
        AAC_CHANNEL_UNSUPPORTED /* Indicating CMX is currently playing*/
        /* unsupported Channel mode.*/
    } aac_channel_enum_type;

    ///////////////////////////////////////////////////////////
    // Member variables
    ///////////////////////////////////////////////////////////

    OMX_U8                         m_flush_cnt ;
    int                            nNumInputBuf;
    int                            nNumOutputBuf;
    int                            m_drv_fd;   // Kernel device node file handle
    int                            endofstream;
    int                            m_first_aac_header;

    bool                           bEOSSent;
    bool                           bOutputPortReEnabled;
    bool                           is_in_th_sleep;
    bool                           is_out_th_sleep;


    unsigned                       m_msg_cnt; // message count
    unsigned                       m_cmd_cnt; // command count
    unsigned                       m_etb_cnt; // Empty This Buffer count
    unsigned                       m_ebd_cnt; // Empty Buffer Done Count
    unsigned                       m_aac_hdr_bit_index;

    unsigned int                   frameDuration;//duration of each frame
    unsigned int                   m_inp_buf_count;//Num of Input Buffers
    unsigned int                   m_out_buf_count;//Num of Output Buffers
    unsigned int                   m_flags;//encapsulate the waiting states.

    unsigned int                   fbd_cnt;
    unsigned int                   nTimestamp;
    unsigned int                   pcm_feedback;//tunnel or non-tunnel
    unsigned int                   ntotal_playtime;
    unsigned int                   output_buffer_size;
    unsigned int                   input_buffer_size;
    unsigned int                   pSamplerate;
    unsigned int                   pChannels;
    unsigned int                   pBitrate ;
    unsigned int                   op_settings_changed ;

    volatile int                   m_is_event_done;
    volatile int                   m_is_in_th_sleep;
    volatile int                   m_is_out_th_sleep;

    input_buffer_map               m_input_buf_hdrs;
    output_buffer_map              m_output_buf_hdrs;

    omx_cmd_queue                  m_input_q;
    omx_cmd_queue                  m_input_ctrl_cmd_q;
    omx_cmd_queue                  m_input_ctrl_ebd_q;
    omx_cmd_queue                  m_command_q;
    omx_cmd_queue                  m_output_q;
    omx_cmd_queue                  m_output_ctrl_cmd_q;
    omx_cmd_queue                  m_output_ctrl_fbd_q;

    pthread_mutexattr_t            m_outputlock_attr;
    pthread_mutexattr_t            m_commandlock_attr;
    pthread_mutexattr_t            m_lock_attr;
    pthread_mutexattr_t            m_state_attr;
    pthread_mutexattr_t            m_flush_attr;
    pthread_mutexattr_t            m_in_th_attr_1;
    pthread_mutexattr_t            m_out_th_attr_1;
    pthread_mutexattr_t            m_event_attr;
    pthread_mutexattr_t            m_in_th_attr;
    pthread_mutexattr_t            m_out_th_attr;

    pthread_cond_t                 cond;
    pthread_cond_t                 in_cond;
    pthread_cond_t                 out_cond;

    pthread_mutex_t                m_lock;
    pthread_mutex_t                m_commandlock;
    pthread_mutex_t                m_outputlock;
    // Mutexes for state change
    pthread_mutex_t                m_state_lock;
    // Mutexes for  flush acks from input and output threads
    pthread_mutex_t                m_flush_lock;
    pthread_mutex_t                m_event_lock;
    pthread_mutex_t                m_in_th_lock;
    pthread_mutex_t                m_out_th_lock;
    pthread_mutex_t                m_in_th_lock_1;
    pthread_mutex_t                m_out_th_lock_1;

    OMX_PTR                        m_app_data;// Application data
    OMX_S32                        m_volume;//Unit to be determined
    //OMX_STATETYPE                nState;
    OMX_STATETYPE                  m_state;// OMX State
    OMX_CALLBACKTYPE               m_cb;// Application callbacks

    struct aac_ipc_info            *m_ipc_to_in_th;// for input thread
    struct aac_ipc_info            *m_ipc_to_out_th;// for output thread
    struct aac_ipc_info            *m_ipc_to_cmd_th;// for command thread

    OMX_AUDIO_PARAM_AACPROFILETYPE m_adec_param;//Cache aac dec param

    ///////////////////////////////////////////////////////////
    // Private methods
    ///////////////////////////////////////////////////////////
    OMX_ERRORTYPE allocate_output_buffer(OMX_HANDLETYPE       hComp,
                                         OMX_BUFFERHEADERTYPE **bufferHdr,
                                         OMX_U32 port,OMX_PTR appData,
                                         OMX_U32              bytes);

    OMX_ERRORTYPE allocate_input_buffer(OMX_HANDLETYPE       hComp,
                                        OMX_BUFFERHEADERTYPE **bufferHdr,
                                        OMX_U32              port,
                                        OMX_PTR              appData,
                                        OMX_U32              bytes);

    OMX_ERRORTYPE use_input_buffer(OMX_IN OMX_HANDLETYPE          hComp,
                                    OMX_INOUT OMX_BUFFERHEADERTYPE **bufHdr,
                                    OMX_IN OMX_U32                 port,
                                    OMX_IN OMX_PTR                 appData,
                                    OMX_IN OMX_U32                 bytes,
                                    OMX_IN OMX_U8*                 buffer);

    OMX_ERRORTYPE use_output_buffer(OMX_IN OMX_HANDLETYPE          hComp,
                                    OMX_INOUT OMX_BUFFERHEADERTYPE **bufHdr,
                                    OMX_IN OMX_U32                 port,
                                    OMX_IN OMX_PTR                 appData,
                                    OMX_IN OMX_U32                 bytes,
                                    OMX_IN OMX_U8*                 buffer);

    OMX_ERRORTYPE fill_this_buffer_proxy(OMX_HANDLETYPE       hComp,
                                         OMX_BUFFERHEADERTYPE *buffer);

    OMX_ERRORTYPE send_command_proxy(OMX_HANDLETYPE  hComp,
                                     OMX_COMMANDTYPE cmd,
                                     OMX_U32         param1,
                                     OMX_PTR         cmdData);

    OMX_ERRORTYPE send_command(OMX_HANDLETYPE hComp,
                               OMX_COMMANDTYPE  cmd,
                               OMX_U32       param1,
                               OMX_PTR      cmdData);

    OMX_ERRORTYPE  aac_frameheader_parser(OMX_BUFFERHEADERTYPE *buffer,
                                          struct aac_header    *header);
    OMX_ERRORTYPE  config_AAC();

    void audaac_extract_adif_header(OMX_U8            *data,
                                    struct aac_header *aac_header_info);

    void audaac_extract_loas_header(OMX_U8            *data,
                                    struct aac_header *aac_header_info);

    void audaac_extract_bits(OMX_U8*, OMX_U8, void*);

    bool allocate_done(void);
    bool release_done();
    bool execute_omx_flush(OMX_IN OMX_U32 param1, bool cmd_cmpl=true);
    bool execute_input_omx_flush(void);
    bool execute_output_omx_flush(void);
    bool search_input_bufhdr(OMX_BUFFERHEADERTYPE *buffer);
    bool search_output_bufhdr(OMX_BUFFERHEADERTYPE *buffer);
    bool post_input(unsigned int p1, unsigned int p2,
                    unsigned int id, bool lock=true);
    bool post_output(unsigned int p1, unsigned int p2,
                     unsigned int id, bool lock=true);
    bool post_command(unsigned int p1, unsigned int p2,
                     unsigned int id, bool lock=true);

    void buffer_done_cb(OMX_BUFFERHEADERTYPE *bufHdr);
    void frame_done_cb(OMX_BUFFERHEADERTYPE *bufHdr);
    void wait_for_event();
    void event_complete();
    void in_th_goto_sleep();
    void in_th_wakeup();
    void out_th_goto_sleep();
    void out_th_wakeup();
    void flush_ack();
};
#endif //_OMX_AAC_DEC_H_

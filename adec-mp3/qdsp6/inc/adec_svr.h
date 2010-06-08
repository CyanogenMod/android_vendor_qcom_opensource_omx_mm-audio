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
#ifndef ADEC_SVR_H
#define ADEC_SVR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pthread.h"

typedef void (*process_message_func)(void* client_data, unsigned char id);

/**
 @brief audio decoder command server structure

  This structure maintains the command
  server context

 */
struct adec_cmd_svr
{
    pthread_t thr;
    int pipe_in;
    int pipe_out;
    int dead;
    process_message_func process_msg_cb;
    void *client_data;
};

struct adec_cmd_cln
{
    pthread_t thr;
    int pipe_in;
    int pipe_out;
    int dead;
    process_message_func process_msg_output_cb;
    void *client_data;
};

/**
 @brief This function starts command server

 @param cb pointer to callback function from the client
 @param client_data reference client wants to get back
  through callback
 @return handle to command server
 */
struct adec_cmd_svr *adec_svr_start(process_message_func cb,
                                    void* client_data);

struct adec_cmd_cln *adec_cln_start(process_message_func cb,
                                    void* client_data);

/**
 @brief This function stop command server

 @param svr handle to command server
 @return none
 */
void adec_svr_stop(struct adec_cmd_svr *svr);

void adec_cln_stop(struct adec_cmd_cln *cln);

/**
 @brief This function post message in the command server

 @param svr handle to command server
 @return none
 */
void adec_svr_post_msg(struct adec_cmd_svr *svr, unsigned char id);
void adec_output_post_msg(struct adec_cmd_cln *cln, unsigned char id);

#ifdef __cplusplus
}
#endif

#endif /* ADEC_SVR */

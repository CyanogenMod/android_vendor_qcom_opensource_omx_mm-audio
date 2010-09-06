#--------------------------------------------------------------------------
#Copyright (c) 2010, Code Aurora Forum. All rights reserved.

#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of Code Aurora nor
#      the names of its contributors may be used to endorse or promote
#      products derived from this software without specific prior written
#      permission.

#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
#NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
#CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
#EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
#PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
#OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
#ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#--------------------------------------------------------------------------

ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

# ---------------------------------------------------------------------------------
#                 Common definitons
# ---------------------------------------------------------------------------------

libOmxAmrEnc-def := -g -O3
libOmxAmrEnc-def += -DQC_MODIFIED
libOmxAmrEnc-def += -D_ANDROID_
libOmxAmrEnc-def += -D_ENABLE_QC_MSG_LOG_
libOmxAmrEnc-def += -DVERBOSE
libOmxAmrEnc-def += -D_DEBUG
libOmxAmrEnc-def += -DAUDIOV2


# ---------------------------------------------------------------------------------
#             Make the Shared library (libOmxAmrEnc)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

libOmxAmrEnc-inc       := $(LOCAL_PATH)/inc
libOmxAmrEnc-inc       += $(TARGET_OUT_HEADERS)/mm-core/omxcore

LOCAL_MODULE            := libOmxAmrEnc
LOCAL_CFLAGS            := $(libOmxAmrEnc-def)
LOCAL_C_INCLUDES        := $(libOmxAmrEnc-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libutils liblog

LOCAL_SRC_FILES         := src/aenc_svr.c
LOCAL_SRC_FILES         += src/omx_amr_aenc.cpp

include $(BUILD_SHARED_LIBRARY)

# ---------------------------------------------------------------------------------
#             Make the apps-test (mm-aenc-omxamr-test)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

mm-amr-enc-test-inc    := $(LOCAL_PATH)/inc
mm-amr-enc-test-inc    += $(LOCAL_PATH)/test

mm-amr-enc-test-inc    += $(TARGET_OUT_HEADERS)/mm-core/omxcore
mm-amr-enc-test-inc    += $(TARGET_OUT_HEADERS)/mm-audio/audio-alsa 
LOCAL_MODULE            := mm-aenc-omxamr-test
LOCAL_CFLAGS            := $(libOmxAmrEnc-def)
LOCAL_C_INCLUDES        := $(mm-amr-enc-test-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libmm-omxcore
LOCAL_SHARED_LIBRARIES  += libOmxAmrEnc
LOCAL_SHARED_LIBRARIES  += libaudioalsa

LOCAL_SRC_FILES         := test/omx_amr_enc_test.c

include $(BUILD_EXECUTABLE)

endif

# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------


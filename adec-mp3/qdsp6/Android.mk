#--------------------------------------------------------------------------
#Copyright (c) 2009, Code Aurora Forum. All rights reserved.

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

libOmxMp3Dec-def := -g -O3
libOmxMp3Dec-def += -DQC_MODIFIED
libOmxMp3Dec-def += -D_ANDROID_
libOmxMp3Dec-def += -DVERBOSE
libOmxMp3Dec-def += -D_DEBUG

# ---------------------------------------------------------------------------------
#             Make the Shared library (libOmxMp3Dec)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

libOmxMp3Dec-inc        := $(LOCAL_PATH)/inc
libOmxMp3Dec-inc        += $(TARGET_OUT_HEADERS)/mm-core/omxcore

LOCAL_MODULE            := libOmxMp3Dec
LOCAL_CFLAGS            := $(libOmxMp3Dec-def)
LOCAL_C_INCLUDES        := $(libOmxMp3Dec-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libutils liblog

LOCAL_SRC_FILES         := src/adec_svr.c
LOCAL_SRC_FILES         += src/omx_mp3_adec.cpp

include $(BUILD_SHARED_LIBRARY)

# ---------------------------------------------------------------------------------
#             Make the apps-test (mm-adec-omxmp3-test)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

mm-mp3-dec-test-inc        := $(LOCAL_PATH)/inc
mm-mp3-dec-test-inc        += $(LOCAL_PATH)/test
mm-mp3-dec-test-inc        += $(TARGET_OUT_HEADERS)/mm-core/omxcore

LOCAL_MODULE               := mm-adec-omxmp3-test
LOCAL_CFLAGS               := $(libOmxMp3Dec-def)
LOCAL_C_INCLUDES           := $(mm-mp3-dec-test-inc)
LOCAL_PRELINK_MODULE       := false
LOCAL_SHARED_LIBRARIES     := libmm-omxcore
LOCAL_SHARED_LIBRARIES     += libOmxMp3Dec

LOCAL_SRC_FILES            := test/omx_mp3_dec_test.c

include $(BUILD_EXECUTABLE)

endif #BUILD_TINY_ANDROID

# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------


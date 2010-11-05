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
libOmxMp3Dec-def += -D_ENABLE_QC_MSG_LOG_
libOmxMp3Dec-def += -DVERBOSE
libOmxMp3Dec-def += -D_DEBUG
libOmxMp3Dec-def += -DAUDIOV2

ifeq ($(BOARD_USES_QCOM_AUDIO_V2), true)
libOmxMp3Dec-def += -DAUDIOV2
endif

# ---------------------------------------------------------------------------------
#             Make the apps-test (mm-adec-omxmp3-test)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

ifeq ($(BOARD_USES_QCOM_AUDIO_V2), true)
mm-mp3-dec-test-inc   += $(TARGET_OUT_HEADERS)/mm-audio/audio-alsa
mm-mp3-dec-test-inc   += $(TARGET_OUT_HEADERS)/mm-core/omxcore
mm-mp3-dec-test-inc   += $(PV_TOP)/codecs_v2/omx/omx_mastercore/include \
        		 $(PV_TOP)/codecs_v2/omx/omx_common/include \
        		 $(PV_TOP)/extern_libs_v2/khronos/openmax/include \
        		 $(PV_TOP)/codecs_v2/omx/omx_baseclass/include \
        		 $(PV_TOP)/codecs_v2/omx/omx_mp3/include \
        		 $(PV_TOP)/codecs_v2/audio/mp3/dec/include \

LOCAL_MODULE            := sw-adec-omxmp3-test
LOCAL_CFLAGS            := $(libOmxMp3Dec-def)
LOCAL_C_INCLUDES        := $(mm-mp3-dec-test-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libopencore_common
LOCAL_SHARED_LIBRARIES  += libomx_sharedlibrary
LOCAL_SHARED_LIBRARIES  += libomx_mp3dec_sharedlibrary
LOCAL_SHARED_LIBRARIES  += libaudioalsa

LOCAL_SRC_FILES         := test/omx_mp3_dec_test.c

include $(BUILD_EXECUTABLE)
endif

endif #BUILD_TINY_ANDROID

# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------

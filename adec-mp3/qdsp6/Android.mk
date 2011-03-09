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
LOCAL_MODULE_TAGS       := optional
LOCAL_CFLAGS            := $(libOmxMp3Dec-def)
LOCAL_C_INCLUDES        := $(libOmxMp3Dec-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libutils liblog

LOCAL_SRC_FILES         := src/adec_svr.c
LOCAL_SRC_FILES         += src/omx_mp3_adec.cpp

ifeq "$(findstring qsd8250,$(QCOM_TARGET_PRODUCT))" "qsd8250"
include $(BUILD_SHARED_LIBRARY)
endif
# ---------------------------------------------------------------------------------
#             Make the apps-test (mm-adec-omxmp3-test)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

mm-mp3-dec-test-inc        := $(LOCAL_PATH)/inc
mm-mp3-dec-test-inc        += $(LOCAL_PATH)/test
mm-mp3-dec-test-inc        += $(TARGET_OUT_HEADERS)/mm-core/omxcore

LOCAL_MODULE               := mm-adec-omxmp3-test
LOCAL_MODULE_TAGS          := optional
LOCAL_CFLAGS               := $(libOmxMp3Dec-def)
LOCAL_C_INCLUDES           := $(mm-mp3-dec-test-inc)
LOCAL_PRELINK_MODULE       := false
LOCAL_SHARED_LIBRARIES     := libmm-omxcore
LOCAL_SHARED_LIBRARIES     += libOmxMp3Dec

LOCAL_SRC_FILES            := test/omx_mp3_dec_test.c

ifeq "$(findstring qsd8250,$(QCOM_TARGET_PRODUCT))" "qsd8250"
include $(BUILD_EXECUTABLE)
endif

endif #BUILD_TINY_ANDROID

# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------


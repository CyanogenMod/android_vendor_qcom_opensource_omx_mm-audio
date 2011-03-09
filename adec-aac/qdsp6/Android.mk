ifneq ($(BUILD_TINY_ANDROID),true)

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

# ---------------------------------------------------------------------------------
#                 Common definitons
# ---------------------------------------------------------------------------------

libOmxAacDec-def :=  -g -O3
libOmxAacDec-def +=  -DQC_MODIFIED
libOmxAacDec-def +=  -D_ANDROID_
libOmxAacDec-def +=  -DVERBOSE
libOmxAacDec-def +=  -D_DEBUG

# ---------------------------------------------------------------------------------
#             Make the Shared library (libOmxAacDec)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

libOmxAacDec-inc        := $(LOCAL_PATH)/inc
libOmxAacDec-inc        += $(TARGET_OUT_HEADERS)/mm-core/omxcore


LOCAL_MODULE            := libOmxAacDec
LOCAL_MODULE_TAGS       := optional
LOCAL_CFLAGS            := $(libOmxAacDec-def)
LOCAL_C_INCLUDES        := $(libOmxAacDec-inc)
LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := libutils liblog

LOCAL_SRC_FILES         := src/adec_svr.c
LOCAL_SRC_FILES         += src/omx_aac_adec.cpp

ifeq "$(findstring qsd8250,$(QCOM_TARGET_PRODUCT))" "qsd8250"
include $(BUILD_SHARED_LIBRARY)
endif

# ---------------------------------------------------------------------------------
#             Make the apps-test (mm-adec-omxaac-test)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)

mm-aac-dec-test-inc        := $(LOCAL_PATH)/inc
mm-aac-dec-test-inc        += $(LOCAL_PATH)/test
mm-aac-dec-test-inc        += $(TARGET_OUT_HEADERS)/mm-core/omxcore

LOCAL_MODULE               := mm-adec-omxaac-test
LOCAL_MODULE_TAGS          := optional
LOCAL_CFLAGS               := $(libOmxAacDec-def)
LOCAL_C_INCLUDES           := $(mm-aac-dec-test-inc)
LOCAL_PRELINK_MODULE       := false
LOCAL_SHARED_LIBRARIES     := libmm-omxcore
LOCAL_SHARED_LIBRARIES     += libOmxAacDec

LOCAL_SRC_FILES            := test/omx_aac_dec_test.c

ifeq "$(findstring qsd8250,$(QCOM_TARGET_PRODUCT))" "qsd8250"
include $(BUILD_EXECUTABLE)
endif

endif #BUILD_TINY_ANDROID

# ---------------------------------------------------------------------------------
#                     END
# ---------------------------------------------------------------------------------


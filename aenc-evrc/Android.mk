ifeq ($(TARGET_ARCH),arm)


AENC_EVRC_PATH:= $(call my-dir)

ifeq "$(findstring msm8660,$(QCOM_TARGET_PRODUCT))" "msm8660"
include $(AENC_EVRC_PATH)/qdsp6/Android.mk
endif

endif

ifeq ($(TARGET_ARCH),arm)


AENC_AMR_PATH:= $(call my-dir)

ifeq "$(findstring msm8660,$(QCOM_TARGET_PRODUCT))" "msm8660"
include $(AENC_AMR_PATH)/qdsp6/Android.mk
endif

endif

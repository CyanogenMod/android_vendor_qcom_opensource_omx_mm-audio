ifneq ($(call is-android-codename,ICECREAM_SANDWICH),true)
include $(call all-subdir-makefiles)
endif

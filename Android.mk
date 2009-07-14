#
# brcm_patchram
#

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	hci_test.c
	

LOCAL_C_INCLUDES :=  \
	$(LOCAL_PATH)/include

LOCAL_SHARED_LIBRARIES := \
	libcutils

LOCAL_MODULE := hcitest

include $(BUILD_EXECUTABLE)

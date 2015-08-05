LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:= vidtest.cpp

LOCAL_MODULE:= vidtest

LOCAL_C_INCLUDES := $(KERNEL_HEADERS)
LOCAL_C_INCLUDES += $(LOCAL_PATH) device/fsl-proprietary/include/

LOCAL_CFLAGS :=

LOCAL_SHARED_LIBRARIES := libcutils \
                          libutils \
                          libg2d \
                          libbinder \
                          libgui \
                          libui \
                          libstagefright_foundation \
                          libstagefright
LOCAL_MODULE_TAGS := eng
include $(BUILD_EXECUTABLE)

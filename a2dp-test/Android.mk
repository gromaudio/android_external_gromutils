LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE            := a2dp_test
LOCAL_MODULE_TAGS       := eng
LOCAL_SRC_FILES         := a2dp_test.cpp
LOCAL_SHARED_LIBRARIES  := libcutils \
                           libutils \
                           libmedia

include $(BUILD_EXECUTABLE)

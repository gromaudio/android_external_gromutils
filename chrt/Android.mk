LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES     := chrt.c \
                       procutils.c
LOCAL_MODULE        := chrt
LOCAL_MODULE_TAGS   := eng
include $(BUILD_EXECUTABLE)

LOCAL_PATH:= $(call my-dir)

ifeq ($(TARGET_BOARD_PLATFORM),imx6)
  include $(CLEAR_VARS)
  LOCAL_SRC_FILES:= camtest.c
  LOCAL_MODULE:= camtest
  LOCAL_C_INCLUDES := $(KERNEL_HEADERS)
  LOCAL_CFLAGS :=
  LOCAL_SHARED_LIBRARIES := libcutils
  LOCAL_MODULE_TAGS := eng
  include $(BUILD_EXECUTABLE)
endif

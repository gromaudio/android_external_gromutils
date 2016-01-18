LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(TARGET_BOARD_PLATFORM),imx6)
  LOCAL_SRC_FILES:= vidtest_imx6.cpp
endif

ifeq ($(TARGET_BOARD_PLATFORM),rk312x)
  LOCAL_SRC_FILES:= vidtest_rk3128.cpp
endif


LOCAL_MODULE:= vidtest

LOCAL_C_INCLUDES := $(KERNEL_HEADERS)

LOCAL_CFLAGS := -O3 -Wall -Wno-unused-parameter

LOCAL_SHARED_LIBRARIES := libcutils \
                          libutils \
                          libbinder \
                          libgui \
                          libui \
                          libstagefright_foundation \
                          libstagefright
ifeq ($(TARGET_BOARD_PLATFORM),imx6)
  LOCAL_C_INCLUDES       += $(LOCAL_PATH) device/fsl-proprietary/include/
  LOCAL_SHARED_LIBRARIES += libg2d
  LOCAL_CFLAGS           += -DTARGET_PLATFORM_IMX6
endif

LOCAL_MODULE_TAGS := eng
include $(BUILD_EXECUTABLE)

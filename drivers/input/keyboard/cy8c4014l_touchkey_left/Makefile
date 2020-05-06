# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
obj-$(CONFIG_NUBIA_CYPRESS_KEYBOARD_LEFT_GAMEFIRE)           += cy8c4014l_touchkey_firmware_update.o
obj-$(CONFIG_NUBIA_CYPRESS_KEYBOARD_LEFT_GAMEFIRE)           += cybtldr_touchkey_command.o
obj-$(CONFIG_NUBIA_CYPRESS_KEYBOARD_LEFT_GAMEFIRE)           += cy8c4014l_touchkey.o

#nubia: nubia 629 v1s
ifeq ($(TARGET_NUBIA_NX629J_V1S), true)
    ccflags-y += -DTARGET_NUBIA_NX629J_V1S
endif

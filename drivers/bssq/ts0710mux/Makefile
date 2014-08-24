#LGE_KERNEL_MUX START
EXTRA_CFLAGS += -DLGE_KERNEL_MUX

ifeq (, $(filter su880 ku8800 , $(HW_REV)))
obj-y += ts0710_mux_ifx.o 
else
obj-y += ts0710_mux.o 
endif

#LGE_KERNEL_MUX END


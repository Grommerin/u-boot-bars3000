

include $(TOPDIR)/config.mk

LIB    = $(obj)lib$(BOARD).o

COBJS  := mx6q_bars3000.o

SRCS   := $(COBJS:.o=.c)
OBJS   := $(addprefix $(obj),$(COBJS))

$(LIB):        $(obj).depend $(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################

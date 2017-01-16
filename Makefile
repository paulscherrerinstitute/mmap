TOP=.

include $(TOP)/configure/CONFIG

mmap_DBD += base.dbd

# library
LIBRARY = mmap

DBDS = mmap.dbd

LIB_SRCS += mmapDrv.c
mmap_DBD += mmapDrv.dbd

LIB_LIBS += regDev
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

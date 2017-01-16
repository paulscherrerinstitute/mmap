ifeq ($(wildcard /ioc/tools/driver.makefile),)
$(warning It seems you do not have the PSI build environment. Remove GNUmakefile.)
include Makefile
else
include /ioc/tools/driver.makefile

BUILDCLASSES += Linux
INCLUDES += -I../include/$(T_A) -I../include/$(OS_CLASS) -I../include

endif

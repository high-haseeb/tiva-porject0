PART=TM4C123GH6PM

#
# The base directory for TivaWare.
#
ROOT=../tivaware

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=../drivers
VPATH+=../tivaware/utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=../tivaware
IPATH+=../../../..

#
# The default rule, which causes the Project Zero Example to be built.
#
all: ${COMPILER}
all: ${COMPILER}/project0.axf


LM4FLASH = lm4flash
BIN_FILE = gcc/project0.bin
flash: all
	$(LM4FLASH) $(BIN_FILE)

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the Project Zero Example.
#
${COMPILER}/project0.axf: ${COMPILER}/project0.o
${COMPILER}/project0.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/project0.axf: ${COMPILER}/uartstdio.o
${COMPILER}/project0.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/project0.axf: project0.ld
SCATTERgcc_project0=project0.ld
ENTRY_project0=ResetISR
CFLAGSgcc=-DTARGET_IS_TM4C123_RB1

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

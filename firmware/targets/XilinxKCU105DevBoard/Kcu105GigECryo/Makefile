# Define target output
target: prom

# Define Firmware Version Number
export PRJ_VERSION = 0x00000008

# Define target part
export PRJ_PART = XCKU040-FFVA1156-2-E

# # Define the Microblaze source path
# export SDK_SRC_PATH = $(PROJ_DIR)/../../../common/core/src

# Whether to use the on-board PHY (connected to the RJ-45 jack)
# or an SFP module for ethernet connectivity.
# Must be 1 or 0
export USE_RJ45_ETH = 0

# Using a non-standard target directory structure, 
# which requires me to define the TOP_DIR path
export TOP_DIR = $(abspath $(PWD)/../../..)

# Required for some imports of ePix CRYO stuff? 
export COMM_TYPE = pgp2b
export COMMON_NAME = cryo
export BUILD_MIG_CORE = 0
export BUILD_MB_CORE = 0
export PROM_FSBL = 0

# Use top level makefile
include $(TOP_DIR)/submodules/ruckus/system_vivado.mk

# Makefile

VERILOG_PATH = ../../src

# --- Simulation Variables ---
TOPLEVEL_LANG ?= verilog
VERILOG_SOURCES = $(VERILOG_PATH)/spi_master.v tb_spi.v
TOPLEVEL = tb_spi
MODULE = test_spi_master

# --- Cocotb Integration ---
include $(shell cocotb-config --makefiles)/Makefile.sim

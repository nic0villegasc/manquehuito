# Makefile for Cocotb test of the manquehuito_domain

# Set the toplevel language
TOPLEVEL_LANG ?= verilog

# Add all Verilog source files needed for the simulation.
# This includes the DUT (manquehuito_domain), all its submodules,
# and the testbench wrapper.
# Paths are assumed to be relative to this Makefile's location.
VERILOG_SOURCES = ../../src/manquehuito_domain.sv \
                  ../../src/control_unit.sv \
                  ../../src/spi_master.sv \
                  ../../src/pc.sv \
                  ../../src/register.sv \
                  ../../src/alu.sv \
                  ../../src/status.sv \
                  ../../src/muxA.sv \
                  ../../src/muxB.sv \
                  ../../src/muxD.sv \
									../../src/ps_pwm_wrapper.v \
									../../src/dead_time_generator.v \
									../../src/comparator.v \
									../../src/signal_generator_0phase.v \
									../../src/signal_generator_180phase.v \
                  tb_manquehuito.sv

# The name of the top-level module in your testbench
TOPLEVEL = tb

# The name of the Python test file (without the .py extension)
MODULE = test_manquehuito

# Include the standard Cocotb simulation makefile
include $(shell cocotb-config --makefiles)/Makefile.sim

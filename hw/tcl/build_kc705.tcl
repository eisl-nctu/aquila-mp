#*****************************************************************************************
#  Create the aquial_ap workspace.      Chun-Jen Tsai Sep/03/2025
# ----------------------------------------------------------------------------------------
#  Note that the source files under ./src are not copied into the workspace.
#  Any modifications of the source files from the Vivado GUI will be made directly
#  to the files in the ./src directory.
#*****************************************************************************************

# Set workspace parameters.
set proj_name   "aquila_mp_kc705"
set fpga_model  xc7k325tffg900-2
set board_model KC705
set xdc_file    kc705.xdc
set mig_file    mig-kc705.prj

# Processing command-line parameters
#
if { $argc == 1 } {
  set args [lindex $argv 0]
} else {
  set args "NONE"
}

# Create an empty project workspace of an FPGA model.
create_project $proj_name ./$proj_name -part $fpga_model

# Set the source tree root directory.
set source_dir "./src"

# Set project properties
set obj [current_project]
set_property -name "default_lib" -value "xil_defaultlib" -objects $obj
set_property -name "enable_resource_estimation" -value "0" -objects $obj
set_property -name "sim.ip.auto_export_scripts" -value "1" -objects $obj
set_property -name "simulator_language" -value "Mixed" -objects $obj
set_property -name "xpm_libraries" -value "XPM_CDC XPM_MEMORY" -objects $obj

# Start reading source files from the source directory.
read_verilog $source_dir/core_rtl/alu.v
read_verilog $source_dir/core_rtl/aquila_config.vh
read_verilog $source_dir/core_rtl/aquila_top.v
read_verilog $source_dir/core_rtl/atomic_unit.v
read_verilog $source_dir/core_rtl/bcu.v
read_verilog $source_dir/core_rtl/bpu.v
read_verilog $source_dir/core_rtl/bram_dp.v
read_verilog $source_dir/core_rtl/core_top.v
read_verilog $source_dir/core_rtl/csr_file.v
read_verilog $source_dir/core_rtl/dcache.v
read_verilog $source_dir/core_rtl/decode.v
read_verilog $source_dir/core_rtl/distri_ram.v
read_verilog $source_dir/core_rtl/distri_ram_dp.v
read_verilog $source_dir/core_rtl/execute.v
read_verilog $source_dir/core_rtl/fetch.v
read_verilog $source_dir/core_rtl/forwarding_unit.v
read_verilog $source_dir/core_rtl/icache.v
read_verilog $source_dir/core_rtl/memory.v
read_verilog $source_dir/core_rtl/muldiv.v
read_verilog $source_dir/core_rtl/pipeline_control.v
read_verilog $source_dir/core_rtl/program_counter.v
read_verilog $source_dir/core_rtl/reg_file.v
read_verilog $source_dir/core_rtl/sram.v
read_verilog $source_dir/core_rtl/sram_dp.v
read_verilog $source_dir/core_rtl/writeback.v

read_verilog $source_dir/soc_rtl/clint.sv
read_verilog $source_dir/soc_rtl/uart.v
read_verilog $source_dir/soc_rtl/coherence_unit.sv
read_verilog $source_dir/soc_rtl/core2axi_if.v
read_verilog $source_dir/soc_rtl/L2cache.v

read_verilog $source_dir/soc_rtl/arbiter/amo_arbiter.sv
read_verilog $source_dir/soc_rtl/arbiter/bus_arbiter.sv
read_verilog $source_dir/soc_rtl/mem_port/cdc_sync.v
read_verilog $source_dir/soc_rtl/mem_port/mem_arbiter.v

# Read synthesis-only files.
add_files -fileset sources_1 -norecurse $source_dir/soc_rtl/soc_top.sv

# Read memory and constraints files.
read_mem $source_dir/mem/uartboot.mem
read_xdc $source_dir/xdc/$xdc_file

# Define the target board parameters for synthesis.
set_property verilog_define [list $board_model] [get_filesets sources_1]

# Set top-level module for synthesis.
set_property top soc_top [get_filesets sources_1]

# Read simulation-only files.
add_files -fileset sim_1 -norecurse $source_dir/soc_rtl/soc_tb.sv
add_files -fileset sim_1 -norecurse $source_dir/soc_rtl/mem_port/mig_7series_sim.v

# Define the target board parameters for simulation.
set_property verilog_define [list $board_model] [get_filesets sim_1]

# Set top-level module for simulation.
set_property top soc_tb [get_filesets sim_1]
set_property -name "top_lib" -value "xil_defaultlib" -objects [get_filesets sim_1]

# ===========================================================================================
#  The following commands adds soft-core IP into the workspace.
# ===========================================================================================

# Adding an Asynchronous FIFO Addr IP
create_ip -name fifo_generator -vendor xilinx.com -library ip -module_name async_fifo_addr
set_property -dict [list \
    CONFIG.fifo_implementation {independent_clocks_distributed_ram} \
    CONFIG.synchronization_stages {3} \
    CONFIG.input_data_width {32} \
    CONFIG.input_depth {16} \
    CONFIG.reset_pin {false} \
    CONFIG.performance_options {first_word_fall_through} ] [get_ips async_fifo_addr]
generate_target all [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/async_fifo_addr/async_fifo_addr.xci]

# Adding an Asynchronous FIFO Data IP
create_ip -name fifo_generator -vendor xilinx.com -library ip -module_name async_fifo_data
set_property -dict [list \
    CONFIG.fifo_implementation {independent_clocks_distributed_ram} \
    CONFIG.synchronization_stages {3} \
    CONFIG.input_data_width {256} \
    CONFIG.input_depth {16} \
    CONFIG.reset_pin {false} \
    CONFIG.performance_options {first_word_fall_through} ] [get_ips async_fifo_data]
generate_target all [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/async_fifo_data/async_fifo_data.xci]

# Adding an Asynchronous FIFO Signal IP
create_ip -name fifo_generator -vendor xilinx.com -library ip -module_name async_fifo_signal
set_property -dict [list \
    CONFIG.fifo_implementation {independent_clocks_distributed_ram} \
    CONFIG.synchronization_stages {3} \
    CONFIG.input_data_width {1} \
    CONFIG.input_depth {16} \
    CONFIG.reset_pin {false} \
    CONFIG.performance_options {first_word_fall_through} ] [get_ips async_fifo_signal]
generate_target all [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/async_fifo_signal/async_fifo_signal.xci]

# Adding a Clock Wizard IP
# For some platforms, you may have to set CONFIG.PRIM_SOURCE {No_buffer}
create_ip -name clk_wiz -vendor xilinx.com -library ip -module_name clk_wiz_0
set_property -dict [list \
    CONFIG.PRIM_SOURCE {No_buffer} \
    CONFIG.PRIM_IN_FREQ {200.00000} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {50.00000} \
    CONFIG.CLKOUT2_USED {false} \
    CONFIG.USE_RESET {false} \
    CONFIG.USE_LOCKED {false}] [get_ips clk_wiz_0]
generate_target all [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci]

# Adding an SPI IP
create_ip -name axi_quad_spi -vendor xilinx.com -library ip -module_name axi_quad_spi_0
set_property -dict [ list \
   CONFIG.C_FIFO_DEPTH {256} \
   CONFIG.C_SCK_RATIO {4} \
   CONFIG.C_USE_STARTUP {0} \
   CONFIG.C_USE_STARTUP_INT {0} \
   CONFIG.QSPI_BOARD_INTERFACE {custom}] [get_ips axi_quad_spi_0]
generate_target all [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/axi_quad_spi_0/axi_quad_spi_0.xci]

# Adding an MIG IP
create_ip -name mig_7series -vendor xilinx.com -library ip -module_name mig_7series_0
file copy $source_dir/mig/$mig_file ${proj_name}/${proj_name}.srcs/sources_1/ip/mig_7series_0/mig.prj
set_property -dict [list CONFIG.XML_INPUT_FILE {mig.prj}] [get_ips mig_7series_0]
generate_target {instantiation_template} [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/mig_7series_0/mig_7series_0.xci]
generate_target all [get_files ${proj_name}/${proj_name}.srcs/sources_1/ip/mig_7series_0/mig_7series_0.xci]

# ===========================================================================================
#  End of wrokspace creation.
# ===========================================================================================
puts "INFO: Project created:${proj_name}"

# Open current project with Vivado GUI
start_gui

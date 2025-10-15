################################################################################
# Constraint file for the Digilent Genesys II development board
################################################################################
# Compress bitstream and configure for fast Flash programming
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.CONFIG.SPI_32BIT_ADDR YES [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]

## Clock Signal
set_property -dict {PACKAGE_PIN AD12 IOSTANDARD LVDS} [get_ports sys_clk_p]
set_property -dict {PACKAGE_PIN AD11 IOSTANDARD LVDS} [get_ports sys_clk_n]
#create_clock -name sys_clk -period 5 [get_ports sys_clk_p]

################################################################################
## Buttons
set_property -dict {PACKAGE_PIN R19 IOSTANDARD LVCMOS33} [get_ports resetn_i]

set_property -dict {PACKAGE_PIN M20 IOSTANDARD LVCMOS33} [get_ports {usr_btn[0]}]; # EAST
set_property -dict {PACKAGE_PIN B19 IOSTANDARD LVCMOS33} [get_ports {usr_btn[1]}]; # NORTH
set_property -dict {PACKAGE_PIN M19 IOSTANDARD LVCMOS33} [get_ports {usr_btn[2]}]; # SOUTH
set_property -dict {PACKAGE_PIN C19 IOSTANDARD LVCMOS33} [get_ports {usr_btn[3]}]; # WEST

## UART
set_property -dict {PACKAGE_PIN Y23 IOSTANDARD LVCMOS33} [get_ports uart_tx]
set_property -dict {PACKAGE_PIN Y20 IOSTANDARD LVCMOS33} [get_ports uart_rx]

## LEDs
set_property -dict {PACKAGE_PIN U29 IOSTANDARD LVCMOS33} [get_ports {usr_led[0]}]
set_property -dict {PACKAGE_PIN U30 IOSTANDARD LVCMOS33} [get_ports {usr_led[1]}]
set_property -dict {PACKAGE_PIN V19 IOSTANDARD LVCMOS33} [get_ports {usr_led[2]}]
set_property -dict {PACKAGE_PIN T28 IOSTANDARD LVCMOS33} [get_ports {usr_led[3]}]

## Switches
#set_property -dict {PACKAGE_PIN K19 IOSTANDARD LVCMOS12} [get_ports {usr_sw[0]}]
#set_property -dict {PACKAGE_PIN H24 IOSTANDARD LVCMOS12} [get_ports {usr_sw[1]}]
#set_property -dict {PACKAGE_PIN G25 IOSTANDARD LVCMOS12} [get_ports {usr_sw[2]}]
#set_property -dict {PACKAGE_PIN G19 IOSTANDARD LVCMOS12} [get_ports {usr_sw[3]}]

## SD Card
# ---- SPI mode ----
set_property -dict {PACKAGE_PIN R26 IOSTANDARD LVCMOS33} [get_ports spi_miso]; #IO_L10N_T1_D15_14 Sch=sd_dat[0]
set_property -dict {PACKAGE_PIN R28 IOSTANDARD LVCMOS33} [get_ports spi_sck]; #IO_L11P_T1_SRCC_14 Sch=sd_sclk
set_property -dict {PACKAGE_PIN R29 IOSTANDARD LVCMOS33} [get_ports spi_mosi]; #IO_L7N_T1_D10_14 Sch=sd_cmd
set_property -dict {PACKAGE_PIN T30 IOSTANDARD LVCMOS33} [get_ports spi_ss]; #IO_L9N_T1_DQS_D13_14 Sch=sd_dat[3]
# ---- SDIO mode ----
#set_property -dict { PACKAGE_PIN P28   IOSTANDARD LVCMOS33 } [get_ports { sd_cd_tri_io[0] }]; #IO_L8N_T1_D12_14 Sch=sd_cd
#set_property -dict { PACKAGE_PIN R29   IOSTANDARD LVCMOS33 } [get_ports { sd_cmd_tri_io }]; #IO_L7N_T1_D10_14 Sch=sd_cmd
#set_property -dict { PACKAGE_PIN R26   IOSTANDARD LVCMOS33 } [get_ports { sd_dat0_i }]; #IO_L10N_T1_D15_14 Sch=sd_dat[0]
#set_property -dict { PACKAGE_PIN R30   IOSTANDARD LVCMOS33 } [get_ports { sd_dat1_i }]; #IO_L9P_T1_DQS_14 Sch=sd_dat[1]
#set_property -dict { PACKAGE_PIN P29   IOSTANDARD LVCMOS33 } [get_ports { sd_dat2_i }]; #IO_L7P_T1_D09_14 Sch=sd_dat[2]
#set_property -dict { PACKAGE_PIN T30   IOSTANDARD LVCMOS33 } [get_ports { sd_dat3_i }]; #IO_L9N_T1_DQS_D13_14 Sch=sd_dat[3]
#set_property -dict { PACKAGE_PIN AE24  IOSTANDARD LVCMOS33 } [get_ports { sd_reset[0] }]; #IO_L12N_T1_MRCC_12 Sch=sd_reset
#set_property -dict { PACKAGE_PIN R28   IOSTANDARD LVCMOS33 } [get_ports { sd_sck_o }]; #IO_L11P_T1_SRCC_14 Sch=sd_sclk

## HDMI TMDS
# TMDS signals
#set_property PACKAGE_PIN AB20 [get_ports hdmi_tx_clk_n]
#set_property IOSTANDARD TMDS_33 [get_ports hdmi_tx_clk_n]
#set_property PACKAGE_PIN AA20 [get_ports hdmi_tx_clk_p]
#set_property IOSTANDARD TMDS_33 [get_ports hdmi_tx_clk_p]
#set_property PACKAGE_PIN AC21 [get_ports {hdmi_tx_data_n[0]}]
#set_property IOSTANDARD TMDS_33 [get_ports {hdmi_tx_data_n[0]}]
#set_property PACKAGE_PIN AC20 [get_ports {hdmi_tx_data_p[0]}]
#set_property IOSTANDARD TMDS_33 [get_ports {hdmi_tx_data_p[0]}]
#set_property PACKAGE_PIN AA23 [get_ports {hdmi_tx_data_n[1]}]
#set_property IOSTANDARD TMDS_33 [get_ports {hdmi_tx_data_n[1]}]
#set_property PACKAGE_PIN AA22 [get_ports {hdmi_tx_data_p[1]}]
#set_property IOSTANDARD TMDS_33 [get_ports {hdmi_tx_data_p[1]}]
#set_property PACKAGE_PIN AC25 [get_ports {hdmi_tx_data_n[2]}]
#set_property IOSTANDARD TMDS_33 [get_ports {hdmi_tx_data_n[2]}]
#set_property PACKAGE_PIN AB24 [get_ports {hdmi_tx_data_p[2]}]
#set_property IOSTANDARD TMDS_33 [get_ports {hdmi_tx_data_p[2]}]

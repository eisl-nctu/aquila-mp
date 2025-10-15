################################################################################
# Constraint file for the QMTech Core Board development board
################################################################################

# Compress bitstream and configure for fast Flash programming
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]

## Clock Signal
set_property -dict {PACKAGE_PIN F22 IOSTANDARD LVCMOS33} [get_ports {sys_clk_i}];
#create_clock -name sys_clk -period 20 [get_ports sys_clk_i]

################################################################################
## Buttons
set_property -dict {PACKAGE_PIN AF10 IOSTANDARD LVCMOS18} [get_ports {usr_btn[0]}];
set_property -dict {PACKAGE_PIN AF8 IOSTANDARD LVCMOS18}  [get_ports {usr_btn[1]}]; # dummy
set_property -dict {PACKAGE_PIN AF13 IOSTANDARD LVCMOS18} [get_ports {usr_btn[2]}]; # dummy
set_property -dict {PACKAGE_PIN AE13 IOSTANDARD LVCMOS18} [get_ports {usr_btn[3]}]; # dummy

## UART
set_property -dict {PACKAGE_PIN AD21 IOSTANDARD LVCMOS33} [get_ports {uart_rx}];  # old: N26
set_property -dict {PACKAGE_PIN AE21 IOSTANDARD LVCMOS33} [get_ports {uart_tx}];  # old: P23

## LEDs
set_property -dict {PACKAGE_PIN J26 IOSTANDARD LVCMOS33} [get_ports {usr_led[0]}];
set_property -dict {PACKAGE_PIN H26 IOSTANDARD LVCMOS33} [get_ports {usr_led[1]}];
set_property -dict {PACKAGE_PIN H21 IOSTANDARD LVCMOS33} [get_ports {usr_led[2]}]; # dummy
set_property -dict {PACKAGE_PIN G21 IOSTANDARD LVCMOS33} [get_ports {usr_led[3]}]; # dummy

## SD Card (SPI)
#set_property -dict { PACKAGE_PIN AF22 IOSTANDARD LVCMOS33 } [get_ports { sdio_d01 }]; # old: W21
set_property -dict { PACKAGE_PIN AE22 IOSTANDARD LVCMOS33 } [get_ports { spi_miso }]; # sdio_d00, old: V21
set_property -dict { PACKAGE_PIN AF23 IOSTANDARD LVCMOS33 } [get_ports { spi_sck }];  # sdio_clk, old: AE23
set_property -dict { PACKAGE_PIN AE23 IOSTANDARD LVCMOS33 } [get_ports { spi_mosi }]; # sdio_cmd, old: AE22
set_property -dict { PACKAGE_PIN  W21 IOSTANDARD LVCMOS33 } [get_ports { spi_ss }];   # sdio_d03, old: AD21
#set_property -dict { PACKAGE_PIN  V21 IOSTANDARD LVCMOS33 } [get_ports { sdio_d02 }]; # old: AF23

## I2C
#set_property -dict { PACKAGE_PIN Y22 IOSTANDARD LVCMOS33 } [get_ports { i2c_scl }];
#set_property -dict { PACKAGE_PIN AA22 IOSTANDARD LVCMOS33 } [get_ports { i2c_sda }];

## Qmcore HDMI
#set_property -dict { PACKAGE_PIN E25 IOSTANDARD TMDS_33 } [get_ports { tmds_clk_p     }];
#set_property -dict { PACKAGE_PIN D25 IOSTANDARD TMDS_33 } [get_ports { tmds_clk_n     }];
#set_property -dict { PACKAGE_PIN F25 IOSTANDARD TMDS_33 } [get_ports { tmds_data_p[0] }];
#set_property -dict { PACKAGE_PIN E26 IOSTANDARD TMDS_33 } [get_ports { tmds_data_n[0] }];
#set_property -dict { PACKAGE_PIN B25 IOSTANDARD TMDS_33 } [get_ports { tmds_data_p[1] }];
#set_property -dict { PACKAGE_PIN B26 IOSTANDARD TMDS_33 } [get_ports { tmds_data_n[1] }];
#set_property -dict { PACKAGE_PIN D26 IOSTANDARD TMDS_33 } [get_ports { tmds_data_p[2] }];
#set_property -dict { PACKAGE_PIN C26 IOSTANDARD TMDS_33 } [get_ports { tmds_data_n[2] }];

## This file is a general .xdc for the Zybo Z7 Rev. B
## It is compatible with the Zybo Z7-20 and Zybo Z7-10
## To use it in a project:
## - uncomment the lines corresponding to used pins
## - rename the used ports (in each line, after get_ports) according to the top level signal names in the project

##Clock signal
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33} [get_ports clk]
#create_clock -period 24.000 -name sys_clk_pin -waveform {0.000 12.000} -add [get_ports clk]


##Switches
#set_property -dict { PACKAGE_PIN G15   IOSTANDARD LVCMOS33 } [get_ports { sw[0] }]; #IO_L19N_T3_VREF_35 Sch=sw[0]
set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports rst_n]
#set_property -dict { PACKAGE_PIN W13   IOSTANDARD LVCMOS33 } [get_ports { sw[2] }]; #IO_L4N_T0_34 Sch=sw[2]
#set_property -dict { PACKAGE_PIN T16   IOSTANDARD LVCMOS33 } [get_ports { sw[3] }]; #IO_L9P_T1_DQS_34 Sch=sw[3]


##Buttons
#set_property -dict { PACKAGE_PIN K18   IOSTANDARD LVCMOS33 } [get_ports { btn[0] }]; #IO_L12N_T1_MRCC_35 Sch=btn[0]
#set_property -dict { PACKAGE_PIN P16   IOSTANDARD LVCMOS33 } [get_ports { btn[1] }]; #IO_L24N_T3_34 Sch=btn[1]
#set_property -dict { PACKAGE_PIN K19   IOSTANDARD LVCMOS33 } [get_ports { btn[2] }]; #IO_L10P_T1_AD11P_35 Sch=btn[2]
#set_property -dict { PACKAGE_PIN Y16   IOSTANDARD LVCMOS33 } [get_ports { btn[3] }]; #IO_L7P_T1_34 Sch=btn[3]


##LEDs
#set_property -dict { PACKAGE_PIN M14   IOSTANDARD LVCMOS33 } [get_ports { led[0] }]; #IO_L23P_T3_35 Sch=led[0]
#set_property -dict { PACKAGE_PIN M15   IOSTANDARD LVCMOS33 } [get_ports { led[1] }]; #IO_L23N_T3_35 Sch=led[1]
#set_property -dict { PACKAGE_PIN G14   IOSTANDARD LVCMOS33 } [get_ports { led[2] }]; #IO_0_35 Sch=led[2]
#set_property -dict { PACKAGE_PIN D18   IOSTANDARD LVCMOS33 } [get_ports { led[3] }]; #IO_L3N_T0_DQS_AD1N_35 Sch=led[3]


##RGB LED 5 (Zybo Z7-20 only)
#set_property -dict { PACKAGE_PIN Y11   IOSTANDARD LVCMOS33 } [get_ports { check_led_ }]; #IO_L18N_T2_13 Sch=led5_r
set_property -dict {PACKAGE_PIN T5 IOSTANDARD LVCMOS33} [get_ports full_o]
set_property -dict {PACKAGE_PIN Y12 IOSTANDARD LVCMOS33} [get_ports empty_o]

##RGB LED 6
#set_property -dict { PACKAGE_PIN V16   IOSTANDARD LVCMOS33 } [get_ports { led6_r }]; #IO_L18P_T2_34 Sch=led6_r
#set_property -dict { PACKAGE_PIN F17   IOSTANDARD LVCMOS33 } [get_ports { led6_g }]; #IO_L6N_T0_VREF_35 Sch=led6_g
#set_property -dict { PACKAGE_PIN M17   IOSTANDARD LVCMOS33 } [get_ports { led6_b }]; #IO_L8P_T1_AD10P_35 Sch=led6_b


##Audio Codec
#set_property -dict { PACKAGE_PIN R19   IOSTANDARD LVCMOS33 } [get_ports { ac_bclk }]; #IO_0_34 Sch=ac_bclk
#set_property -dict { PACKAGE_PIN R17   IOSTANDARD LVCMOS33 } [get_ports { ac_mclk }]; #IO_L19N_T3_VREF_34 Sch=ac_mclk
#set_property -dict { PACKAGE_PIN P18   IOSTANDARD LVCMOS33 } [get_ports { ac_muten }]; #IO_L23N_T3_34 Sch=ac_muten
#set_property -dict { PACKAGE_PIN R18   IOSTANDARD LVCMOS33 } [get_ports { ac_pbdat }]; #IO_L20N_T3_34 Sch=ac_pbdat
#set_property -dict { PACKAGE_PIN T19   IOSTANDARD LVCMOS33 } [get_ports { ac_pblrc }]; #IO_25_34 Sch=ac_pblrc
#set_property -dict { PACKAGE_PIN R16   IOSTANDARD LVCMOS33 } [get_ports { ac_recdat }]; #IO_L19P_T3_34 Sch=ac_recdat
#set_property -dict { PACKAGE_PIN Y18   IOSTANDARD LVCMOS33 } [get_ports { ac_reclrc }]; #IO_L17P_T2_34 Sch=ac_reclrc
#set_property -dict { PACKAGE_PIN N18   IOSTANDARD LVCMOS33 } [get_ports { ac_scl }]; #IO_L13P_T2_MRCC_34 Sch=ac_scl
#set_property -dict { PACKAGE_PIN N17   IOSTANDARD LVCMOS33 } [get_ports { ac_sda }]; #IO_L23P_T3_34 Sch=ac_sda


##Additional Ethernet signals
#set_property -dict { PACKAGE_PIN F16   IOSTANDARD LVCMOS33  PULLUP true    } [get_ports { eth_int_pu_b }]; #IO_L6P_T0_35 Sch=eth_int_pu_b
#set_property -dict { PACKAGE_PIN E17   IOSTANDARD LVCMOS33 } [get_ports { eth_rst_b }]; #IO_L3P_T0_DQS_AD1P_35 Sch=eth_rst_b


##USB-OTG over-current detect pin
#set_property -dict { PACKAGE_PIN U13   IOSTANDARD LVCMOS33 } [get_ports { otg_oc }]; #IO_L3P_T0_DQS_PUDC_B_34 Sch=otg_oc


##Fan (Zybo Z7-20 only)
#set_property -dict { PACKAGE_PIN Y13   IOSTANDARD LVCMOS33  PULLUP true    } [get_ports { fan_fb_pu }]; #IO_L20N_T3_13 Sch=fan_fb_pu


##HDMI RX
#set_property -dict { PACKAGE_PIN W19   IOSTANDARD LVCMOS33 } [get_ports { hdmi_rx_hpd }]; #IO_L22N_T3_34 Sch=hdmi_rx_hpd
#set_property -dict { PACKAGE_PIN W18   IOSTANDARD LVCMOS33 } [get_ports { hdmi_rx_scl }]; #IO_L22P_T3_34 Sch=hdmi_rx_scl
#set_property -dict { PACKAGE_PIN Y19   IOSTANDARD LVCMOS33 } [get_ports { hdmi_rx_sda }]; #IO_L17N_T2_34 Sch=hdmi_rx_sda
#set_property -dict { PACKAGE_PIN U19   IOSTANDARD TMDS_33     } [get_ports { hdmi_rx_clk_n }]; #IO_L12N_T1_MRCC_34 Sch=hdmi_rx_clk_n
#set_property -dict { PACKAGE_PIN U18   IOSTANDARD TMDS_33     } [get_ports { hdmi_rx_clk_p }]; #IO_L12P_T1_MRCC_34 Sch=hdmi_rx_clk_p
#set_property -dict { PACKAGE_PIN W20   IOSTANDARD TMDS_33     } [get_ports { hdmi_rx_n[0] }]; #IO_L16N_T2_34 Sch=hdmi_rx_n[0]
#set_property -dict { PACKAGE_PIN V20   IOSTANDARD TMDS_33     } [get_ports { hdmi_rx_p[0] }]; #IO_L16P_T2_34 Sch=hdmi_rx_p[0]
#set_property -dict { PACKAGE_PIN U20   IOSTANDARD TMDS_33     } [get_ports { hdmi_rx_n[1] }]; #IO_L15N_T2_DQS_34 Sch=hdmi_rx_n[1]
#set_property -dict { PACKAGE_PIN T20   IOSTANDARD TMDS_33     } [get_ports { hdmi_rx_p[1] }]; #IO_L15P_T2_DQS_34 Sch=hdmi_rx_p[1]
#set_property -dict { PACKAGE_PIN P20   IOSTANDARD TMDS_33     } [get_ports { hdmi_rx_n[2] }]; #IO_L14N_T2_SRCC_34 Sch=hdmi_rx_n[2]
#set_property -dict { PACKAGE_PIN N20   IOSTANDARD TMDS_33     } [get_ports { hdmi_rx_p[2] }]; #IO_L14P_T2_SRCC_34 Sch=hdmi_rx_p[2]

##HDMI RX CEC (Zybo Z7-20 only)
#set_property -dict { PACKAGE_PIN Y8    IOSTANDARD LVCMOS33 } [get_ports { hdmi_rx_cec }]; #IO_L14N_T2_SRCC_13 Sch=hdmi_rx_cec


##HDMI TX
#set_property -dict { PACKAGE_PIN E18   IOSTANDARD LVCMOS33 } [get_ports { hdmi_tx_hpd }]; #IO_L5P_T0_AD9P_35 Sch=hdmi_tx_hpd
#set_property -dict { PACKAGE_PIN G17   IOSTANDARD LVCMOS33 } [get_ports { hdmi_tx_scl }]; #IO_L16P_T2_35 Sch=hdmi_tx_scl
#set_property -dict { PACKAGE_PIN G18   IOSTANDARD LVCMOS33 } [get_ports { hdmi_tx_sda }]; #IO_L16N_T2_35 Sch=hdmi_tx_sda
#set_property -dict { PACKAGE_PIN H17   IOSTANDARD TMDS_33     } [get_ports { hdmi_tx_clk_n }]; #IO_L13N_T2_MRCC_35 Sch=hdmi_tx_clk_n
#set_property -dict { PACKAGE_PIN H16   IOSTANDARD TMDS_33     } [get_ports { hdmi_tx_clk_p }]; #IO_L13P_T2_MRCC_35 Sch=hdmi_tx_clk_p
#set_property -dict { PACKAGE_PIN D20   IOSTANDARD TMDS_33     } [get_ports { hdmi_tx_n[0] }]; #IO_L4N_T0_35 Sch=hdmi_tx_n[0]
#set_property -dict { PACKAGE_PIN D19   IOSTANDARD TMDS_33     } [get_ports { hdmi_tx_p[0] }]; #IO_L4P_T0_35 Sch=hdmi_tx_p[0]
#set_property -dict { PACKAGE_PIN B20   IOSTANDARD TMDS_33     } [get_ports { hdmi_tx_n[1] }]; #IO_L1N_T0_AD0N_35 Sch=hdmi_tx_n[1]
#set_property -dict { PACKAGE_PIN C20   IOSTANDARD TMDS_33     } [get_ports { hdmi_tx_p[1] }]; #IO_L1P_T0_AD0P_35 Sch=hdmi_tx_p[1]
#set_property -dict { PACKAGE_PIN A20   IOSTANDARD TMDS_33     } [get_ports { hdmi_tx_n[2] }]; #IO_L2N_T0_AD8N_35 Sch=hdmi_tx_n[2]
#set_property -dict { PACKAGE_PIN B19   IOSTANDARD TMDS_33     } [get_ports { hdmi_tx_p[2] }]; #IO_L2P_T0_AD8P_35 Sch=hdmi_tx_p[2]

##HDMI TX CEC
#set_property -dict { PACKAGE_PIN E19   IOSTANDARD LVCMOS33 } [get_ports { hdmi_tx_cec }]; #IO_L5N_T0_AD9N_35 Sch=hdmi_tx_cec


##Pmod Header JA (XADC)
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS33} [get_ports {datain[0]}]
set_property -dict {PACKAGE_PIN L14 IOSTANDARD LVCMOS33} [get_ports {datain[1]}]
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports {datain[2]}]
set_property -dict {PACKAGE_PIN K14 IOSTANDARD LVCMOS33} [get_ports {datain[3]}]
set_property -dict {PACKAGE_PIN N16 IOSTANDARD LVCMOS33} [get_ports {datain[4]}]
set_property -dict {PACKAGE_PIN L15 IOSTANDARD LVCMOS33} [get_ports {datain[5]}]
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS33} [get_ports {datain[6]}]
set_property -dict {PACKAGE_PIN J14 IOSTANDARD LVCMOS33} [get_ports {datain[7]}]


##Pmod Header JB (Zybo Z7-20 only) #일단 임의로 사용
#set_property -dict {PACKAGE_PIN V8 IOSTANDARD LVCMOS33} [get_ports {dataout[0]}]
#set_property -dict {PACKAGE_PIN W8 IOSTANDARD LVCMOS33} [get_ports {dataout[1]}]
#set_property -dict {PACKAGE_PIN U7 IOSTANDARD LVCMOS33} [get_ports {dataout[2]}]
#set_property -dict {PACKAGE_PIN V7 IOSTANDARD LVCMOS33} [get_ports {dataout[3]}]
#set_property -dict {PACKAGE_PIN Y7 IOSTANDARD LVCMOS33} [get_ports {dataout[4]}]
#set_property -dict {PACKAGE_PIN Y6 IOSTANDARD LVCMOS33} [get_ports {dataout[5]}]
#set_property -dict {PACKAGE_PIN V6 IOSTANDARD LVCMOS33} [get_ports {dataout[6]}]
#set_property -dict {PACKAGE_PIN W6 IOSTANDARD LVCMOS33} [get_ports {dataout[7]}]


##Pmod Header JC
#set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVCMOS33} [get_ports {dataout[8]}]
#set_property -dict {PACKAGE_PIN W15 IOSTANDARD LVCMOS33} [get_ports {dataout[9]}]
#set_property -dict {PACKAGE_PIN T11 IOSTANDARD LVCMOS33} [get_ports {dataout[10]}]
set_property -dict {PACKAGE_PIN T10 IOSTANDARD LVCMOS33} [get_ports o_SPI_Clk_]
set_property -dict {PACKAGE_PIN W14 IOSTANDARD LVCMOS33} [get_ports o_TX_Ready_]
set_property -dict {PACKAGE_PIN Y14 IOSTANDARD LVCMOS33} [get_ports dataout]
set_property -dict {PACKAGE_PIN T12 IOSTANDARD LVCMOS33} [get_ports clk_SPI]
set_property -dict {PACKAGE_PIN U12 IOSTANDARD LVCMOS33} [get_ports rden_i]


##Pmod Header JD
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33} [get_ports wren_i]
set_property -dict {PACKAGE_PIN T15 IOSTANDARD LVCMOS33} [get_ports {datain[8]}]
set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports {datain[9]}]
set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports {datain[10]}]
set_property -dict {PACKAGE_PIN U14 IOSTANDARD LVCMOS33} [get_ports {datain[11]}]
set_property -dict {PACKAGE_PIN U15 IOSTANDARD LVCMOS33} [get_ports clk_adc]
set_property -dict {PACKAGE_PIN V17 IOSTANDARD LVCMOS33} [get_ports clk_fifo]
set_property -dict {PACKAGE_PIN V18 IOSTANDARD LVCMOS33} [get_ports clk_ILA]


##Pmod Header JE
#set_property -dict { PACKAGE_PIN V12   IOSTANDARD LVCMOS33 } [get_ports { je[0] }]; #IO_L4P_T0_34 Sch=je[1]
#set_property -dict { PACKAGE_PIN W16   IOSTANDARD LVCMOS33 } [get_ports { je[1] }]; #IO_L18N_T2_34 Sch=je[2]
#set_property -dict { PACKAGE_PIN J15   IOSTANDARD LVCMOS33 } [get_ports { je[2] }]; #IO_25_35 Sch=je[3]
#set_property -dict { PACKAGE_PIN H15   IOSTANDARD LVCMOS33 } [get_ports { je[3] }]; #IO_L19P_T3_35 Sch=je[4]
#set_property -dict { PACKAGE_PIN V13   IOSTANDARD LVCMOS33 } [get_ports { je[4] }]; #IO_L3N_T0_DQS_34 Sch=je[7]
#set_property -dict { PACKAGE_PIN U17   IOSTANDARD LVCMOS33 } [get_ports { je[5] }]; #IO_L9N_T1_DQS_34 Sch=je[8]
#set_property -dict { PACKAGE_PIN T17   IOSTANDARD LVCMOS33 } [get_ports { je[6] }]; #IO_L20P_T3_34 Sch=je[9]
#set_property -dict { PACKAGE_PIN Y17   IOSTANDARD LVCMOS33 } [get_ports { je[7] }]; #IO_L7N_T1_34 Sch=je[10]


##Pcam MIPI CSI-2 Connector
## This configuration expects the sensor to use 672Mbps/lane = 336 MHz HS_Clk
#create_clock -period 2.976 -name dphy_hs_clock_clk_p -waveform {0.000 1.488} [get_ports dphy_hs_clock_clk_p]
#set_property INTERNAL_VREF 0.6 [get_iobanks 35]
#set_property -dict { PACKAGE_PIN J19   IOSTANDARD HSUL_12     } [get_ports { dphy_clk_lp_n }]; #IO_L10N_T1_AD11N_35 Sch=lp_clk_n
#set_property -dict { PACKAGE_PIN H20   IOSTANDARD HSUL_12     } [get_ports { dphy_clk_lp_p }]; #IO_L17N_T2_AD5N_35 Sch=lp_clk_p
#set_property -dict { PACKAGE_PIN M18   IOSTANDARD HSUL_12     } [get_ports { dphy_data_lp_n[0] }]; #IO_L8N_T1_AD10N_35 Sch=lp_lane_n[0]
#set_property -dict { PACKAGE_PIN L19   IOSTANDARD HSUL_12     } [get_ports { dphy_data_lp_p[0] }]; #IO_L9P_T1_DQS_AD3P_35 Sch=lp_lane_p[0]
#set_property -dict { PACKAGE_PIN L20   IOSTANDARD HSUL_12     } [get_ports { dphy_data_lp_n[1] }]; #IO_L9N_T1_DQS_AD3N_35 Sch=lp_lane_n[1]
#set_property -dict { PACKAGE_PIN J20   IOSTANDARD HSUL_12     } [get_ports { dphy_data_lp_p[1] }]; #IO_L17P_T2_AD5P_35 Sch=lp_lane_p[1]
#set_property -dict { PACKAGE_PIN H18   IOSTANDARD LVDS_25     } [get_ports { dphy_hs_clock_clk_n }]; #IO_L14N_T2_AD4N_SRCC_35 Sch=mipi_clk_n
#set_property -dict { PACKAGE_PIN J18   IOSTANDARD LVDS_25     } [get_ports { dphy_hs_clock_clk_p }]; #IO_L14P_T2_AD4P_SRCC_35 Sch=mipi_clk_p
#set_property -dict { PACKAGE_PIN M20   IOSTANDARD LVDS_25     } [get_ports { dphy_data_hs_n[0] }]; #IO_L7N_T1_AD2N_35 Sch=mipi_lane_n[0]
#set_property -dict { PACKAGE_PIN M19   IOSTANDARD LVDS_25     } [get_ports { dphy_data_hs_p[0] }]; #IO_L7P_T1_AD2P_35 Sch=mipi_lane_p[0]
#set_property -dict { PACKAGE_PIN L17   IOSTANDARD LVDS_25     } [get_ports { dphy_data_hs_n[1] }]; #IO_L11N_T1_SRCC_35 Sch=mipi_lane_n[1]
#set_property -dict { PACKAGE_PIN L16   IOSTANDARD LVDS_25     } [get_ports { dphy_data_hs_p[1] }]; #IO_L11P_T1_SRCC_35 Sch=mipi_lane_p[1]
#set_property -dict { PACKAGE_PIN G19   IOSTANDARD LVCMOS33 } [get_ports { cam_clk }]; #IO_L18P_T2_AD13P_35 Sch=cam_clk
#set_property -dict { PACKAGE_PIN G20   IOSTANDARD LVCMOS33 	PULLUP true} [get_ports { cam_gpio }]; #IO_L18N_T2_AD13N_35 Sch=cam_gpio
#set_property -dict { PACKAGE_PIN F20   IOSTANDARD LVCMOS33 } [get_ports { cam_scl }]; #IO_L15N_T2_DQS_AD12N_35 Sch=cam_scl
#set_property -dict { PACKAGE_PIN F19   IOSTANDARD LVCMOS33 } [get_ports { cam_sda }]; #IO_L15P_T2_DQS_AD12P_35 Sch=cam_sda


##Unloaded Crypto Chip SWI (for future use)
#set_property -dict { PACKAGE_PIN P19   IOSTANDARD LVCMOS33 } [get_ports { crypto_sda }]; #IO_L13N_T2_MRCC_34 Sch=crypto_sda


##Unconnected Pins (Zybo Z7-20 only)
#set_property PACKAGE_PIN T9 [get_ports {dataout[0]}]
#set_property PACKAGE_PIN U10 [get_ports {dataout[1]}]
#set_property PACKAGE_PIN U5 [get_ports {dataout[2]}]
#set_property PACKAGE_PIN U8 [get_ports {dataout[3]}]
#set_property PACKAGE_PIN U9 [get_ports {dataout[4]}]
#set_property PACKAGE_PIN V10 [get_ports {dataout[5]}]
#set_property PACKAGE_PIN V11 [get_ports {dataout[6]}]
#set_property PACKAGE_PIN V5 [get_ports {dataout[7]}]
#set_property PACKAGE_PIN W10 [get_ports {dataout[8]}]
#set_property PACKAGE_PIN W11 [get_ports {dataout[9]}]
#set_property PACKAGE_PIN W9 [get_ports {dataout[10]}]
#set_property PACKAGE_PIN Y9 [get_ports {dataout[11]}]






create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 1 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 8192 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL false [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list c1/inst/clk_ILA]]
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 12 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {s1/i_TX_Byte[0]} {s1/i_TX_Byte[1]} {s1/i_TX_Byte[2]} {s1/i_TX_Byte[3]} {s1/i_TX_Byte[4]} {s1/i_TX_Byte[5]} {s1/i_TX_Byte[6]} {s1/i_TX_Byte[7]} {s1/i_TX_Byte[8]} {s1/i_TX_Byte[9]} {s1/i_TX_Byte[10]} {s1/i_TX_Byte[11]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 12 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list {s1/r_TX_Byte[0]} {s1/r_TX_Byte[1]} {s1/r_TX_Byte[2]} {s1/r_TX_Byte[3]} {s1/r_TX_Byte[4]} {s1/r_TX_Byte[5]} {s1/r_TX_Byte[6]} {s1/r_TX_Byte[7]} {s1/r_TX_Byte[8]} {s1/r_TX_Byte[9]} {s1/r_TX_Byte[10]} {s1/r_TX_Byte[11]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 12 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list {datain_IBUF[0]} {datain_IBUF[1]} {datain_IBUF[2]} {datain_IBUF[3]} {datain_IBUF[4]} {datain_IBUF[5]} {datain_IBUF[6]} {datain_IBUF[7]} {datain_IBUF[8]} {datain_IBUF[9]} {datain_IBUF[10]} {datain_IBUF[11]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 12 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list {fifo_out_spi_in[0]} {fifo_out_spi_in[1]} {fifo_out_spi_in[2]} {fifo_out_spi_in[3]} {fifo_out_spi_in[4]} {fifo_out_spi_in[5]} {fifo_out_spi_in[6]} {fifo_out_spi_in[7]} {fifo_out_spi_in[8]} {fifo_out_spi_in[9]} {fifo_out_spi_in[10]} {fifo_out_spi_in[11]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
set_property port_width 1 [get_debug_ports u_ila_0/probe4]
connect_debug_port u_ila_0/probe4 [get_nets [list debug_empty_o]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
set_property port_width 1 [get_debug_ports u_ila_0/probe5]
connect_debug_port u_ila_0/probe5 [get_nets [list debug_full_o]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe6]
set_property port_width 1 [get_debug_ports u_ila_0/probe6]
connect_debug_port u_ila_0/probe6 [get_nets [list debug_rden_i]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe7]
set_property port_width 1 [get_debug_ports u_ila_0/probe7]
connect_debug_port u_ila_0/probe7 [get_nets [list debug_wren_i]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe8]
set_property port_width 1 [get_debug_ports u_ila_0/probe8]
connect_debug_port u_ila_0/probe8 [get_nets [list dataout_OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe9]
set_property port_width 1 [get_debug_ports u_ila_0/probe9]
connect_debug_port u_ila_0/probe9 [get_nets [list o_TX_Ready__OBUF]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe10]
set_property port_width 1 [get_debug_ports u_ila_0/probe10]
connect_debug_port u_ila_0/probe10 [get_nets [list o_SPI_Clk__OBUF]]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_ILA_OBUF]

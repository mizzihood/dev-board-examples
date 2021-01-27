##############################################################################
## This file is part of 'Example Project Firmware'.
## It is subject to the license terms in the LICENSE.txt file found in the
## top-level directory of this distribution and at:
##    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html.
## No part of 'Example Project Firmware', including this file,
## may be copied, modified, propagated, or distributed except according to
## the terms contained in the LICENSE.txt file.
##############################################################################

####################################
## Application Timing Constraints ##
####################################

##########################
## Misc. Configurations ##
##########################
create_generated_clock -name appClk        [get_pins U_App/U_Reg/U_CoreClockGen/MmcmGen.U_Mmcm/CLKOUT0]
create_generated_clock -name idelayCtrlClk [get_pins U_App/U_Reg/U_CoreClockGen/MmcmGen.U_Mmcm/CLKOUT1]
create_generated_clock -name adcClk        [get_pins U_App/U_Reg/U_CoreClockGen/MmcmGen.U_Mmcm/CLKOUT2]
create_generated_clock -name bitClk        [get_pins U_App/U_Reg/U_iserdesClockGen/MmcmGen.U_Mmcm/CLKOUT0]
create_generated_clock -name deserClk      [get_pins U_App/U_Reg/U_iserdesClockGen/MmcmGen.U_Mmcm/CLKOUT1]
create_generated_clock -name byteClk       [get_pins U_App/U_Reg/U_iserdesClockGen/MmcmGen.U_Mmcm/CLKOUT2]
create_generated_clock -name asicRdClk     [get_pins U_App/U_Reg/U_iserdesClockGen/MmcmGen.U_Mmcm/CLKOUT3]
create_generated_clock -name dnaClk        [get_pins {U_App/U_Reg/U_AxiVersion/GEN_DEVICE_DNA.DeviceDna_1/GEN_ULTRA_SCALE.DeviceDnaUltraScale_Inst/BUFGCE_DIV_Inst/O}]
create_generated_clock -name ethClk125MHz  [get_pins GEN_GTH.U_1GigE/GEN_INT_PLL.U_MMCM/MmcmGen.U_Mmcm/CLKOUT0]

set_clock_groups -asynchronous -group [get_clocks ethClk125MHz]  -group [get_clocks appClk]
set_clock_groups -asynchronous -group [get_clocks ethClk125MHz]  -group [get_clocks byteClk]
set_clock_groups -asynchronous -group [get_clocks sysClk]  -group [get_clocks appClk]
set_clock_groups -asynchronous -group [get_clocks sysClk]  -group [get_clocks bitClk]
set_clock_groups -asynchronous -group [get_clocks sysClk]  -group [get_clocks byteClk]
set_clock_groups -asynchronous -group [get_clocks sysClk]  -group [get_clocks asicRdClk]
set_clock_groups -asynchronous -group [get_clocks sysClk]  -group [get_clocks adcBitClkR]
set_clock_groups -asynchronous -group [get_clocks appClk]  -group [get_clocks byteClk]
set_clock_groups -asynchronous -group [get_clocks appClk]  -group [get_clocks deserClk]
set_clock_groups -asynchronous -group [get_clocks dnaClk]  -group [get_clocks byteClk]
set_clock_groups -asynchronous -group [get_clocks dnaClk]  -group [get_clocks appClk]
set_clock_groups -asynchronous -group [get_clocks byteClk] -group [get_clocks deserClk]
set_clock_groups -asynchronous -group [get_clocks appClk]  -group [get_clocks adcBitClkR]
set_clock_groups -asynchronous -group [get_clocks appClk]  -group [get_clocks adcBitClkRD4]
set_clock_groups -asynchronous -group [get_clocks appClk]  -group [get_clocks adcMonDoClkP]
set_clock_groups -asynchronous -group [get_clocks appClk]  -group [get_clocks bitClk]
set_clock_groups -asynchronous -group [get_clocks appClk]  -group [get_clocks asicRdClk]
set_clock_groups -asynchronous -group [get_clocks byteClk] -group [get_clocks adcBitClkR]
set_clock_groups -asynchronous -group [get_clocks byteClk] -group [get_clocks adcBitClkRD4]
set_clock_groups -asynchronous -group [get_clocks appClk]  -group [get_clocks idelayCtrlClk]
#set_clock_groups -asynchronous -group [get_clocks -of_objects [get_pins U_Core/U_Mmcm/PllGen.U_Pll/CLKOUT0]] -group [get_clocks -of_objects [get_pins U_App/U_CoreClockGen/MmcmGen.U_Mmcm/CLKOUT2]]


# I/O Port Mapping

set_property -dict { PACKAGE_PIN V12 IOSTANDARD ANALOG } [get_ports { vPIn }]
set_property -dict { PACKAGE_PIN W11 IOSTANDARD ANALOG } [get_ports { vNIn }]

set_property -dict { PACKAGE_PIN AN8 IOSTANDARD LVCMOS18 } [get_ports { extRst }]

set_property -dict { PACKAGE_PIN AP8 IOSTANDARD LVCMOS18 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN H23 IOSTANDARD LVCMOS18 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN P20 IOSTANDARD LVCMOS18 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN P21 IOSTANDARD LVCMOS18 } [get_ports { led[3] }]
set_property -dict { PACKAGE_PIN N22 IOSTANDARD LVCMOS18 } [get_ports { led[4] }]
set_property -dict { PACKAGE_PIN M22 IOSTANDARD LVCMOS18 } [get_ports { led[5] }]
set_property -dict { PACKAGE_PIN R23 IOSTANDARD LVCMOS18 } [get_ports { led[6] }]
set_property -dict { PACKAGE_PIN P23 IOSTANDARD LVCMOS18 } [get_ports { led[7] }]

# MDIO/Ext. PHY
set_property PACKAGE_PIN K25     [get_ports "phyIrqN"]
set_property IOSTANDARD LVCMOS18 [get_ports "phyIrqN"]
set_property PACKAGE_PIN L25     [get_ports "phyMdc"]
set_property IOSTANDARD LVCMOS18 [get_ports "phyMdc"]
set_property PACKAGE_PIN H26     [get_ports "phyMdio"]
set_property IOSTANDARD LVCMOS18 [get_ports "phyMdio"]
set_property PACKAGE_PIN J23     [get_ports "phyRstN"]
set_property IOSTANDARD LVCMOS18 [get_ports "phyRstN"]

# GPIO DIP Switch
set_property PACKAGE_PIN AN16 [get_ports "gpioDip[0]"]
set_property IOSTANDARD LVCMOS12 [get_ports "gpioDip[0]"]
set_property PACKAGE_PIN AN19 [get_ports "gpioDip[1]"]
set_property IOSTANDARD LVCMOS12 [get_ports "gpioDip[1]"]
set_property PACKAGE_PIN AP18 [get_ports "gpioDip[2]"]
set_property IOSTANDARD LVCMOS12 [get_ports "gpioDip[2]"]
set_property PACKAGE_PIN AN14 [get_ports "gpioDip[3]"]
set_property IOSTANDARD LVCMOS12 [get_ports "gpioDip[3]"]

#GPIO SMA
set_property -dict { PACKAGE_PIN  H27 IOSTANDARD LVCMOS18 } [get_ports { user_sma_gpio_p }]

# On-Board System clock
set_property ODT RTT_48 [get_ports "sysClk300N"]
set_property PACKAGE_PIN AK16 [get_ports "sysClk300N"]
set_property IOSTANDARD DIFF_SSTL12_DCI [get_ports "sysClk300N"]
set_property PACKAGE_PIN AK17 [get_ports "sysClk300P"]
set_property IOSTANDARD DIFF_SSTL12_DCI [get_ports "sysClk300P"]
set_property ODT RTT_48 [get_ports "sysClk300P"]


set_clock_groups -asynchronous -group [get_clocks {ethClk125MHz}] -group [get_clocks {dnaClk}]


# Adding pins for CRYO
#################
# Cheeseball switch (this is already mapped to gpio[0]!)  =(
#set_property -dict { PACKAGE_PIN  AN16 IOSTANDARD LVCMOS12 } [get_ports { rstSwitch }]
# FEMB-specific constraints
set_property -dict { PACKAGE_PIN  V27 IOSTANDARD LVCMOS18 } [get_ports { asicGlblRst }]
set_property -dict { PACKAGE_PIN  V28 IOSTANDARD LVCMOS18 } [get_ports { pulse }]
set_property -dict { PACKAGE_PIN  V29 IOSTANDARD LVDS } [get_ports { asicSaciClk_p }]
set_property -dict { PACKAGE_PIN  W29 IOSTANDARD LVDS } [get_ports { asicSaciClk_n }]
set_property -dict { PACKAGE_PIN  V22 IOSTANDARD LVDS } [get_ports { asicSaciCmd_p }]
set_property -dict { PACKAGE_PIN  V23 IOSTANDARD LVDS } [get_ports { asicSaciCmd_n }]
set_property -dict { PACKAGE_PIN  U24 IOSTANDARD LVDS } [get_ports { asicSaciRsp_p }]
set_property -dict { PACKAGE_PIN  U25 IOSTANDARD LVDS } [get_ports { asicSaciRsp_n }]
set_property -dict { PACKAGE_PIN  AB25 IOSTANDARD LVDS } [get_ports { asicSmpClk_p }]
set_property -dict { PACKAGE_PIN  AB26 IOSTANDARD LVDS } [get_ports { asicSmpClk_n }]
#set_property -dict { PACKAGE_PIN  ?? IOSTANDARD LVCMOS18 } [get_ports { clk_p_R }]
#set_property -dict { PACKAGE_PIN  ?? IOSTANDARD LVCMOS18 } [get_ports { clk_n_R }]
set_property -dict { PACKAGE_PIN  V21 IOSTANDARD LVDS } [get_ports { asicD0out_p[0] }]
set_property -dict { PACKAGE_PIN  W21 IOSTANDARD LVDS } [get_ports { asicD0out_n[0] }]
set_property EQUALIZATION EQ_LEVEL1 [get_ports {asicD0out_p[0]}]
set_property DIFF_TERM_ADV TERM_100 [get_ports {asicD0out_p[0]}]
set_property -dict { PACKAGE_PIN  AC22 IOSTANDARD LVDS } [get_ports { asicD1out_p[0] }]
set_property -dict { PACKAGE_PIN  AC23 IOSTANDARD LVDS } [get_ports { asicD1out_n[0] }]
set_property EQUALIZATION EQ_LEVEL1 [get_ports {asicD1out_p[0]}]
set_property DIFF_TERM_ADV TERM_100 [get_ports {asicD1out_p[0]}]
set_property -dict { PACKAGE_PIN  V26 IOSTANDARD LVDS } [get_ports { asicR0_p }]
set_property -dict { PACKAGE_PIN  W26 IOSTANDARD LVDS } [get_ports { asicR0_n }]
set_property -dict { PACKAGE_PIN  T22 IOSTANDARD LVCMOS18 } [get_ports { asicSaciSel[0]}]
set_property -dict { PACKAGE_PIN  T23 IOSTANDARD LVCMOS18 } [get_ports { asicSaciSel[1]}]
set_property -dict { PACKAGE_PIN  U21 IOSTANDARD LVDS } [get_ports { asicD0out_p[1] }]
set_property -dict { PACKAGE_PIN  U22 IOSTANDARD LVDS } [get_ports { asicD0out_n[1] }]
set_property EQUALIZATION EQ_LEVEL1 [get_ports {asicD0out_p[1]}]
set_property DIFF_TERM_ADV TERM_100 [get_ports {asicD0out_p[1]}]
set_property -dict { PACKAGE_PIN  AA20 IOSTANDARD LVDS } [get_ports { asicD1out_p[1] }]
set_property -dict { PACKAGE_PIN  AB20 IOSTANDARD LVDS } [get_ports { asicD1out_n[1] }]
set_property EQUALIZATION EQ_LEVEL1 [get_ports {asicD1out_p[1]}]
set_property DIFF_TERM_ADV TERM_100 [get_ports {asicD1out_p[1]}]

# Jitter cleaner pins
set_property -dict { PACKAGE_PIN W28  IOSTANDARD LVCMOS18 } [get_ports { pllSck }]
set_property -dict { PACKAGE_PIN U27  IOSTANDARD LVCMOS18 } [get_ports { pllSdo }]
set_property -dict { PACKAGE_PIN Y28  IOSTANDARD LVCMOS18 } [get_ports { pllSdi }]
set_property -dict { PACKAGE_PIN U26  IOSTANDARD LVCMOS18 } [get_ports { pllCsL }]
set_property -dict { PACKAGE_PIN AA22 IOSTANDARD LVDS } [get_ports { pllInClk_p }]
set_property -dict { PACKAGE_PIN AB22 IOSTANDARD LVDS } [get_ports { pllInClk_n }]
set_property LVDS_PRE_EMPHASIS TRUE [get_ports { pllInClk_p }]
set_property LVDS_PRE_EMPHASIS TRUE [get_ports { asicSmpClk_p }]
set_property LVDS_PRE_EMPHASIS TRUE [get_ports { asicSaciCmd_p }]
set_property LVDS_PRE_EMPHASIS TRUE [get_ports { asicSaciClk_p }]
set_property LVDS_PRE_EMPHASIS TRUE [get_ports { asicR0_p }]

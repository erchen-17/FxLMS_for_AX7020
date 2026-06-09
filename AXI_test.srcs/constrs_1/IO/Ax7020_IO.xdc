###############################################################################
# System Clock & Reset
###############################################################################

# 50MHz system clock
set_property PACKAGE_PIN U18 [get_ports clock_50m]
set_property IOSTANDARD LVCMOS33 [get_ports clock_50m]
create_clock -period 20.000 -name sys_clk [get_ports clock_50m]

###############################################################################
# WM8731 I2C Interface
###############################################################################

# I2C SCL
set_property PACKAGE_PIN W19 [get_ports aud_scl]
set_property IOSTANDARD LVCMOS33 [get_ports aud_scl]

# I2C SDA (inout)
set_property PACKAGE_PIN W18 [get_ports aud_sda]
set_property IOSTANDARD LVCMOS33 [get_ports aud_sda]
set_property PULLTYPE PULLUP [get_ports aud_sda]


###############################################################################
# WM8731 I2S Interface
###############################################################################

# I2S Bit Clock (BCLK)
set_property PACKAGE_PIN P14 [get_ports bclk]
set_property IOSTANDARD LVCMOS33 [get_ports bclk]

# I2S Left/Right Clock (LRCLK / WS)
set_property PACKAGE_PIN W15 [get_ports adclrc]
set_property IOSTANDARD LVCMOS33 [get_ports adclrc]

set_property PACKAGE_PIN Y16 [get_ports daclrc]
set_property IOSTANDARD LVCMOS33 [get_ports daclrc]

# I2S ADC Data (from WM8731)
set_property PACKAGE_PIN Y17 [get_ports adc_dat]
set_property IOSTANDARD LVCMOS33 [get_ports adc_dat]

# I2S DAC Data (to WM8731)
set_property PACKAGE_PIN R14 [get_ports dac_dat]
set_property IOSTANDARD LVCMOS33 [get_ports dac_dat]

# J11 Reference Microphone
# I2C SCL
set_property PACKAGE_PIN F17 [get_ports aud_scl_1]
set_property IOSTANDARD LVCMOS33 [get_ports aud_scl_1]

# I2C SDA (inout)
set_property PACKAGE_PIN F16 [get_ports aud_sda_1]
set_property IOSTANDARD LVCMOS33 [get_ports aud_sda_1]
set_property PULLTYPE PULLUP [get_ports aud_sda_1]

###############################################################################
# WM8731 I2S Interface
###############################################################################

# I2S Bit Clock (BCLK)
set_property PACKAGE_PIN F19 [get_ports bclk_1]
set_property IOSTANDARD LVCMOS33 [get_ports bclk_1]

# I2S Left/Right Clock (LRCLK / WS)
set_property PACKAGE_PIN H18 [get_ports adclrc_1]
set_property IOSTANDARD LVCMOS33 [get_ports adclrc_1]

# I2S ADC Data (from WM8731)
set_property PACKAGE_PIN G20 [get_ports adc_dat_1]
set_property IOSTANDARD LVCMOS33 [get_ports adc_dat_1]



###############################################################################
# FxLMS Test Mode Button (Button A)
###############################################################################
# Button A: Enable FxLMS mode with adaptive weight update

set_property PACKAGE_PIN N16 [get_ports btn_fxlms_enable]
set_property IOSTANDARD LVCMOS33 [get_ports btn_fxlms_enable]
set_property PULLTYPE PULLDOWN [get_ports btn_fxlms_enable]

# Mark as asynchronous input (false path)
set_false_path -from [get_ports btn_fxlms_enable]

###############################################################################
# Pass-through Mode Button (Button B)
###############################################################################
# Button B: Enable pass-through mode (no FxLMS processing)


set_property PACKAGE_PIN R17 [get_ports btn_passthrough]
set_property IOSTANDARD LVCMOS33 [get_ports btn_passthrough]
set_property PULLTYPE PULLDOWN [get_ports btn_passthrough]

# Mark as asynchronous input (false path)
set_false_path -from [get_ports btn_passthrough]

###############################################################################
# Secondary Path Estimation Mode Button (Button C)
###############################################################################
# Button C: Enable secondary path estimation mode
# When pressed, system uses white noise to estimate S'(z)

set_property PACKAGE_PIN T17 [get_ports btn_estimate_mode]
set_property IOSTANDARD LVCMOS33 [get_ports btn_estimate_mode]
set_property PULLTYPE PULLDOWN [get_ports btn_estimate_mode]

# Mark as asynchronous input (false path)
set_false_path -from [get_ports btn_estimate_mode]

###############################################################################
#  NEW Button D: 参数切换按键 btn_switch KEY1
###############################################################################
set_property PACKAGE_PIN N15 [get_ports btn_switch]
set_property IOSTANDARD LVCMOS33 [get_ports btn_switch]
set_property PULLTYPE PULLDOWN [get_ports btn_switch]
set_false_path -from [get_ports btn_switch]

###############################################################################
#  NEW 4个LED：显示 16 种参数状态
###############################################################################
set_property PACKAGE_PIN M14 [get_ports led_1st]
set_property IOSTANDARD LVCMOS33 [get_ports led_1st]

set_property PACKAGE_PIN M15 [get_ports led_2nd]
set_property IOSTANDARD LVCMOS33 [get_ports led_2nd]

set_property PACKAGE_PIN K16 [get_ports led_3rd]
set_property IOSTANDARD LVCMOS33 [get_ports led_3rd]

set_property PACKAGE_PIN J16 [get_ports led_4th]
set_property IOSTANDARD LVCMOS33 [get_ports led_4th]






set_time_format -unit ns -decimal_places 3

# For FX3 clocking 
create_clock -name EXT_CLK  -period 10.000  [get_ports {EXT_CLK1  EXT_CLK1(n)  EXT_CLK2  EXT_CLK2(n)}]
create_generated_clock -source {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|vco0ph[0]} -divide_by 2 -duty_cycle 50.00 -name {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk} {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}
create_generated_clock -source {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|refclkin} -divide_by 2 -multiply_by 8 -duty_cycle 50.00 -name {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|vcoph[0]} {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|vcoph[0]}

# For IO clocking
create_clock -name USER_CLK -period 125.000 [get_ports {USER_CLK1 USER_CLK1(n) USER_CLK2 USER_CLK2(n)}]
create_generated_clock -name {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk} -source [get_pins {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|vco0ph[0]}] -duty_cycle 50/1 -multiply_by 1 -divide_by 24 -master_clock {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|vcoph[0]} [get_pins {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] 
create_generated_clock -name {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|vcoph[0]} -source [get_pins {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|refclkin}] -duty_cycle 50/1 -multiply_by 42 -master_clock {USER_CLK} [get_pins {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|vcoph[0]}] 

create_generated_clock -name {DATA_CLK} -source [get_pins {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] -duty_cycle 50.000 -multiply_by 1 -divide_by 2 -master_clock {inst8|user_io_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk} [get_nets {inst7|DATA_CLK}] 




derive_clock_uncertainty
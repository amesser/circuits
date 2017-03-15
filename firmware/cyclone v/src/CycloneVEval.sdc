set_time_format -unit ns -decimal_places 3

create_clock -name EXT_CLK -period 10.000 [get_ports {EXT_CLK1 EXT_CLK1(n) EXT_CLK2 EXT_CLK2(n)}]
create_generated_clock -source {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|vco0ph[0]} -divide_by 2 -duty_cycle 50.00 -name {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk} {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}
create_generated_clock -source {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|refclkin} -divide_by 2 -multiply_by 8 -duty_cycle 50.00 -name {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|vcoph[0]} {inst2|dummy_pll_inst|altera_pll_i|cyclonev_pll|fpll_0|fpll|vcoph[0]}
derive_clock_uncertainty
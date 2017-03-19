LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.numeric_std.all;

ENTITY dram_buffer IS
  PORT(

    data_in        : IN   std_logic_vector(31 downto 0);
    clk_in         : OUT  std_logic := '0';

    level_in       : in   integer RANGE 0 TO (2**14) := 0;    
    enable_in      : OUT  std_logic := '0';
    

    data_out       : OUT std_logic_vector(31 downto 0) := (others => '0');
    clk_out        : OUT  std_logic := '0';
    enable_out     : OUT  std_logic := '0';
    
    space_out      : IN  integer RANGE 0 TO (2**14) := 2**14;
   

    emif_avl_write_addr      : OUT std_logic_vector(26 DOWNTO 0) := (others => '0');
    emif_avl_write_data      : OUT std_logic_vector(31 DOWNTO 0) := (others => '0');
    emif_avl_write_req       : OUT std_logic := '0';
    emif_avl_write_ready     : IN  std_logic := '0';

    emif_avl_read_addr       : OUT std_logic_vector(26 DOWNTO 0) := (others => '0');
    emif_avl_read_data       : IN  std_logic_vector(31 DOWNTO 0) := (others => '0');
    emif_avl_read_req        : OUT std_logic := '0';
    emif_avl_read_ready      : IN  std_logic := '0';

    emif_avl_clk             : IN  std_logic := '0'
    
  );
END dram_buffer;

ARCHITECTURE dram_buffer_beh OF dram_buffer IS
  SUBTYPE addr_type IS integer range 0 to 2**27 - 1;
  
  SIGNAL  write_addr   : addr_type := 0;
  SIGNAL  read_addr    : addr_type := 0;
  
  SIGNAL  write_enable : std_logic := '0';
  SIGNAL  read_enable  : std_logic := '0';
BEGIN
  clk_in  <= emif_avl_clk;
  clk_out <= emif_avl_clk;

  buffer_in: PROCESS
  BEGIN
    WAIT UNTIL rising_edge(emif_avl_clk);

    emif_avl_write_addr <= std_logic_vector(to_unsigned(write_addr, emif_avl_write_addr'length));
    emif_avl_write_data <= data_in;
    
    IF write_enable = '1' THEN
      emif_avl_write_req  <= '1';
      
      IF emif_avl_write_ready = '1' THEN
        enable_in <= '1';
        
        IF  write_addr < (write_addr'high - 3) THEN
          write_addr <= write_addr + 4;
        ELSE
          write_addr <= 0;
        END IF;
      
        IF level_in < 2 THEN
          emif_avl_write_req  <= '0';
          write_enable        <= '0';
        END IF;
      END IF;
    ELSE
      enable_in <= '0';

      IF level_in > 512 THEN
        write_enable  <= '1';
      END IF;      
    END IF;
  END PROCESS;
  
  buffer_out: PROCESS
  BEGIN
    WAIT UNTIL rising_edge(emif_avl_clk);

    emif_avl_read_addr <= std_logic_vector(to_unsigned(read_addr, emif_avl_read_addr'length));
    data_out           <= emif_avl_read_data;
    
    IF read_enable = '1' THEN
      emif_avl_read_req  <= '1';
      
      IF emif_avl_read_ready = '1' THEN
        enable_out <= '1';
        
        IF read_addr < (read_addr'high - 3) THEN
          read_addr <= read_addr + 4;
        ELSE
          read_addr <= 0;
        END IF;
        
        IF (space_out < 2) or (read_addr = write_addr)THEN
          emif_avl_read_req  <= '0';
          read_enable        <= '0';
        END IF;
      END IF;
    ELSE
      enable_out <= '0';

      IF (space_out > 512) and ((write_addr - read_addr) > 511) THEN
        read_enable  <= '1';
      END IF;      
    END IF;
  END PROCESS;
  
END dram_buffer_beh;
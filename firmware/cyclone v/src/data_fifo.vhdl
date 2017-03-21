LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.numeric_std.all;

ENTITY data_fifo IS
  PORT(
    data_write      : IN  std_logic_vector(31 downto 0);
    clk_write       : IN  std_logic := '0';
    write_enable    : IN  std_logic := '0';
    
    data_read       : OUT std_logic_vector(31 downto 0) := (others => '0');
    clk_read        : IN  std_logic := '0';
    read_enable     : IN  std_logic := '0';
    
    level           : OUT integer RANGE 0 TO (2**14) := 0;
    space           : OUT integer RANGE 0 TO (2**14) := 2**14
  );
END data_fifo;

ARCHITECTURE data_fifo_beh OF data_fifo IS
  SUBTYPE fifo_element IS std_logic_vector(31 downto 0); 
  
  TYPE    fifo_buffer_type IS array (0 TO (2**14 - 1)) of fifo_element;
  SUBTYPE index_type  IS integer RANGE 0 TO (2*fifo_buffer_type'length - 1);
  
  SIGNAL  fifo_buffer  : fifo_buffer_type := (others => (others => '0'));
  SIGNAL  write_offset : index_type := 0;
  SIGNAL  read_offset  : index_type := 0;
  
  SIGNAL  fifo_level   : integer RANGE 0 TO (2**14) := 0;
  SIGNAL  fifo_space   : integer RANGE 0 TO (2**14) := fifo_buffer_type'length;
BEGIN

  write_fifo: PROCESS
  BEGIN
    WAIT UNTIL rising_edge(clk_write);
    
    IF write_enable = '1' THEN
      IF fifo_space > 0 THEN
        fifo_buffer(write_offset mod fifo_buffer'length) <= data_write;
        
        IF write_offset < write_offset'high THEN
          write_offset <= write_offset + 1;
        ELSE
          write_offset <= 0;
        END IF;
        
        fifo_space <= write_offset - read_offset - 1;
      ELSE
        fifo_space <= write_offset - read_offset;
      END IF;
    ELSE
      fifo_space <= write_offset - read_offset;
    END IF;
  END PROCESS;

  read_fifo_fifo: PROCESS
  BEGIN
    WAIT UNTIL rising_edge(clk_read);
    
    data_read <= fifo_buffer(read_offset mod fifo_buffer'length);
    
    IF read_enable = '1' THEN
      IF fifo_level > 0 THEN
        IF read_offset < read_offset'high THEN
          read_offset <= read_offset + 1;
        ELSE
          read_offset <= 0;
        END IF;
        
        fifo_level <= write_offset - read_offset - 1;
      ELSE
        fifo_level <= write_offset - read_offset;
      END IF;
    ELSE
      fifo_level <= write_offset - read_offset;
    END IF;
  END PROCESS;

  level     <= fifo_level;
  space     <= fifo_space;
  
END data_fifo_beh;
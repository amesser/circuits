LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.numeric_std.all;

ENTITY user_io IS
  PORT(
    EXTCLK    : IN std_logic := '0';
    
    USER_IO_A : INOUT std_logic_vector (15 downto 0) := (others => 'Z');
    USER_IO_B : INOUT std_logic_vector (15 downto 0) := (others => 'Z');
    USER_IO_C : INOUT std_logic_vector (15 downto 0) := (others => 'Z');

    DATA_OUT      : OUT std_logic_vector   (31 downto 0) := (others => '0');
    DATA_CLK      : OUT std_logic := '0';
    DATA_ENABLE   : OUT std_logic := '1';
    
    DATA_SPACE    : IN  integer RANGE 0 TO 2**14
    );
END user_io;

ARCHITECTURE user_io_beh OF user_io IS
  SIGNAL state : std_logic := '0';
  SIGNAL reg   : std_logic_vector(15 downto 0) := (others => '0');
BEGIN  
  stream_data : PROCESS
  BEGIN
    WAIT UNTIL rising_edge(EXTCLK);

    DATA_CLK <= state;    
    
    IF state = '0' THEN
      reg      <= USER_IO_B;
      state    <= '1';
    ELSE
      DATA_OUT <= USER_IO_B & reg;
      state    <= '0'; 
    END IF;
    
    IF DATA_SPACE > 0 THEN
      USER_IO_A  <= USER_IO_B;
      USER_IO_C  <= USER_IO_B;
    ELSE
      USER_IO_A  <= (others => '0');
      USER_IO_C  <= (others => '1');
    END IF;
  END PROCESS; 
END user_io_beh;
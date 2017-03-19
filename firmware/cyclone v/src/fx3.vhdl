LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_arith.all;
USE ieee.numeric_std.all;

ENTITY cypress_slave_fifo IS
  PORT(
    PCLK   : OUT   std_logic := '0';
    SLCSn  : OUT   std_logic := '1';
    PKTEND : OUT   std_logic := '0';
    FLAG   : IN    std_logic_vector(3 DOWNTO 0);
    A      : OUT   std_logic_vector(1 DOWNTO  0) := (others => '0');
    D      : INOUT std_logic_vector(31 DOWNTO 0) := (others => 'Z');
    SLWRn  : OUT   std_logic := '1';
    SLRDn  : OUT   std_logic := '1';
    SLOEn  : OUT   std_logic := '1';
    
    CLK           : IN    std_logic := '0';
    STREAM_CLK    : OUT    std_logic := '0';
    STREAM_DATA   : IN    std_logic_vector(31 downto 0) := (others => '0'); 
    STREAM_LEVEL  : IN    integer range 0 to (2**14); 
    STREAM_ENABLE : OUT   std_logic := '0'
  );
END cypress_slave_fifo;

ARCHITECTURE beh of cypress_slave_fifo IS  
  SIGNAL selector   : integer range 0 to 3 := 0;
  SIGNAL state      : integer range 0 to 4 := 0;
  SIGNAL read_req   : boolean := true;
BEGIN

  PCLK       <= CLK;
  STREAM_CLK <= CLK;
    
  sched: PROCESS
  BEGIN
    WAIT until rising_edge(CLK);
        
    IF state = 0 THEN
      IF (flag(selector) = '1') and STREAM_LEVEL > 512 THEN
        state <=  1;
        SLCSn <= '0';
        A     <= std_logic_vector(to_unsigned(selector, A'length));
      ELSIF selector < 3 THEN
        selector <= selector + 1;
      ELSE
        selector <= 0;
      END IF;
    ELSIF state = 1 THEN
      IF read_req THEN
        SLRDn        <= '0';
        SLOEn        <= '0';
      ELSE
        SLWRn <= '0';
      END IF;
      
      state    <= 2;
    ELSIF state = 2 THEN
      IF flag(selector) = '1' THEN
        STREAM_ENABLE <= '1';
        D <= STREAM_DATA;
      ELSE
        STREAM_ENABLE <= '0';
        SLWRn <= '1';
        SLRDn <= '1';
        SLOEn <= '1';
        D     <= (others => 'Z');
        state <= 0;
      END IF;
    END IF;
  END PROCESS;

END beh;
    
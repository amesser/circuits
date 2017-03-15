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
    AVL_RDATA     : IN    std_logic_vector(31 downto 0) := (others => '0'); 
    AVL_WDATA     : OUT   std_logic_vector(31 downto 0) := (others => '0'); 
    AVL_READ_REQ  : OUT   std_logic := '0';
    AVL_WRITE_REQ : OUT   std_logic := '0';
    AVL_ADDR      : OUT   std_logic_vector(26 downto 0) := (others => '0')
  );
END cypress_slave_fifo;

ARCHITECTURE beh of cypress_slave_fifo IS
  SUBTYPE fifo_addr IS integer range 0 to  2**(26-1);
  TYPE fifo_addrs IS array (0 TO 3) of fifo_addr;
  
  SIGNAL avalon_addr: fifo_addrs := (0,0,0,0);
  SIGNAL selector   : integer range 0 to 3 := 0;
  SIGNAL state      : integer range 0 to 4 := 0;
  SIGNAL read_req   : boolean := true;
BEGIN

  PCLK <= CLK;
  
  PROCESS (selector)
  BEGIN
    read_req <= (selector mod 2) = 0;
  END PROCESS;
  
  sched: PROCESS
  BEGIN
    WAIT until rising_edge(CLK);
    
    AVL_ADDR <= std_logic_vector(to_unsigned(avalon_addr(selector), AVL_ADDR'length));
    
    IF state = 0 THEN
      IF flag(selector) = '1' THEN
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
        AVL_READ_REQ <= '1';
      ELSE
        SLWRn <= '0';
      END IF;
      
      state    <= 2;
    ELSIF state = 2 THEN
      IF flag(selector) = '1' THEN
        IF read_req THEN
          AVL_WDATA     <= D;
          AVL_WRITE_REQ <= '1';
        ELSE
          D <= AVL_RDATA;
        END IF;
        
        IF avalon_addr(selector) < avalon_addr(selector)'high THEN
          avalon_addr(selector) <= avalon_addr(selector) + 1;
        ELSE
          avalon_addr(selector) <= 0;
        END IF;
      ELSE
        AVL_WRITE_REQ <= '0';
        AVL_READ_REQ <= '0';
        SLWRn <= '1';
        SLRDn <= '1';
        SLOEn <= '1';
        D     <= (others => 'Z');
        state <= 0;
      END IF;
    END IF;
  END PROCESS;

END beh;
    
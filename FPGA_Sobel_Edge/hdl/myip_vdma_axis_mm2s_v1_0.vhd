library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;
use IEEE.STD_LOGIC_ARITH.all;

entity myip_vdma_axis_mm2s_v1_0 is
	generic (
		-- Users to add parameters here
        FIFO_DEPTH : integer := 640;
		-- User parameters ends
		-- Do not modify the parameters beyond this line

        
		-- Parameters of Axi Slave Bus Interface S00_AXIS
		C_S00_AXIS_TDATA_WIDTH	: integer	:= 32
	);
	port (
		-- Users to add ports here
		clk_video : in std_logic;   --27MHz
		reset_active_low : in std_logic;-- := '1';--active low
		en_display_active_high : in std_logic;-- := '1'; --active high
        video_data_out : out std_logic_vector(23 downto 0);--(C_S00_AXIS_TDATA_WIDTH-1 downto 0);
        hsync    : out std_logic;
        vsync    : out std_logic;
        
--        vdma_axis_clk : out std_logic;
--        axis_slave_valid : out std_logic;
		-- User ports ends
		-- Do not modify the ports beyond this line


		-- Ports of Axi Slave Bus Interface S00_AXIS
		s00_axis_aclk	: in std_logic;  --100MHz
		s00_axis_aresetn	: in std_logic;
		s00_axis_tready	: out std_logic;
		s00_axis_tdata	: in std_logic_vector(C_S00_AXIS_TDATA_WIDTH-1 downto 0);
		s00_axis_tstrb	: in std_logic_vector((C_S00_AXIS_TDATA_WIDTH/8)-1 downto 0);
		s00_axis_tlast	: in std_logic;
		s00_axis_tvalid	: in std_logic;
		
		s00_axis_tuser	: in std_logic 
	);
end myip_vdma_axis_mm2s_v1_0;

architecture arch_imp of myip_vdma_axis_mm2s_v1_0 is

	-- component declaration
	component myip_vdma_axis_mm2s_v1_0_S00_AXIS is
		generic (
		FIFO_DEPTH : integer := 640;
		C_S_AXIS_TDATA_WIDTH	: integer	:= 32
		);
		port (
            clk_video  : IN  std_logic;
--            reset_active_low : IN  std_logic;
--              sync_vga_en_flag : in std_logic;
           h_blank_vga : in  std_logic;
           v_blank_vga : in  std_logic;
            video_data_out : out std_logic_vector(C_S00_AXIS_TDATA_WIDTH-1 downto 0);
            en_display_active_high : in std_logic;
            cnt_vga_en_o : out std_logic;
--		axis_slave_data_o : out std_logic_vector(C_S_AXIS_TDATA_WIDTH-1 downto 0);
--		axis_tready_vga : in std_logic;
--		sync_vga_start_in : in std_logic;
--        cnt_h_sync_vga :in integer range 0 to 857;
--        cnt_v_sync_vga :in integer range 0 to 524;
--		axis_slave_valid : out std_logic;
		S_AXIS_ACLK	: in std_logic;
		S_AXIS_ARESETN	: in std_logic;
		S_AXIS_TREADY	: out std_logic;
		S_AXIS_TDATA	: in std_logic_vector(C_S_AXIS_TDATA_WIDTH-1 downto 0);
		S_AXIS_TSTRB	: in std_logic_vector((C_S_AXIS_TDATA_WIDTH/8)-1 downto 0);
		S_AXIS_TLAST	: in std_logic;
		S_AXIS_TVALID	: in std_logic
		);
	end component myip_vdma_axis_mm2s_v1_0_S00_AXIS;
	
	component VGA_count
        Port ( 
                      clk_video  : IN  std_logic;
                    rst_system : IN  std_logic;
                   
--                    f_video_en : IN std_logic;
--                    f0_vga_en  : IN std_logic;
                    cnt_vga_en : IN std_logic;
--                    black_vga_en :inout  std_logic;
                    cnt_h_sync_vga :inout integer range 0 to 857;
                    cnt_v_sync_vga :inout integer range 0 to 524;
                    sync_vga_en : inout  std_logic;
                    h_sync_vga : out  std_logic;
                    v_sync_vga : out  std_logic;
                    h_blank_vga : out  std_logic;
                    v_blank_vga : out  std_logic;
                    
                    axis_start	: in std_logic
             );
    
    end component;
    
    
   signal h_blank_vga :  std_logic; --when LOW is active
   signal v_blank_vga :  std_logic; --when LOW is active

    signal cnt_vga_en : std_logic := '0'; -- to ctrl start vga sync count
--    signal cnt_vga_en_t : std_logic:='0'; --Field 0
--    signal axis_tready_vga : std_logic := '0';

--signal axis_slave_data_o_i : std_logic_vector(C_S00_AXIS_TDATA_WIDTH-1 downto 0);
 signal cnt_h_sync_vga : integer range 0 to 857;
 signal cnt_v_sync_vga : integer range 0 to 524;
 signal sync_vga_en_flag :   std_logic;

signal axis_tuser_flag : std_logic := '0';
 -----AXIs FIFO------------------------------------------------------------
-- constant array_range : integer := 256;--8;
-- constant array_range_cnv :  integer := 8;--3; -- array_range total value's  Binary size
-- type fifo_arrays is array (0 to array_range-1) of std_logic_vector(C_S00_AXIS_TDATA_WIDTH-1 downto 0); --0~7 (31 downt 0)
-- signal fifo_array : fifo_arrays;
-- signal fi_index : integer range 0 to array_range-1 := 0;   
-- signal fo_index : integer range 0 to array_range-1 := 0;
-- signal fifo_full_flag : std_logic:='0'; -- when fifo full that to HIGH
 
  signal video_data_out_buf : std_logic_vector(C_S00_AXIS_TDATA_WIDTH-1 downto 0);
  --------------------------------------------------------------------------
begin

--axis_slave_data_o <= axis_slave_data_o_i(23 downto 0);
-- Instantiation of Axi Bus Interface S00_AXIS
myip_vdma_axis_mm2s_v1_0_S00_AXIS_inst : myip_vdma_axis_mm2s_v1_0_S00_AXIS
	generic map (
	   FIFO_DEPTH => FIFO_DEPTH,
		C_S_AXIS_TDATA_WIDTH	=> C_S00_AXIS_TDATA_WIDTH
	)
	port map (
	  clk_video=>clk_video,
--	  reset_active_low => reset_active_low,
--	  sync_vga_en_flag =>   sync_vga_en_flag ,
	  h_blank_vga => h_blank_vga,
      v_blank_vga => v_blank_vga,
      video_data_out=>video_data_out_buf,
      
      en_display_active_high=>en_display_active_high ,
      cnt_vga_en_o => cnt_vga_en,
--	   axis_slave_data_o => axis_slave_data_o_i,
--	  axis_tready_vga => axis_tready_vga,
--	   sync_vga_start_in => sync_vga_en_out, -- start in HIGH
--	  cnt_h_sync_vga =>cnt_h_sync_vga,
--       cnt_v_sync_vga =>cnt_v_sync_vga,
       
--	   axis_slave_valid => axis_slave_valid ,
		S_AXIS_ACLK	=> s00_axis_aclk,
		S_AXIS_ARESETN	=> s00_axis_aresetn,
		S_AXIS_TREADY	=> s00_axis_tready,
		S_AXIS_TDATA	=> s00_axis_tdata,
		S_AXIS_TSTRB	=> s00_axis_tstrb,
		S_AXIS_TLAST	=> s00_axis_tlast,
		S_AXIS_TVALID	=> s00_axis_tvalid
	);



vgacount: VGA_count
	Port map( 
        clk_video  =>clk_video ,
        rst_system =>reset_active_low,
      
--        f_video_en =>f_video_en_t,
--        f0_vga_en =>f0_vga_en_t,
        cnt_vga_en => cnt_vga_en,--en_display_active_high,
--        black_vga_en =>black_vga_en,
        cnt_h_sync_vga =>cnt_h_sync_vga,
        cnt_v_sync_vga =>cnt_v_sync_vga,
        sync_vga_en =>sync_vga_en_flag,
        h_sync_vga =>hsync,
        v_sync_vga =>vsync,
         h_blank_vga => h_blank_vga,
         v_blank_vga => v_blank_vga,
         
         axis_start => axis_tuser_flag
         );
	-- Add user logic here

    video_data_out <= video_data_out_buf(23 downto 0)-- output video 
    when ( cnt_h_sync_vga >= 0 and cnt_h_sync_vga < 640 and cnt_v_sync_vga >= 0 and cnt_v_sync_vga < 480)
    else (others=>'0');


process(s00_axis_aclk,s00_axis_aresetn)
begin
    if s00_axis_aresetn = '0' then
       axis_tuser_flag <= '0';
	elsif (rising_edge (s00_axis_aclk)) then -- AXI CLOCK
	   if s00_axis_tuser = '1' then
	       axis_tuser_flag <= '1';
	   end if;
	end if;
end process;

---- axis protocle
---- it bypass to s00_axis_tready
--axis_tready_vga <= sync_vga_en_flag and (not fifo_full_flag);--(not h_blank_vga) and (not v_blank_vga);

--process(s00_axis_aclk)--100Mhz
--begin
--    if reset_active_low = '0' then
--        fi_index <= 0;
--        for i in 0 to array_range-1 loop
--             fifo_array(i) <= (others=>'0');
--        end loop;
--	elsif (rising_edge (s00_axis_aclk)) then
--	   if s00_axis_tvalid='1' and axis_tready_vga='1' then
--	       fifo_array(fi_index) <= s00_axis_tdata;
	       
--	       if fi_index = array_range-1 then
--	           fi_index <= 0;
--	       else
--	           fi_index <= fi_index + 1;
--	       end if;
--	   end if;
--	end if;
--end process;


---- vga read fifo
--process(clk_video)--27Mhz
--begin
--	if reset_active_low = '0' then
--        fo_index <= 0;
--	elsif (rising_edge (clk_video)) then
--	   if sync_vga_en_flag='1' and (h_blank_vga='0') and (v_blank_vga='0') then
--	       video_data_out_buf <= fifo_array(fo_index);
	       
--	       if fo_index = array_range-1 then
--	           fo_index <= 0;
--	       else
--	           fo_index <= fo_index + 1;
--	       end if;
--	   end if;
--	end if;
--end process;


---- fifo ctrl 
---- let fi_index = fo_index - 1 that data full
--fifo_full_flag <= '0';--'1' when conv_std_logic_vector(fi_index,array_range_cnv) = (conv_std_logic_vector(fo_index,array_range_cnv)-1) else '0';
                  
    
--	-- User logic ends

end arch_imp;

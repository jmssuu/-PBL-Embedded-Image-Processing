library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.NUMERIC_STD.all;
use IEEE.STD_LOGIC_ARITH.all;
use work.HOG_PACK.all;

library UNISIM;
use UNISIM.VComponents.all;

entity main_code is
port(
	clk_video : in std_logic;
	rst_system_i: in std_logic;
	
	-----design wrapper----------------------------------------------
	DDR_addr : inout STD_LOGIC_VECTOR ( 14 downto 0 );
    DDR_ba : inout STD_LOGIC_VECTOR ( 2 downto 0 );
    DDR_cas_n : inout STD_LOGIC;
    DDR_ck_n : inout STD_LOGIC;
    DDR_ck_p : inout STD_LOGIC;
    DDR_cke : inout STD_LOGIC;
    DDR_cs_n : inout STD_LOGIC;
    DDR_dm : inout STD_LOGIC_VECTOR ( 3 downto 0 );
    DDR_dq : inout STD_LOGIC_VECTOR ( 31 downto 0 );
    DDR_dqs_n : inout STD_LOGIC_VECTOR ( 3 downto 0 );
    DDR_dqs_p : inout STD_LOGIC_VECTOR ( 3 downto 0 );
    DDR_odt : inout STD_LOGIC;
    DDR_ras_n : inout STD_LOGIC;
    DDR_reset_n : inout STD_LOGIC;
    DDR_we_n : inout STD_LOGIC;
    FIXED_IO_ddr_vrn : inout STD_LOGIC;
    FIXED_IO_ddr_vrp : inout STD_LOGIC;
    FIXED_IO_mio : inout STD_LOGIC_VECTOR ( 53 downto 0 );
    FIXED_IO_ps_clk : inout STD_LOGIC;
    FIXED_IO_ps_porb : inout STD_LOGIC;
    FIXED_IO_ps_srstb : inout STD_LOGIC;
    btns : in STD_LOGIC_VECTOR ( 3 downto 0 );
    leds : out STD_LOGIC_VECTOR ( 7 downto 0 );
    sws : in STD_LOGIC_VECTOR ( 7 downto 0 );
    ---------------------------------------------------
    
    
-------------------videoin---------------------------
	data_video	 : in std_logic_vector(7 downto 0);
----------------vga1-----------------------------
	r_out : out std_logic_vector(3 downto 0);
	g_out : out std_logic_vector(3 downto 0);
	b_out : out std_logic_vector(3 downto 0);
	h_sync_vga : out std_logic;
	v_sync_vga : out std_logic;		
----------------vga2-----------------------------
    vga2_r_out : out std_logic_vector(7 downto 0);
    vga2_g_out : out std_logic_vector(7 downto 0);
    vga2_b_out : out std_logic_vector(7 downto 0);
    h_sync_vga2 : out std_logic;
    v_sync_vga2 : out std_logic;  
        
----------sw-----------------------------		
--	sw6 :in std_logic;
--	sw5 :in std_logic;
--	sw4 :in std_logic; 
--	sw3 :in std_logic;
--	sw2 :in std_logic;  
--	sw1 :in std_logic;
--	sw0 :in std_logic;
	
----------HOG DEBUG-----------------------	
--	BlockOut : OUT BlockBin;
--	BlockDataValidOut : OUT std_logic;
----------I2C-----------------------------		
	sda : inout std_logic;
	scl : inout std_logic
);
end main_code;

architecture Behavioral of main_code is

signal rst_system : std_logic := '0';
--BRAM counter----------------------------------
--component blk_mem_gen_v7_3 is
--  PORT (
--    clka : IN STD_LOGIC;
--    ena : IN STD_LOGIC;
--    addra : IN STD_LOGIC_VECTOR(18 DOWNTO 0);
--    douta : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
--  );
--END component;


--signal BR_Enable_W 		: std_logic;
--signal BR_WriteHigh_W	: std_logic_vector(0 downto 0);
--signal BR_DataIn_W 		: std_logic_vector(7 downto 0);
--signal BR_Address_W 	: integer range 0 to (524287-1);		
--signal BR_Address_std_W	: std_logic_vector(18 downto 0);
--signal BR_DataOut_W		: std_logic_vector(7 downto 0);

signal BR_Enable_R 		: std_logic;
signal BR_WriteHigh_R	: std_logic_vector(0 downto 0);
signal BR_DataIn_R 		: std_logic_vector(7 downto 0);
signal BR_Address_R 	: integer range 0 to (524287-1);		
signal BR_Address_std_R	: std_logic_vector(18 downto 0);
signal BR_DataOut_R		: std_logic_vector(7 downto 0);

signal Bram_out : std_logic_vector(7 downto 0);

signal data_video_i: std_logic_vector ((8-1) downto 0);
--BRAM counter----------------------------------

signal r_vga:std_logic_vector(3 downto 0);
signal g_vga:std_logic_vector(3 downto 0);
signal b_vga:std_logic_vector(3 downto 0);

--video-in--parameter----------------------------------------------------------------------------------------------------
signal EAV_new : std_logic_vector(1 downto 0):="ZZ";
signal SAV_old : std_logic_vector(1 downto 0):="ZZ";
signal EAV_state : std_logic_vector(1 downto 0):="00";
signal SAV_state : std_logic_vector(1 downto 0):="00";
signal SAV_en : std_logic:='0';

signal cnt_video_hsync : integer range 0 to 1715:=0;

signal f_video_en : std_logic:='Z'; --Field
signal cnt_video_en : std_logic:='0';
signal cnt_vga_en : std_logic:='0';
signal buf_vga_en : std_logic:='0';

signal cnt_h_sync_vga : integer range 0 to 857:=0;
signal cnt_v_sync_vga : integer range 0 to 524:=0;
signal black_vga_en : std_logic:='0';
signal sync_vga_en : std_logic:='0';
signal f0_vga_en : std_logic:='0'; --Field 0
--video-in--parameter----------------------------------------------------------------------------------------------------

--VGA-8bit-------------------------------------------------------------------------------------------------------
--state-------------------------------------------------------------------------------------------------------
signal image_data_enable : std_logic:='0';

signal buf_data_state : std_logic_vector(1 downto 0):="00";
--state-------------------------------------------------------------------------------------------------------

--VGA-8bit-------------------------------------------------------------------------------------------------------
signal buf_vga_state : std_logic_vector(1 downto 0):="00";

type Array_Y is ARRAY (integer range 0 to 639) of std_logic_vector(7 downto 0);
signal buf_vga_Y : Array_Y;
signal buf_vga_Y_buf : std_logic_vector(7 downto 0);
signal buf_vga_R, buf_vga_G, buf_vga_B : Array_Y;

signal buf_vga_Y_in_cnt : integer range 0 to 639:=0;
signal buf_vga_Y_out_cnt : integer range 0 to 639:=639;

signal buf_vga_bram : Array_Y;

signal YCR_C1 : std_logic_vector(11 downto 0):=x"5a1";--constant1 of YUV convert R, 1.4075(d) * 1024(d) = 5a1(H)
signal YCG_C1 : std_logic_vector(11 downto 0):=x"161";--constant1 of YUV convert G, 0.3455(d) * 1024(d) = 161(H)
signal YCG_C2 : std_logic_vector(11 downto 0):=x"2de";--constant2 of YUV convert G, 0.7169(d) * 1024(d) = 2de(H)
signal YCB_C1 : std_logic_vector(11 downto 0):=x"71d";--constant1 of YUV convert B, 1.7790(d) * 1024(d) = 71d(H)
signal Cb_register, Cr_register : std_logic_vector(7 downto 0);

signal YCR : std_logic_vector(19 downto 0);
signal YCG : std_logic_vector(19 downto 0);
signal YCB : std_logic_vector(19 downto 0);

component design_1_wrapper is
  port (
  
       DDR_addr : inout STD_LOGIC_VECTOR ( 14 downto 0 );
     DDR_ba : inout STD_LOGIC_VECTOR ( 2 downto 0 );
     DDR_cas_n : inout STD_LOGIC;
     DDR_ck_n : inout STD_LOGIC;
     DDR_ck_p : inout STD_LOGIC;
     DDR_cke : inout STD_LOGIC;
     DDR_cs_n : inout STD_LOGIC;
     DDR_dm : inout STD_LOGIC_VECTOR ( 3 downto 0 );
     DDR_dq : inout STD_LOGIC_VECTOR ( 31 downto 0 );
     DDR_dqs_n : inout STD_LOGIC_VECTOR ( 3 downto 0 );
     DDR_dqs_p : inout STD_LOGIC_VECTOR ( 3 downto 0 );
     DDR_odt : inout STD_LOGIC;
     DDR_ras_n : inout STD_LOGIC;
     DDR_reset_n : inout STD_LOGIC;
     DDR_we_n : inout STD_LOGIC;
     FIXED_IO_ddr_vrn : inout STD_LOGIC;
     FIXED_IO_ddr_vrp : inout STD_LOGIC;
     FIXED_IO_mio : inout STD_LOGIC_VECTOR ( 53 downto 0 );
     FIXED_IO_ps_clk : inout STD_LOGIC;
     FIXED_IO_ps_porb : inout STD_LOGIC;
     FIXED_IO_ps_srstb : inout STD_LOGIC;
--     axi_data_i_0 : in STD_LOGIC_VECTOR ( 31 downto 0 );
--     axi_data_o_0 : out STD_LOGIC_VECTOR ( 31 downto 0 );
     
  
     ----------------------------------------------------------------------------
--     btns : in STD_LOGIC_VECTOR ( 3 downto 0 );
--     hog_data_0 : in STD_LOGIC_VECTOR ( 8063 downto 0 );
     leds : out STD_LOGIC_VECTOR ( 7 downto 0 );
     sws : in STD_LOGIC_VECTOR ( 7 downto 0 );
     
     ----AXI VDMA------------------------------------------------------------------- 
     
          
--     vid_io_in_0_active_video : in STD_LOGIC;
--     vid_io_in_0_data : in STD_LOGIC_VECTOR ( 23 downto 0 );
--     vid_io_in_0_field : in STD_LOGIC;
--     vid_io_in_0_hblank : in STD_LOGIC;
--     vid_io_in_0_hsync : in STD_LOGIC;
--     vid_io_in_0_vblank : in STD_LOGIC;
--     vid_io_in_0_vsync : in STD_LOGIC;
     vid_io_in_clk_0 : in STD_LOGIC;
     
     vid_data_1 : out STD_LOGIC_VECTOR ( 23 downto 0 );--( 31 downto 0 );--( 23 downto 0 );
     vid_hsync_1 : out STD_LOGIC;
     vid_vsync_1 : out STD_LOGIC;
     
     vid_data_2 : out STD_LOGIC_VECTOR ( 23 downto 0 );--( 31 downto 0 );--( 23 downto 0 );
     vid_hsync_2 : out STD_LOGIC;
     vid_vsync_2 : out STD_LOGIC;
     
     reset_active_low_0 : in STD_LOGIC;
     en_display_active_high_0 : in STD_LOGIC;
     ---VDMA in----------------------------------------------------------
     AXIsToVDMA_h_blank_vga_0 : in STD_LOGIC;
     AXIsToVDMA_en_0 : in STD_LOGIC;
     AXIsToVDMA_v_blank_vga_0 : in STD_LOGIC;
     AXIsToVDMA_video_data_in_0 : in STD_LOGIC_VECTOR ( 31 downto 0 )
--     vtc0_clken_0 : in STD_LOGIC;
--     vtc0_gen_clken_0 : in STD_LOGIC;
--     vtc0_resetn_0 : in STD_LOGIC;
     ---------------------------------------------------------
--     vtiming_in_0_active_video : in STD_LOGIC;
--     vtiming_in_0_field : in STD_LOGIC;
--     vtiming_in_0_hblank : in STD_LOGIC;
--     vtiming_in_0_hsync : in STD_LOGIC;
--     vtiming_in_0_vblank : in STD_LOGIC;
--     vtiming_in_0_vsync : in STD_LOGIC;
--      hsync_out_0 : out STD_LOGIC;
--      vsync_out_0 : out STD_LOGIC;
--    fid_0 : in STD_LOGIC;
--    vid_io_out_reset_0 : in STD_LOGIC
--    aresetn_0 : in STD_LOGIC;
--axis_enable_0 : in STD_LOGIC


    

  );
end component;


     signal axi_data_i_0 : STD_LOGIC_VECTOR ( 31 downto 0 );
     signal axi_data_o_0 : STD_LOGIC_VECTOR ( 31 downto 0 );
     signal hog_data_0 : STD_LOGIC_VECTOR ( 8063 downto 0 );
     
     signal BlocksInRow_cnt : integer range 0 to 6 := 0;            --HOG count row = 7
     signal BlocksInCol_cnt : integer range 0 to 14 := 0;           --HOG count col = 15
     
     --to DMA vga-----------------------------------------------
--     signal vid_io_in_0_active_video : STD_LOGIC;
     signal vid_io_in_0_data : STD_LOGIC_VECTOR ( 23 downto 0 );
--     signal vid_io_in_0_field : STD_LOGIC;
--     signal vid_io_in_0_hblank : STD_LOGIC;
--     signal vid_io_in_0_hsync : STD_LOGIC;
--     signal vid_io_in_0_vblank : STD_LOGIC;
--     signal vid_io_in_0_vsync : STD_LOGIC;
     signal vtc_hsync_out_0 : STD_LOGIC;
     signal vtc_vsync_out_0 : STD_LOGIC;
     -----video VDMA in------------------------------------------------
     signal AXIsToVDMA_video_data_in_0 : STD_LOGIC_VECTOR ( 31 downto 0 );
     
     --out vga-----------------------------------------------
     signal vid_data_1,vid_data_2 : STD_LOGIC_VECTOR ( 23 downto 0 );--( 31 downto 0 );--( 23 downto 0 );
     signal vid_hsync_1 : STD_LOGIC;
     signal vid_vsync_1 : STD_LOGIC;
--VGA-8bit-------------------------------------------------------------------------------------------------------
component i2c
Port (
		 clk : IN  std_logic;
		 rst: IN  std_logic;
		 sda : inout std_logic;
		 scl : inout std_logic
);
end component;

component video_in
Port ( 
		clk_video  : IN  std_logic;
		rst_system : IN  std_logic;
		data_video : IN  std_logic_vector(7 downto 0)  ;
		f_video_en : inout std_logic;
		cnt_video_en : inout std_logic;
		cnt_vga_en : inout std_logic ;
		buf_vga_en : inout std_logic ;
		cnt_video_hsync : inout  integer range 0 to 1715;
		f0_vga_en : inout std_logic;
	 
		black_vga_en :inout  std_logic;
		cnt_h_sync_vga :inout integer range 0 to 857;
		cnt_v_sync_vga :inout integer range 0 to 524;
		sync_vga_en : inout  std_logic;
		v_sync_vga : out  std_logic;
		h_sync_vga : out  std_logic;
		h_blank_vga : out  std_logic;
        v_blank_vga : out  std_logic
);
end component;

	signal h_sync_vga_buf : std_logic;
    signal v_sync_vga_buf : std_logic;  
    signal h_blank_vga_buf : std_logic;
    signal v_blank_vga_buf : std_logic;

--component  HOG_Descriptor is
--Port (
--    clk_video  : IN  std_logic;
--    rst_system : IN  std_logic;
--    --**
--    data_video: in std_logic_vector ((8-1) downto 0);
--    image_data_enable : in  std_logic;
--    cnt_h_sync_vga : in integer range 0 to 857;
--    cnt_v_sync_vga : in integer range 0 to 524;
--    buf_vga_Y_out_cnt :in integer range 0 to 639;
    
--    SB_gradient_out_o : out std_logic_vector(10 downto 0):=(others=>'0');
--    SB_angle_out_o : out std_logic_vector(7 downto 0):=(others=>'0');
    
--    startX : in integer range 0 to 639 := 69;
--    startY : in integer range 0 to 479 := 209;
--    BlockOut : OUT BlockBin;
--    BlockDataValidOut : OUT std_logic 
--);
--end component ;
signal BlockOut : BlockBin;
signal BlockDataValidOut : std_logic;

signal SB_gradient_out : std_logic_vector(10 downto 0):=(others=>'0');
signal SB_angle_out : std_logic_vector(7 downto 0):=(others=>'0');


begin

rst_system <= not rst_system_i;

--v_sync_vga<= v_sync_vga_buf;
with sws(1) select
v_sync_vga<= vid_vsync_1 when '1',--"01",
--             vtc_vsync_out_0 when "10",
             v_sync_vga_buf when others;

--h_sync_vga<= h_sync_vga_buf;
with sws(1) select
h_sync_vga<= vid_hsync_1 when '1',--"01",
--             vtc_hsync_out_0 when "10",
             h_sync_vga_buf when others;

with sws(0) select
    r_out<= vid_data_1(7 downto 4) when '1',
            r_vga when others;

with sws(0) select
    g_out<= vid_data_1(15 downto 12) when '1',
            g_vga when others;
            
with sws(0) select
    b_out<= vid_data_1(23 downto 20) when '1',
            b_vga when others;    
            
 vga2_r_out <= vid_data_2(7 downto 0);          
 vga2_g_out <= vid_data_2(15 downto 8);     
 vga2_b_out <= vid_data_2(23 downto 16); 
                              
 vid_io_in_0_data <= data_video_i & data_video_i & data_video_i;
 AXIsToVDMA_video_data_in_0 <= "00000000" & data_video_i & data_video_i & data_video_i;
--block_rom:blk_mem_gen_v7_3
--  PORT map(
--    clka => clk_video,
--    ena => BR_Enable_R,
--    addra => BR_Address_std_R,
--    douta => BR_DataOut_R
--  );
  

design_wrapper:design_1_wrapper 
  port map(
    DDR_addr => DDR_addr,
    DDR_ba => DDR_ba,
    DDR_cas_n => DDR_cas_n,
    DDR_ck_n => DDR_ck_n,
    DDR_ck_p => DDR_ck_p,
    DDR_cke => DDR_cke,
    DDR_cs_n => DDR_cs_n,
    DDR_dm => DDR_dm,
    DDR_dq => DDR_dq,
    DDR_dqs_n => DDR_dqs_n,
    DDR_dqs_p  => DDR_dqs_p,
    DDR_odt  => DDR_odt,
    DDR_ras_n => DDR_ras_n,
    DDR_reset_n  => DDR_reset_n,
    DDR_we_n  => DDR_we_n,
    FIXED_IO_ddr_vrn => FIXED_IO_ddr_vrn,
    FIXED_IO_ddr_vrp => FIXED_IO_ddr_vrp,
    FIXED_IO_mio => FIXED_IO_mio,
    FIXED_IO_ps_clk  => FIXED_IO_ps_clk,
    FIXED_IO_ps_porb  => FIXED_IO_ps_porb,
    FIXED_IO_ps_srstb => FIXED_IO_ps_srstb,
    
--    axi_data_i_0 => axi_data_i_0,
--    axi_data_o_0 => axi_data_o_0,
--    hog_data_0  => hog_data_0,
--    btns => btns,
    leds => leds,
    sws => sws,
    
    vid_io_in_clk_0 => clk_video,
    
--    vid_io_in_0_active_video => sync_vga_en,--active high
--    vid_io_in_0_data => vid_io_in_0_data,
--    vid_io_in_0_field => sws(7),--com LOW--clk_video,
--    vid_io_in_0_hblank => h_blank_vga_buf,
--    vid_io_in_0_hsync => h_sync_vga_buf,--vid_io_in_0_hsync,
--    vid_io_in_0_vblank => v_blank_vga_buf,
--    vid_io_in_0_vsync => v_sync_vga_buf,--vid_io_in_0_vsync,
    
    vid_data_1 => vid_data_1,
    vid_hsync_1 => vid_hsync_1,
    vid_vsync_1 => vid_vsync_1,
    
    vid_data_2 => vid_data_2,
    vid_hsync_2 => h_sync_vga2,
    vid_vsync_2 => v_sync_vga2,
    
    reset_active_low_0 => rst_system,
    en_display_active_high_0 => cnt_vga_en,
    
    ---video VDMA in-------------------------------------------
    AXIsToVDMA_h_blank_vga_0 => h_blank_vga_buf,
    AXIsToVDMA_v_blank_vga_0 => v_blank_vga_buf,
    AXIsToVDMA_en_0 => cnt_vga_en,
    AXIsToVDMA_video_data_in_0 => AXIsToVDMA_video_data_in_0
    --------------------------------------
--    vtc0_clken_0 => sws(7),
--    vtc0_gen_clken_0 => sws(1),
--    vtc0_resetn_0 => rst_system,  --active low
    
--    vtiming_in_0_active_video => '1',
--    vtiming_in_0_field => '1',
--    vtiming_in_0_hblank => h_blank_vga_buf,
--    vtiming_in_0_hsync => h_sync_vga_buf,--vid_io_in_0_hsync,
--    vtiming_in_0_vblank => v_blank_vga_buf,
--    vtiming_in_0_vsync => v_sync_vga_buf,--vid_io_in_0_vsync,
--    hsync_out_0 => vtc_hsync_out_0,
--    vsync_out_0 => vtc_vsync_out_0,
--      fid_0 => sws(7),
--      vid_io_out_reset_0 => rst_system_i--active High
--   aresetn_0 => sws(4),
--    axis_enable_0 => sws(3)
      
  );

i2c_1 :i2c
PORT MAP (
		clk => clk_video,
		rst => rst_system,
		sda => sda,
		scl => scl
);						
VIDEO_IN1 : video_in
PORT MAP (
		clk_video  =>clk_video,
		rst_system =>rst_system,
		data_video =>data_video,
		f_video_en =>f_video_en,
		cnt_video_en =>cnt_video_en,
		cnt_vga_en =>cnt_vga_en,
		buf_vga_en =>buf_vga_en,
		cnt_video_hsync =>cnt_video_hsync,
		f0_vga_en =>f0_vga_en,
	   
		black_vga_en =>black_vga_en,
		cnt_h_sync_vga =>cnt_h_sync_vga,
		cnt_v_sync_vga =>cnt_v_sync_vga,
		sync_vga_en =>sync_vga_en,
		v_sync_vga =>v_sync_vga_buf,--v_sync_vga,
		h_sync_vga =>h_sync_vga_buf,--h_sync_vga 
		h_blank_vga => h_blank_vga_buf,
        v_blank_vga => v_blank_vga_buf            
);			



--HOG_1:HOG_Descriptor
--port map(
--    clk_video  => clk_video,
--    rst_system => rst_system,
--    data_video => data_video_i,
----    data_video => Bram_out,
----    data_video => buf_vga_Y(buf_vga_Y_out_cnt),
--    image_data_enable =>image_data_enable,
--    cnt_h_sync_vga =>cnt_h_sync_vga,
--    cnt_v_sync_vga =>cnt_v_sync_vga,
--    buf_vga_Y_out_cnt =>buf_vga_Y_out_cnt,
    
--    SB_gradient_out_o => SB_gradient_out,
--    SB_angle_out_o => SB_angle_out,
    
--    startX => 30,
--    startY => 30,
--    BlockOut => BlockOut,
--    BlockDataValidOut => BlockDataValidOut
--);
			
			
with sws(6 downto 4) select 
data_video_i <= buf_vga_Y(buf_vga_Y_out_cnt) when "100",
                buf_vga_Y(buf_vga_Y_out_cnt) when "110",
                Bram_out when "010",
                Bram_out when "011",
                buf_vga_Y(buf_vga_Y_out_cnt) when others;
--axi_data_o_0 <= axi_data_i_0;

------- HOG Send --------------------------------------------------------------------------			
--HOG_send:process(rst_system, clk_video, axi_data_i_0)
--    variable axi_data_i_action : std_logic_vector(31 downto 0) := (others=>'0');
--begin
			
--    if rst_system = '0' then
--        BlocksInRow_cnt <= 0;
--        BlocksInCol_cnt <= 0;
--        axi_data_i_action := axi_data_i_0;
        

--        hog_data_0 <= (others=>'0');

                        
--    elsif rising_edge(clk_video) then
    
----        if axi_data_i_action /= axi_data_i_0 then
----            axi_data_i_action := axi_data_i_0;  
           
--            if BlockDataValidOut='1' then
            
----                 for i in 0 to 251 loop
----                    hog_data_0((i+1)*32-1 downto i*32) <= conv_std_logic_vector(i,32);
----                 end loop;
                           
--                for i in 0 to 35 loop
--                    hog_data_0((i+1)*32-1+(1152*BlocksInRow_cnt) downto i*32+(1152*BlocksInRow_cnt)) <= BlockOut(i)(31 downto 0);
----                    hog_data_0((i+1)*36-1+(1296*BlocksInRow_cnt) downto i*36+(1296*BlocksInRow_cnt)) <= BlockOut(i)(35 downto 0);
--                end loop;
                
                
--                if BlocksInRow_cnt = 6 then
--                    BlocksInRow_cnt <= 0;
----                    axi_data_i_action := axi_data_i_0; ------------------------#################test one times
--                    if BlocksInCol_cnt = 14 then --when 15 times = one page
--                        BlocksInCol_cnt <= 0;
--                    else
--                        BlocksInCol_cnt <= BlocksInCol_cnt + 1;
--                    end if;
--                else
--                    BlocksInRow_cnt <= BlocksInRow_cnt + 1;
--                end if;
            
--            end if;
----        end if;
        
--    end if;

--end process;
			
			
--VGA-RGB-9bit----------------------------------------------------------------------------------------------------
VGA_OUT_Control:process(rst_system, clk_video)
begin
if rst_system = '0' then
	r_vga <= "0000";
	g_vga <= "0000";
	b_vga <= "0000";
	buf_vga_Y_out_cnt <= 0;
elsif rising_edge(clk_video) then				
    if ( cnt_h_sync_vga >= 0 and cnt_h_sync_vga < 640 and cnt_v_sync_vga >= 0 and cnt_v_sync_vga < 480) then
	    buf_vga_Y_out_cnt <= buf_vga_Y_out_cnt - 1;
		
		
		
		if sws(6 downto 5) ="10" then 
              b_vga <= SB_gradient_out(3 downto 0);
              g_vga <= SB_gradient_out(7 downto 4);
              r_vga <= SB_gradient_out(10 downto 8) & '0';


        elsif sws(6 downto 5) ="11" then
            r_vga <= SB_angle_out(3 downto 0);
            g_vga <= SB_angle_out(7 downto 4);
            b_vga <= (others=>'0');


        elsif sws(5 downto 4) = "10" then
            b_vga <= SB_gradient_out(3 downto 0);
            g_vga <= SB_gradient_out(7 downto 4);
            r_vga <= SB_gradient_out(10 downto 8) & '0';
        
        elsif sws(5 downto 4) = "11" then
            r_vga <= SB_angle_out(3 downto 0);
            g_vga <= SB_angle_out(7 downto 4);
            b_vga <= (others=>'0');
            
        elsif sws(3) ='1' then
             r_vga <= buf_vga_Y(buf_vga_Y_out_cnt)(7 downto 4);
             g_vga <= buf_vga_Y(buf_vga_Y_out_cnt)(7 downto 4);
             b_vga <= buf_vga_Y(buf_vga_Y_out_cnt)(7 downto 4);  
        elsif sws(2) ='1' then
            r_vga <= Bram_out(7 downto 4);
            g_vga <= Bram_out(7 downto 4);
            b_vga <= Bram_out(7 downto 4);
        else 
            r_vga <= "0000";
            g_vga <= "0000";
            b_vga <= "0000"; 
        end if;       
        
                         
    else
		r_vga <= "0000";
		g_vga <= "0000";
		b_vga <= "0000";
		buf_vga_Y_out_cnt <= 639;
    end if;
end if;
end process;
----VGA-RGB-9bit---------------------------------------------------------------------------------------------------

--################################################ TP Block RAM 307200 ############################################
-- BR_Address_std_W <= CONV_STD_LOGIC_VECTOR(BR_Address_W, 19);
 BR_Address_std_R <= CONV_STD_LOGIC_VECTOR(BR_Address_R, 19);

process(clk_video,rst_system)
begin
    
if rst_system = '0' then
    BR_Enable_R <= '0';          
    BR_Address_R <= 0;  
    Bram_out <= (others => '0');                  
    
elsif rising_edge(clk_video) then     
        
    if buf_vga_en = '1' and ( cnt_h_sync_vga >= 0 and cnt_h_sync_vga < 640 and cnt_v_sync_vga >= 0 and cnt_v_sync_vga < 480) then   
             BR_Enable_R <= '1';                                          
        if BR_Address_R = 307199 then -- address = 640 x 480 = 307200    638x478 = 304964                                 
            BR_Address_R <= 0;
        else     
            BR_Address_R <= BR_Address_R + 1;                     
        end if;   
 
        Bram_out <= BR_DataOut_R(7 downto 0);                        
        
     elsif image_data_enable ='0' then   
        BR_Enable_R <= '0';                 
        BR_Address_R <= BR_Address_R; 
        Bram_out <= "00000000";                              
       
     end if;   
end if;
end process;

-- process(clk_video,rst_system)
-- variable buf_state : std_logic_vector(1 downto 0);
-- begin
--	 if rst_system = '0' then
--		 BR_Enable_R       <= '0';
--		 BR_WriteHigh_R    <= "0";
--		 BR_Address_R      <= 0;	
--		 BR_DataIn_R       <= (others =>'0');
--		 Bram_out          <= "00000000";
--	 elsif rising_edge(clk_video) then
--        if ( cnt_h_sync_vga >= 0 and cnt_h_sync_vga < 640 and cnt_v_sync_vga >= 0 and cnt_v_sync_vga < 480) then
--            BR_Enable_R         <= '1';
--            BR_WriteHigh_R      <= "0";
                            
--            if BR_Address_R = 307199 then -- address = 640 x 480 = 307200	638x478 = 304964							 	
--               BR_Address_R     <= 0;
--            else
--               BR_Address_R     <= BR_Address_R + 1;				 	
--            end if;
                   
--            Bram_out <= BR_DataOut_R(7 downto 0);
--        elsif image_data_enable ='0' then      
--            BR_Enable_R			<= '0';
--            BR_WriteHigh_R		<= (others=>'0');
--            --BR_Address_R  		<= BR_Address_R;
--            BR_DataIn_R			<= (others =>'0');
--            Bram_out <= "00000000";
--        end if;
--    end if;
-- end process;


-- process(clk_video,rst_system)
-- variable buf_state : std_logic_vector(1 downto 0);
-- begin
	-- if rst_system = '0' then
		-- BR_Enable_W			<= '0';
		-- BR_WriteHigh_W		<= "0";
		-- BR_Address_W  		<= 0;
		-- BR_DataIn_W			<= (others =>'0');				
	-- elsif rising_edge(clk_video) then
		-- if ( cnt_h_sync_vga >= 0 and cnt_h_sync_vga < 640 and cnt_v_sync_vga >= 0 and cnt_v_sync_vga < 480) then
			
			-- --###
			-- BR_DataIn_W <= buf_vga_Y(buf_vga_Y_out_cnt);														
			-- --###
			-- if sw0 ='1'then 
				-- BR_Enable_W			<= '1';
				-- BR_WriteHigh_W		<= "1";
			-- elsif sw0 ='0'then 
				-- BR_Enable_W			<= '1';
				-- BR_WriteHigh_W		<= "0";
			-- end if ;

			-- if BR_Address_W = 307199 then -- address = 640 x 480 = 307200	638x478 = 304964							 	
				-- BR_Address_W <= 0;			
				
			-- else
				-- BR_Address_W <= BR_Address_W + 1;				 	
			-- end if;

		-- elsif image_data_enable ='0' then
			-- BR_Enable_W			<= '0';
			-- BR_WriteHigh_W		<= "0";
			-- --BR_Address_W  		<= BR_Address_W;
			-- BR_DataIn_W			<= (others =>'0');
				
		-- end if;
	-- end if;
-- end process;

--################################## TP Block RAM 307200 ###############################################

--Buf-state---------------------------------------------------------------------------------------------------
process(rst_system, clk_video)
begin
	if rst_system = '0' then	
		image_data_enable <= '0';
		buf_data_state <= "00";
	else
		if rising_edge(clk_video) then
			if (buf_vga_en = '1' and (cnt_video_hsync >= 0 and cnt_video_hsync < 1280 and cnt_v_sync_vga >= 0 and cnt_v_sync_vga < 480)) then --buf_vga_en >>image begin enable      (cnt_video_hsync < 1290)  >> 640*2 =1280  effective data			
				buf_data_state <= buf_data_state + '1';--cb(00)  Y(01)  cr(10)  Y(11)  			
				image_data_enable <= '1';							
			else			
				image_data_enable <= '0';			
				buf_data_state <= "00";
			end if;
		end if;
	end if;
end process;
--Buf-state---------------------------------------------------------------------------------------------------

video_buffer : process(rst_system, clk_video)
begin
if rst_system = '0' then
	buf_vga_state <= "00";
	buf_vga_Y_in_cnt <= 0;

	YCR <= x"00000"; --YUV convert R
	YCG <= x"00000"; --YUV convert G
	YCB <= x"00000"; --YUV convert B

	Cr_register <= x"00";
	Cb_register <= x"00";
else
	if rising_edge(clk_video) then
		if (buf_vga_en = '1' and cnt_video_hsync < 1280) then

			case buf_vga_state(0) is
				when '0' =>	buf_vga_state <= buf_vga_state + "01"; --the vdata is Cb
				    if YCR > x"00000" or YCG > x"00000" or YCB > x"00000" then	--The first data has not yet completed calculate
					    if YCR(18) = '1' or YCR(19) = '1' then
                            buf_vga_R(buf_vga_Y_in_cnt) <= "11111111";
						else
							buf_vga_R(buf_vga_Y_in_cnt) <= YCR(17 downto 10);
						end if;
						if YCG(18) = '1' or YCG(19) = '1' then
							buf_vga_G(buf_vga_Y_in_cnt) <= "11111111";
						else
							buf_vga_G(buf_vga_Y_in_cnt) <= YCG(17 downto 10);
						end if;
						if YCB(18) = '1' or YCB(19) = '1' then
							buf_vga_B(buf_vga_Y_in_cnt) <= "11111111";
						else
							buf_vga_B(buf_vga_Y_in_cnt) <= YCB(17 downto 10);
						end if;
					end if;

					if buf_vga_state(1)='0' then --cb
						Cb_register <= data_video;
						YCR <= Cr_register * YCR_C1;
						YCG <= data_video * YCG_C1 + Cr_register * YCG_C2;
						YCB <= data_video * YCB_C1;
					else						 --cr
						Cr_register <= data_video;
						YCR <= data_video * YCR_C1;
						YCG <= Cb_register * YCG_C1 + data_video * YCG_C2;
						YCB <= Cb_register * YCB_C1;
					end if;
	--					buf_vga_C(buf_vga_Y_in_cnt) <= data_video(7 downto 0);
                when '1' =>	buf_vga_state <= buf_vga_state + "01"; --the vdata is Y
				    --x"400"   = 1024(d)
					--x"2d0a3" = 1024(d) * 128(d) * 1.4075(d)
					--x"b0e5"  = 1024(d) * 128(d) * 0.3455(d)
					--x"16f0d" = 1024(d) * 128(d) * 0.7169(d)
					--x"38ed9" = 1024(d) * 128(d) * 1.7790(d)
--					YCR <= YCR + vdata * x"400" - x"2d0a3";
--					YCG <= (YCG xor x"fffff") + vdata * x"400" + x"b0e5" + x"16f0d";
--					YCB <= YCB + vdata * x"400" - x"38ed9";
					if YCR + ( "00"& data_video & "0000000000") < x"2d0a3" then
						YCR <= x"00000";
					else
						YCR <= YCR + ( "00"& data_video & "0000000000") - x"2d0a3";
					end if;
					if YCG > ( "00"& data_video & "0000000000") + x"b0e5" + x"16f0d" then
						YCG <= x"00000";
					else
						YCG <=	( "00"& data_video & "0000000000") + x"b0e5" + x"16f0d" - YCG;
					end if;
					if YCB + ( "00"& data_video & "0000000000") < x"38ed9" then
						YCB <= x"00000";
					else
						YCB <= YCB + ( "00"& data_video & "0000000000") - x"38ed9";
					end if;
					if buf_vga_Y_in_cnt = 639 then
						buf_vga_Y_in_cnt <= 0;
					else
						buf_vga_Y_in_cnt <= buf_vga_Y_in_cnt + 1;
					end if;		
					
--					buf_vga_Y(buf_vga_Y_in_cnt) <= Bram_out(7 downto 0);	---### test block rom video			
					buf_vga_Y(buf_vga_Y_in_cnt) <= data_video(7 downto 0);
				when others => null;
			end case;
		else
			buf_vga_state        <= "00";
			buf_vga_Y_in_cnt     <= 0;
			YCR                  <= x"00000";
			YCG                  <= x"00000";
			YCB                  <= x"00000";
		end if;
	end if;
end if;
end process;
end Behavioral;


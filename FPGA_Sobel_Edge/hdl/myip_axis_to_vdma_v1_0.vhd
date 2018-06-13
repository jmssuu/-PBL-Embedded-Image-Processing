library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity myip_axis_to_vdma_v1_0 is
	generic (
		-- Users to add parameters here

		-- User parameters ends
		-- Do not modify the parameters beyond this line


		-- Parameters of Axi Master Bus Interface M00_AXIS
		C_M00_AXIS_TDATA_WIDTH	: integer	:= 32;
--		C_M00_AXIS_START_COUNT	: integer	:= 32;
		FIFO_DEPTH      : integer	:= 640
--		VIDEO_HSYNC_COUNT : integer := 640;
--		VIDEO_VSYNC_COUNT : integer := 480
	);
	port (
		-- Users to add ports here
		clk_video  : IN  std_logic;
--           reset_active_low : IN  std_logic;
        data_in_en_flag_active_high : in std_logic;
         h_blank_vga : in  std_logic;
         v_blank_vga : in  std_logic;
        video_data_in : in std_logic_vector(C_M00_AXIS_TDATA_WIDTH-1 downto 0);
      
--		video_clk : in STD_LOGIC;  --27Mhz
--        video_data : in STD_LOGIC_VECTOR ( 31 downto 0 );--( 23 downto 0 );--( 31 downto 0 );--( 23 downto 0 );
--        video_hsync : in STD_LOGIC;
--        video_vsync : in STD_LOGIC;
--        sync_vga_en : in  std_logic;
		-- User ports ends
		-- Do not modify the ports beyond this line

        m00_axis_tuser	: out std_logic := '0'; --for s2mm tuser mode set used
--        m00_axis_fsync	: out std_logic;

		-- Ports of Axi Master Bus Interface M00_AXIS
		m00_axis_aclk	: in std_logic;
		m00_axis_aresetn	: in std_logic;
		m00_axis_tvalid	: out std_logic;
		m00_axis_tdata	: out std_logic_vector(C_M00_AXIS_TDATA_WIDTH-1 downto 0);
		m00_axis_tstrb	: out std_logic_vector((C_M00_AXIS_TDATA_WIDTH/8)-1 downto 0);
		m00_axis_tlast	: out std_logic;
		m00_axis_tready	: in std_logic

	);
end myip_axis_to_vdma_v1_0;

architecture arch_imp of myip_axis_to_vdma_v1_0 is

	-- component declaration
	component myip_axis_to_vdma_v1_0_M00_AXIS is
		generic (
		C_M_AXIS_TDATA_WIDTH	: integer	:= 32;
		C_M_START_COUNT	: integer	:= 32;
		FIFO_DEPTH      : integer	:= 640
--		VIDEO_HSYNC_COUNT : integer;
--        VIDEO_VSYNC_COUNT : integer
		);
		port (
		clk_video  : IN  std_logic;
--           reset_active_low : IN  std_logic;
        data_in_en_flag_active_high : in std_logic;
         h_blank_vga : in  std_logic;
         v_blank_vga : in  std_logic;
        video_data_in : in std_logic_vector(C_M00_AXIS_TDATA_WIDTH-1 downto 0);
--		video_clk : in STD_LOGIC;  --27Mhz
--        video_data : in STD_LOGIC_VECTOR ( 31 downto 0 );--( 23 downto 0 );--( 31 downto 0 );--( 23 downto 0 );
--        video_hsync : in STD_LOGIC;
--        video_vsync : in STD_LOGIC;
--        sync_vga_en : in  std_logic;   
		
--		M_AXIS_TUSER    : out std_logic; --for s2mm tuser mode set used
----        M_AXIS_FSYNC    : out std_logic; 

		M_AXIS_ACLK	: in std_logic;
		M_AXIS_ARESETN	: in std_logic;
		M_AXIS_TVALID	: out std_logic;
		M_AXIS_TDATA	: out std_logic_vector(C_M_AXIS_TDATA_WIDTH-1 downto 0);
		M_AXIS_TSTRB	: out std_logic_vector((C_M_AXIS_TDATA_WIDTH/8)-1 downto 0);
		M_AXIS_TLAST	: out std_logic;
		M_AXIS_TREADY	: in std_logic
		);
	end component myip_axis_to_vdma_v1_0_M00_AXIS;

begin

-- Instantiation of Axi Bus Interface M00_AXIS
myip_axis_to_vdma_v1_0_M00_AXIS_inst : myip_axis_to_vdma_v1_0_M00_AXIS
	generic map (
		C_M_AXIS_TDATA_WIDTH	=> C_M00_AXIS_TDATA_WIDTH,
--		C_M_START_COUNT	=> C_M00_AXIS_START_COUNT,
		FIFO_DEPTH => FIFO_DEPTH
--        VIDEO_HSYNC_COUNT => VIDEO_HSYNC_COUNT,
--        VIDEO_VSYNC_COUNT => VIDEO_VSYNC_COUNT
	)
	port map (
	   clk_video => clk_video,
    --           reset_active_low : IN  std_logic;
        data_in_en_flag_active_high => data_in_en_flag_active_high,
         h_blank_vga => h_blank_vga,
         v_blank_vga => v_blank_vga,
        video_data_in => video_data_in,
            
--        video_clk => video_clk,--27MHz
--        video_data => video_data,
--        video_hsync => video_hsync,
--        video_vsync => video_vsync,
--        sync_vga_en =>  sync_vga_en,
		
--		M_AXIS_TUSER => m00_axis_tuser,
----        M_AXIS_FSYNC => m00_axis_fsync,
    
		M_AXIS_ACLK	=> m00_axis_aclk,
		M_AXIS_ARESETN	=> m00_axis_aresetn,
		M_AXIS_TVALID	=> m00_axis_tvalid,
		M_AXIS_TDATA	=> m00_axis_tdata,
		M_AXIS_TSTRB	=> m00_axis_tstrb,
		M_AXIS_TLAST	=> m00_axis_tlast,
		M_AXIS_TREADY	=> m00_axis_tready
	);

	-- Add user logic here

	-- User logic ends

end arch_imp;

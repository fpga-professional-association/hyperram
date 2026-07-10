	component hbgpio is
		port (
			ck_cell_ck_export      : in    std_logic                     := 'X';             -- export
			ck_cell_din_export     : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- export
			ck_cell_pad_out_export : out   std_logic_vector(0 downto 0);                     -- export
			ck_cell_cke_export     : in    std_logic                     := 'X';             -- export
			dq_cell_ck_in_export   : in    std_logic                     := 'X';             -- export
			dq_cell_ck_out_export  : in    std_logic                     := 'X';             -- export
			dq_cell_dout_export    : out   std_logic_vector(15 downto 0);                    -- export
			dq_cell_din_export     : in    std_logic_vector(15 downto 0) := (others => 'X'); -- export
			dq_cell_oe_export      : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- export
			dq_cell_pad_io_export  : inout std_logic_vector(7 downto 0)  := (others => 'X')  -- export
		);
	end component hbgpio;

	u0 : component hbgpio
		port map (
			ck_cell_ck_export      => CONNECTED_TO_ck_cell_ck_export,      --      ck_cell_ck.export
			ck_cell_din_export     => CONNECTED_TO_ck_cell_din_export,     --     ck_cell_din.export
			ck_cell_pad_out_export => CONNECTED_TO_ck_cell_pad_out_export, -- ck_cell_pad_out.export
			ck_cell_cke_export     => CONNECTED_TO_ck_cell_cke_export,     --     ck_cell_cke.export
			dq_cell_ck_in_export   => CONNECTED_TO_dq_cell_ck_in_export,   --   dq_cell_ck_in.export
			dq_cell_ck_out_export  => CONNECTED_TO_dq_cell_ck_out_export,  --  dq_cell_ck_out.export
			dq_cell_dout_export    => CONNECTED_TO_dq_cell_dout_export,    --    dq_cell_dout.export
			dq_cell_din_export     => CONNECTED_TO_dq_cell_din_export,     --     dq_cell_din.export
			dq_cell_oe_export      => CONNECTED_TO_dq_cell_oe_export,      --      dq_cell_oe.export
			dq_cell_pad_io_export  => CONNECTED_TO_dq_cell_pad_io_export   --  dq_cell_pad_io.export
		);


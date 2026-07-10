module hbgpio (
		input  wire        ck_cell_ck_export,      //      ck_cell_ck.export
		input  wire [1:0]  ck_cell_din_export,     //     ck_cell_din.export
		output wire [0:0]  ck_cell_pad_out_export, // ck_cell_pad_out.export
		input  wire        ck_cell_cke_export,     //     ck_cell_cke.export
		input  wire        dq_cell_ck_in_export,   //   dq_cell_ck_in.export
		input  wire        dq_cell_ck_out_export,  //  dq_cell_ck_out.export
		output wire [15:0] dq_cell_dout_export,    //    dq_cell_dout.export
		input  wire [15:0] dq_cell_din_export,     //     dq_cell_din.export
		input  wire [7:0]  dq_cell_oe_export,      //      dq_cell_oe.export
		inout  wire [7:0]  dq_cell_pad_io_export   //  dq_cell_pad_io.export
	);
endmodule


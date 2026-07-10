	hbgpio u0 (
		.ck_cell_ck_export      (_connected_to_ck_cell_ck_export_),      //   input,   width = 1,      ck_cell_ck.export
		.ck_cell_din_export     (_connected_to_ck_cell_din_export_),     //   input,   width = 2,     ck_cell_din.export
		.ck_cell_pad_out_export (_connected_to_ck_cell_pad_out_export_), //  output,   width = 1, ck_cell_pad_out.export
		.ck_cell_cke_export     (_connected_to_ck_cell_cke_export_),     //   input,   width = 1,     ck_cell_cke.export
		.dq_cell_ck_in_export   (_connected_to_dq_cell_ck_in_export_),   //   input,   width = 1,   dq_cell_ck_in.export
		.dq_cell_ck_out_export  (_connected_to_dq_cell_ck_out_export_),  //   input,   width = 1,  dq_cell_ck_out.export
		.dq_cell_dout_export    (_connected_to_dq_cell_dout_export_),    //  output,  width = 16,    dq_cell_dout.export
		.dq_cell_din_export     (_connected_to_dq_cell_din_export_),     //   input,  width = 16,     dq_cell_din.export
		.dq_cell_oe_export      (_connected_to_dq_cell_oe_export_),      //   input,   width = 8,      dq_cell_oe.export
		.dq_cell_pad_io_export  (_connected_to_dq_cell_pad_io_export_)   //   inout,   width = 8,  dq_cell_pad_io.export
	);


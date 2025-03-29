module dmi_intel (
	input logic				clk_i,	// DMI Clock
	input logic				rst_ni,	// Asynchronous reset active low

	output dm::dmi_req_t	dmi_req_o,
	output logic			dmi_req_valid_o,
	input logic				dmi_req_ready_i,

	input dm::dmi_resp_t	dmi_resp_i,
	output logic			dmi_resp_ready_o,
	input logic				dmi_resp_valid_i,

	input logic				tck,
	input logic				tms,
	input logic				tdi,
	output logic			tdo
);

	typedef enum logic [1:0] {
		DMINoError = 2'h0, DMIReservedError = 2'h1,
		DMIOPFailed = 2'h2, DMIBusy = 2'h3
	} dmi_error_e;
	dmi_error_e error_d, error_q;

	logic		jtag_dmi_clear;	// Synchronous reset of DMI triggered by TestLogicReset in jtag TAP
	logic		dmi_clear;		// Functional (warm) reset of the entire DMI
	logic		update;
	logic		capture;
	logic		shift;

	logic dtmcs_select;

	// -------------------------------
	// Debug Module Control and Status
	// -------------------------------

	dm::dtmcs_t dtmcs_d, dtmcs_q;

	assign dmi_clear = jtag_dmi_clear || (dtmcs_select && update && dtmcs_q.dmihardreset);

	always_comb begin
		dtmcs_d = dtmcs_q;
		if (capture) begin
			if (dtmcs_select) begin
				dtmcs_d	= '{
					zero1: 			'0,
					dmihardreset:	1'b0,
					dmireset:		1'b0,
					zero0:			'0,
					idle:			3'd1,		// The size of address in dmi
					dmistat:		error_q,	// 0: No error, 2: Op failed, 3: too fast
					abits:			6'd7,		// The size of address in dmi
					version:		4'd1		// Version described in spec version 0.13 (and later?)
				};
			end
		end

		if (shift) begin
			if (dtmcs_select) dtmcs_d = {tdi, 31'(dtmcs_q >> 1)};
		end
	end

	always_ff @(posedge tck or negedge rst_ni) begin
		if (!rst_ni) begin
			dtmcs_q <= '0;
		end else begin
			dtmcs_q <= dtmcs_d;
		end
	end

	// ----------------------------
	// DMI (Debug Module Interface)
	// ----------------------------

	logic			dmi_select;

	typedef struct packed {
		logic [6:0]	address;
		logic [31:0] data;
		logic [1:0]	op;
	} dmi_t;

	typedef enum logic [2:0] { Idle, Read, WaitReadValid, Write, WaitWriteValid } state_e;
	state_e						state_d, state_q;

	logic [$bits(dmi_t)-1:0]	dr_d, dr_q;
	logic [6:0]					address_d, address_q;
	logic [31:0]				data_d, data_q;

	dmi_t			dmi;
	assign			dmi = 				dmi_t'(dr_q);

	dm::dmi_req_t	dmi_req;
	assign			dmi_req.addr =		address_q;
	assign			dmi_req.data =		data_q;
	assign			dmi_req.op =		(state_q == Write) ? dm::DTM_WRITE : dm::DTM_READ;
	assign			dmi_req_o =			dmi_req;
	logic			dmi_req_valid;
	assign			dmi_req_valid_o =	dmi_req_valid;

	// We will always be ready to accept the data we requested.
	assign dmi_resp_ready_o = 1'b1;

	logic error_dmi_busy;
	logic error_dmi_op_failed;

	always_comb begin : p_fsm
		error_dmi_busy		= 1'b0;
		error_dmi_op_failed	= 1'b0;
		state_d				= state_q;
		address_d			= address_q;
		data_d				= data_q;
		error_d				= error_q;

		dmi_req_valid = 1'b0;

		if (dmi_clear) begin
			state_d	 = Idle;
			data_d		= '0;
			error_d	 = DMINoError;
			address_d = '0;
		end else begin
			unique case (state_q)
				Idle: begin
					// make sure that no error is sticky
					if (dmi_select && update && (error_q == DMINoError)) begin
						// save address and value
						address_d = dmi.address;
						data_d = dmi.data;
						if (dm::dtm_op_e'(dmi.op) == dm::DTM_READ) begin
							state_d = Read;
						end else if (dm::dtm_op_e'(dmi.op) == dm::DTM_WRITE) begin
							state_d = Write;
						end
						// else this is a nop and we can stay here
					end
				end

				Read: begin
					dmi_req_valid = 1'b1;
					if (dmi_req_ready_i) begin
						state_d = WaitReadValid;
					end
				end

				WaitReadValid: begin
					// load data into register and shift out
					if (dmi_resp_valid_i) begin
						unique case (dmi_resp_i.resp)
							dm::DTM_SUCCESS: begin
								data_d = dmi_resp_i.data;
							end
							dm::DTM_ERR: begin
								//data_d = 32'hDEAD_BEEF;
								error_dmi_op_failed = 1'b1;
							end
							dm::DTM_BUSY: begin
								data_d = 32'hB051_B051;
								error_dmi_busy = 1'b1;
							end
							//default: begin
							//	data_d = 32'hBAAD_C0DE;
							//end
						endcase
						state_d = Idle;
					end
				end

				Write: begin
					dmi_req_valid = 1'b1;
					// request sent, wait for response before going back to idle
					if (dmi_req_ready_i) begin
						state_d = WaitWriteValid;
					end
				end

				WaitWriteValid: begin
					// got a valid answer go back to idle
					if (dmi_resp_valid_i) begin
						unique case (dmi_resp_i.resp)
							dm::DTM_ERR: error_dmi_op_failed = 1'b1;
							dm::DTM_BUSY: error_dmi_busy = 1'b1;
							default: ;
						endcase
						state_d = Idle;
					end
				end

				default: begin
					// just wait for idle here
					if (dmi_resp_valid_i) begin
						state_d = Idle;
					end
				end
			endcase

			// update means we got another request but we didn't finish
			// the one in progress, this state is sticky
			if (update && state_q != Idle) begin
				error_dmi_busy = 1'b1;
			end

			// if capture goes high while we are in the read state
			// or in the corresponding wait state we are not giving back a valid word
			// -> throw an error
			if (capture && (state_q == Read || state_q == WaitReadValid)) begin
				error_dmi_busy = 1'b1;
			end

			if (error_dmi_busy && error_q == DMINoError) begin
				error_d = DMIBusy;
			end

			if (error_dmi_op_failed && error_q == DMINoError) begin
				error_d = DMIOPFailed;
			end

			// clear sticky error flag
			if (update && dtmcs_q.dmireset && dtmcs_select) begin
				error_d = DMINoError;
			end
		end
	end

	always_comb begin : p_shift
		dr_d		= dr_q;
		if (dmi_clear) begin
			dr_d = '0;
		end else begin
			if (capture) begin
				if (dmi_select) begin
					if (error_q == DMINoError && !error_dmi_busy) begin
						dr_d = {address_q, data_q, DMINoError};
						// DMI was busy, report an error
					end else if (error_q == DMIBusy || error_dmi_busy) begin
						dr_d = {address_q, data_q, DMIBusy};
					end
				end
			end

			if (shift) begin
				if (dmi_select) begin
					dr_d = {tdi, dr_q[$bits(dr_q)-1:1]};
				end
			end
		end
	end

	always_ff @(posedge tck or negedge rst_ni) begin
		if (!rst_ni) begin
			dr_q		<= '0;
			state_q		<= Idle;
			address_q	<= '0;
			data_q		<= '0;
			error_q		<= DMINoError;
		end else begin
			dr_q		<= dr_d;
			state_q		<= state_d;
			address_q	<= address_d;
			data_q		<= data_d;
			error_q		<= error_d;
		end
	end

	// ----------------------------
	// JTAG
	// ----------------------------

	localparam exit2_dr			= 0;
	localparam exit1_dr			= 1;
	localparam shift_dr			= 2;
	localparam pause_dr			= 3;
	localparam select_ir_scan	= 4;
	localparam update_dr		= 5;
	localparam capture_dr		= 6;
	localparam select_dr_scan	= 7;
	localparam exit2_ir			= 8;
	localparam exit1_ir			= 9;
	localparam shift_ir			= 10;
	localparam pause_ir			= 11;
	localparam run_test_idle	= 12;
	localparam update_ir		= 13;
	localparam capture_ir		= 14;
	localparam test_logic_reset	= 15;

	reg [3:0]	fsm_state = 		test_logic_reset;

	reg [9:0]	ir_shiftreg = 		0;
	reg [9:0]	ir_reg = 			0;

	assign		jtag_dmi_clear =	fsm_state == test_logic_reset;
	assign		update =			fsm_state == update_dr;
	assign		capture =			fsm_state == capture_dr;
	assign		shift =				fsm_state == shift_dr;
	assign		dtmcs_select =		ir_reg == 10'h00c;
	assign		dmi_select =		ir_reg == 10'h00e;

	always @(posedge tck) begin
		case(fsm_state) 
			test_logic_reset	: fsm_state <= tms ? test_logic_reset	: run_test_idle;
			run_test_idle		: fsm_state <= tms ? select_dr_scan		: run_test_idle;
			select_dr_scan		: fsm_state <= tms ? select_ir_scan		: capture_dr;
			capture_dr			: fsm_state <= tms ? exit1_dr			: shift_dr;
			shift_dr			: fsm_state <= tms ? exit1_dr			: shift_dr;
			exit1_dr			: fsm_state <= tms ? update_dr			: pause_dr;
			pause_dr			: fsm_state <= tms ? exit2_dr			: pause_dr;
			exit2_dr			: fsm_state <= tms ? update_dr			: shift_dr;
			update_dr			: fsm_state <= tms ? select_dr_scan		: run_test_idle;
			select_ir_scan		: fsm_state <= tms ? test_logic_reset	: capture_ir;
			capture_ir			: fsm_state <= tms ? exit1_ir			: shift_ir;
			shift_ir			: fsm_state <= tms ? exit1_ir			: shift_ir;
			exit1_ir			: fsm_state <= tms ? update_ir			: pause_ir;
			pause_ir			: fsm_state <= tms ? exit2_ir			: pause_dr;
			exit2_ir			: fsm_state <= tms ? update_ir			: shift_ir;
			update_ir			: fsm_state <= tms ? select_dr_scan		: run_test_idle;
		endcase
	end

	always @(posedge tck) begin
		if (fsm_state == shift_ir) begin
			ir_shiftreg <= { tdi, ir_shiftreg[9:1] };
		end

		if (fsm_state == update_ir) begin
			ir_reg <= ir_shiftreg;
		end
	end

	always @(negedge tck) begin
		if (dtmcs_select) begin
			tdo	<= dtmcs_q[0];
		end
		if (dmi_select) begin
			tdo	<= dr_q[0];
		end
	end
	
endmodule
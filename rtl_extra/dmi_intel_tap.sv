module dmi_intel_tap (
	input  logic        tck_i,
	input  logic        tms_i,
	input  logic        tdi_i,
	output logic        tdo_o,

	output logic        update_o,
	output logic        capture_o,
	output logic        shift_o,

	output logic        dtmcs_select_o,
	input  logic        dtmcs_tdo_i,

	output logic        dmi_select_o,
	output logic        dmi_clear_o,
	input  logic        dmi_tdo_i
);

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

	reg [3:0]	fsm_state		= test_logic_reset;

	reg [9:0]	ir_shiftreg		= 0;
	reg [9:0]	ir_reg			= 0;

	assign		update_o		= fsm_state == update_dr;
	assign		capture_o		= fsm_state == capture_dr;
	assign		shift_o			= fsm_state == shift_dr;
	assign		dtmcs_select_o	= ir_reg == 10'h00c;
	assign		dmi_select_o	= ir_reg == 10'h00e;
	assign		dmi_clear_o		= fsm_state == test_logic_reset;

	always @(posedge tck_i) begin
		case(fsm_state) 
			test_logic_reset	: fsm_state <= tms_i ? test_logic_reset	: run_test_idle;
			run_test_idle		: fsm_state <= tms_i ? select_dr_scan	: run_test_idle;
			select_dr_scan		: fsm_state <= tms_i ? select_ir_scan	: capture_dr;
			capture_dr			: fsm_state <= tms_i ? exit1_dr			: shift_dr;
			shift_dr			: fsm_state <= tms_i ? exit1_dr			: shift_dr;
			exit1_dr			: fsm_state <= tms_i ? update_dr		: pause_dr;
			pause_dr			: fsm_state <= tms_i ? exit2_dr			: pause_dr;
			exit2_dr			: fsm_state <= tms_i ? update_dr		: shift_dr;
			update_dr			: fsm_state <= tms_i ? select_dr_scan	: run_test_idle;
			select_ir_scan		: fsm_state <= tms_i ? test_logic_reset	: capture_ir;
			capture_ir			: fsm_state <= tms_i ? exit1_ir			: shift_ir;
			shift_ir			: fsm_state <= tms_i ? exit1_ir			: shift_ir;
			exit1_ir			: fsm_state <= tms_i ? update_ir		: pause_ir;
			pause_ir			: fsm_state <= tms_i ? exit2_ir			: pause_dr;
			exit2_ir			: fsm_state <= tms_i ? update_ir		: shift_ir;
			update_ir			: fsm_state <= tms_i ? select_dr_scan	: run_test_idle;
		endcase
	end

	always @(posedge tck_i) begin
		if (fsm_state == shift_ir) begin
			ir_shiftreg <= { tdi_i, ir_shiftreg[9:1] };
		end

		if (fsm_state == update_ir) begin
			ir_reg <= ir_shiftreg;
		end
	end

	always @(negedge tck_i) begin
		if (dtmcs_select_o) begin
			tdo_o	<= dtmcs_tdo_i;
		end
		if (dmi_select_o) begin
			tdo_o	<= dmi_tdo_i;
		end
	end
	
endmodule
module dmi_intel (
  input  logic         clk_i,      // DMI Clock
  input  logic         rst_ni,     // Asynchronous reset active low

  output dm::dmi_req_t dmi_req_o,
  output logic         dmi_req_valid_o,
  input  logic         dmi_req_ready_i,

  input dm::dmi_resp_t dmi_resp_i,
  output logic         dmi_resp_ready_o,
  input  logic         dmi_resp_valid_i,

  input  logic         tck_i,
  input  logic         tms_i,
  input  logic         tdi_i,
  output logic         tdo_o
);

  typedef enum logic [1:0] {
    DMINoError = 2'h0, DMIReservedError = 2'h1,
    DMIOPFailed = 2'h2, DMIBusy = 2'h3
  } dmi_error_e;
  dmi_error_e error_d, error_q;

  logic jtag_dmi_clear; // Synchronous reset of DMI triggered by TestLogicReset in jtag TAP
  logic dmi_clear;		// Functional (warm) reset of the entire DMI
  logic update;
  logic capture;
  logic shift;

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
        dtmcs_d  = '{
                      zero1        : '0,
                      dmihardreset : 1'b0,
                      dmireset     : 1'b0,
                      zero0        : '0,
                      idle         : 3'd1, // 1: Enter Run-Test/Idle and leave it immediately
                      dmistat      : error_q, // 0: No error, 2: Op failed, 3: too fast
                      abits        : 6'd7, // The size of address in dmi
                      version      : 4'd1  // Version described in spec version 0.13 (and later?)
                    };
      end
    end

    if (shift) begin
      if (dtmcs_select) dtmcs_d  = {tdi_i, 31'(dtmcs_q >> 1)};
    end
  end

  always_ff @(posedge tck_i) begin
    dtmcs_q <= dtmcs_d;
  end

  // ----------------------------
  // DMI (Debug Module Interface)
  // ----------------------------

  logic        dmi_select;

  dm::dmi_req_t  dmi_req;
  logic          dmi_req_ready;
  logic          dmi_req_valid;

  dm::dmi_resp_t dmi_resp;
  logic          dmi_resp_valid;
  logic          dmi_resp_ready;

  typedef struct packed {
    logic [6:0]  address;
    logic [31:0] data;
    logic [1:0]  op;
  } dmi_t;

  typedef enum logic [2:0] { Idle, Read, WaitReadValid, Write, WaitWriteValid } state_e;
  state_e state_d, state_q;

  logic [$bits(dmi_t)-1:0] dr_d, dr_q;
  logic [6:0] address_d, address_q;
  logic [31:0] data_d, data_q;

  dmi_t  dmi;
  assign dmi          = dmi_t'(dr_q);
  assign dmi_req.addr = address_q;
  assign dmi_req.data = data_q;
  assign dmi_req.op   = (state_q == Write) ? dm::DTM_WRITE : dm::DTM_READ;
  // We will always be ready to accept the data we requested.
  assign dmi_resp_ready = 1'b1;

  logic error_dmi_busy;
  logic error_dmi_op_failed;

  always_comb begin : p_fsm
    error_dmi_busy = 1'b0;
    error_dmi_op_failed = 1'b0;
    // default assignments
    state_d   = state_q;
    address_d = address_q;
    data_d    = data_q;
    error_d   = error_q;

    dmi_req_valid = 1'b0;

    if (dmi_clear) begin
      state_d   = Idle;
      data_d    = '0;
      error_d   = DMINoError;
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
          if (dmi_req_ready) begin
            state_d = WaitReadValid;
          end
        end

        WaitReadValid: begin
          // load data into register and shift out
          if (dmi_resp_valid) begin
            unique case (dmi_resp.resp)
              dm::DTM_SUCCESS: begin
                data_d = dmi_resp.data;
              end
              dm::DTM_ERR: begin
                data_d = 32'hDEAD_BEEF;
                error_dmi_op_failed = 1'b1;
              end
              dm::DTM_BUSY: begin
                data_d = 32'hB051_B051;
                error_dmi_busy = 1'b1;
              end
              default: begin
                data_d = 32'hBAAD_C0DE;
              end
            endcase
            state_d = Idle;
          end
        end

        Write: begin
          dmi_req_valid = 1'b1;
          // request sent, wait for response before going back to idle
          if (dmi_req_ready) begin
            state_d = WaitWriteValid;
          end
        end

        WaitWriteValid: begin
          // got a valid answer go back to idle
          if (dmi_resp_valid) begin
            unique case (dmi_resp.resp)
              dm::DTM_ERR: error_dmi_op_failed = 1'b1;
              dm::DTM_BUSY: error_dmi_busy = 1'b1;
              default: ;
            endcase
            state_d = Idle;
          end
        end

        default: begin
          // just wait for idle here
          if (dmi_resp_valid) begin
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
    dr_d    = dr_q;
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
          dr_d = {tdi_i, dr_q[$bits(dr_q)-1:1]};
        end
      end
    end
  end

  always_ff @(posedge tck_i) begin
    dr_q      <= dr_d;
    state_q   <= state_d;
    address_q <= address_d;
    data_q    <= data_d;
    error_q   <= error_d;
  end

  // ---------
  // TAP
  // ---------
  dmi_intel_tap i_dmi_intel_tap (
    .tck_i			( tck_i            ),
    .tms_i			( tms_i            ),
    .tdi_i			( tdi_i            ),
    .tdo_o			( tdo_o            ),
    .dmi_clear_o    ( jtag_dmi_clear   ),
    .update_o       ( update           ),
    .capture_o      ( capture          ),
    .shift_o        ( shift            ),
    .dtmcs_select_o ( dtmcs_select     ),
    .dtmcs_tdo_i    ( dtmcs_q[0]       ),
    .dmi_select_o   ( dmi_select       ),
    .dmi_tdo_i      ( dr_q[0]          )
  );

  // ---------
  // CDC
  // ---------
  altera_avalon_st_clock_crosser #(.BITS_PER_SYMBOL($bits(dm::dmi_req_t))) i_cdc_req (
    .in_clk(tck_i),
    .in_reset(dmi_clear),
    .in_ready(dmi_req_ready),
    .in_valid(dmi_req_valid),
    .in_data(dmi_req),
    .out_clk(clk_i),
    .out_reset(~rst_ni),
    .out_ready(dmi_req_ready_i),
    .out_valid(dmi_req_valid_o),
    .out_data(dmi_req_o)
  );

  altera_avalon_st_clock_crosser #(.BITS_PER_SYMBOL($bits(dm::dmi_resp_t))) i_cdc_resp (
    .in_clk(clk_i),
    .in_reset(~rst_ni),
    .in_ready(dmi_resp_ready_o),
    .in_valid(dmi_resp_valid_i),
    .in_data(dmi_resp_i),
    .out_clk(tck_i),
    .out_reset(dmi_clear),
    .out_ready(dmi_resp_ready),
    .out_valid(dmi_resp_valid),
    .out_data(dmi_resp)
  );

endmodule
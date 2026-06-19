// ======================================================================
//  simple_axi_master.sv
//
//  PURPOSE
//    Behavioral AXI4 master that acts as both stimulus generator and
//    protocol checker in simulation. It issues a configurable number of
//    write and read bursts and self-checks every response.
//
//  FEATURES
//    - Issues NUM_TRANSACTIONS write (AW) and read (AR) requests, each a
//      fixed BURST_BEATS-beat burst (AWLEN/ARLEN = BURST_BEATS-1).
//    - Multiple-outstanding: does not wait for B/R before issuing the next
//      AW/AR, stressing the interconnect and slave queues.
//    - Deterministic, self-describing payloads: address and write data are
//      derived from MASTER_ID + transaction index, so any beat is traceable.
//    - In-order self-checking: verifies BID/RID, BRESP/RRESP == OKAY, and
//      RLAST placement against the expected per-transaction values.
//    - Injects R-channel backpressure (RREADY periodically deasserted) to
//      exercise the read-data handshake.
//    - Demo pacing knobs (ISSUE_GAP, START_DELAY) to space the request
//      stream and offset the two masters - see PARAMETERS / testbench header.
//
//  DATA PATH  (one FSM per AXI channel)
//
//    wr_req[] --> [AW issuer]  AWID/AWADDR/AWLEN  ------------> to arbiter
//                 [W  issuer]  WDATA/WLAST (AW order, no WID) ->
//    [B checker] <-- BID/BRESP                                <--
//    rd_req[] --> [AR issuer]  ARID/ARADDR/ARLEN -------------> to arbiter
//    [R checker] <-- RID/RDATA/RLAST/RRESP                    <--
//
//    Write data follows AW issue order because AXI4 has no WID. The B and R
//    checkers advance simple per-transaction indices (b_recv_idx, r_txn_idx),
//    so they assume in-order completion (see ASSUMPTIONS).
//
//  PARAMETERS
//    ID_WIDTH, ADDR_WIDTH, DATA_WIDTH - bus geometry.
//    NUM_TRANSACTIONS - write & read requests issued by this master.
//    DEBUG       - 1 enables the per-channel trace prints.
//    ISSUE_GAP   - idle cycles between successive AW/AR issues
//                  (0 = back-to-back worst case; >0 = spaced demo).
//    START_DELAY - idle cycles before the first issue; offsets the two
//                  masters relative to each other.
//
//  ASSUMPTIONS & LIMITS
//    - Responses must return in request order (in-order slave/interconnect);
//      out-of-order completion across IDs is NOT supported by the checkers.
//    - AXI ID is ID_WIDTH bits, so the transaction index and the AXI ID
//      diverge once NUM_TRANSACTIONS > 2^ID_WIDTH (IDs wrap). Use
//      ID_WIDTH >= clog2(NUM_TRANSACTIONS) for unique, traceable IDs.
//
//  DEBUG TRACE  (full legend in the testbench header)
//    [time] SRC CH dir txn=M<route>.<id>  <fields>
//    SRC = M0|M1, CH = AW/W/B/AR/R, dir = >> request / << response.
//    NOTE: the W line prints the transaction index while every other line
//    prints the AXI ID; these differ when IDs wrap (see above).
// ======================================================================
module simple_axi_master #(
    parameter int ID_WIDTH = 4,
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 128,
    parameter int NUM_TRANSACTIONS = 2,
    parameter bit DEBUG = 1'b0, // 1 = enable prints, 0 = disable
    // Demo pacing. Both default to 0 = back-to-back worst-case stress test.
    // ISSUE_GAP   : idle ACLK cycles between this master's successive AW/AR issues.
    // START_DELAY : idle ACLK cycles before this master's first issue (used to
    //               offset the two masters so their events do not land on the
    //               same timestamps, making the trace easier to follow).
    parameter int ISSUE_GAP   = 0,
    parameter int START_DELAY = 0
)(
    input  wire                         ACLK,
    input  wire                         ARESETn,

    // AXI WRITE ADDRESS
    output reg                          AWVALID,
    input  wire                         AWREADY,
    output reg  [ID_WIDTH-1:0]          AWID,
    output reg  [ADDR_WIDTH-1:0]        AWADDR,
    output reg  [7:0]                   AWLEN,

    // AXI WRITE DATA
    output reg                          WVALID,
    input  wire                         WREADY,
    output reg  [DATA_WIDTH-1:0]        WDATA,
    output reg                          WLAST,

    // AXI WRITE RESPONSE
    input  wire                         BVALID,
    output reg                          BREADY,
    input  wire [ID_WIDTH-1:0]          BID,
    input  wire [1:0]                   BRESP,

    // AXI READ ADDRESS
    output reg                          ARVALID,
    input  wire                         ARREADY,
    output reg  [ID_WIDTH-1:0]          ARID,
    output reg  [ADDR_WIDTH-1:0]        ARADDR,
    output reg  [7:0]                   ARLEN,

    // AXI READ DATA
    input  wire                         RVALID,
    output reg                          RREADY,
    input  wire [ID_WIDTH-1:0]          RID,
    input  wire [DATA_WIDTH-1:0]        RDATA,
    input  wire                         RLAST,
    input  wire [1:0]                   RRESP,

    // Master ID: 0 or 1, only used to generate deterministic addresses/data.
    input  wire                         MASTER_ID,

    output reg                          protocol_error_bresp,
    output reg                          protocol_error_rresp,
    output reg                          protocol_error_bid,
    output reg                          protocol_error_rid,
    output reg                          protocol_error_rlast
);

    localparam int BURST_BEATS = 4;
    localparam int BURST_LEN   = BURST_BEATS - 1;
    localparam int TXN_IDX_W = (NUM_TRANSACTIONS <= 1) ? 1 : $clog2(NUM_TRANSACTIONS);

    typedef struct packed {
        logic [ID_WIDTH-1:0]   id;
        logic [ADDR_WIDTH-1:0] addr;
        logic [7:0]            len;
    } req_t; // Request structure for AW and AR channels. Used for deterministic request generation and tracking.
    /*  id ? AXI ID for the transaction
        addr ? address for the transaction
        len ? burst length (number of beats - 1) for the transaction  */

    req_t wr_req [0:NUM_TRANSACTIONS-1];
    req_t rd_req [0:NUM_TRANSACTIONS-1];

    // Deterministic request initialization for simulation.
    integer i;
    always_comb begin
        for (i = 0; i < NUM_TRANSACTIONS; i = i + 1) begin
            wr_req[i].id   = i;
            rd_req[i].id   = i;
            wr_req[i].len  = BURST_LEN[7:0];
            rd_req[i].len  = BURST_LEN[7:0];

            if (MASTER_ID == 1'b0) begin
                wr_req[i].addr = 32'h0000_1000 + (i * 32'h0000_0100);
                rd_req[i].addr = 32'h0000_3000 + (i * 32'h0000_0100);
            end else begin
                wr_req[i].addr = 32'h0000_4000 + (i * 32'h0000_0100);
                rd_req[i].addr = 32'h0000_2000 + (i * 32'h0000_0100);
            end
        end
    end

    function automatic logic [DATA_WIDTH-1:0] make_wdata(input int txn, input int beat);
        logic [DATA_WIDTH-1:0] base;
        begin
            base = MASTER_ID ? 128'hA000_0000_0000_0000_0000_0000_0000_0000
            : 128'h1000_0000_0000_0000_0000_0000_0000_0000;
            make_wdata = base | (txn << 8) | beat;
        end
    endfunction

    // Decode AXI response code for debug prints.
    function automatic string resp_str(input logic [1:0] resp);
        case (resp)
            2'b00:   resp_str = "OKAY";
            2'b01:   resp_str = "EXOKAY";
            2'b10:   resp_str = "SLVERR";
            default: resp_str = "DECERR";
        endcase
    endfunction

    // Returns msg when cond is set, otherwise a truly empty string. Using a
    // string-typed function avoids the zero-padding (trailing spaces) that an
    // unequal-width string-literal ternary would emit on the no-error case.
    function automatic string tag_if(input bit cond, input string msg);
        tag_if = cond ? msg : "";
    endfunction

    // ------------------------------------------------------------------
    // AW issuer: sends multiple write addresses independently of B.
    // ------------------------------------------------------------------
    logic [TXN_IDX_W:0] aw_issue_idx;
    logic [7:0]         aw_gap; // demo pacing: idle-cycle countdown between issues

    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            AWVALID <= 1'b0;
            AWID    <= '0;
            AWADDR  <= '0;
            AWLEN   <= '0;
            aw_issue_idx <= '0;
            aw_gap  <= START_DELAY[7:0]; // 0 by default; offsets this master's first issue
        end else begin
            if (aw_gap != 8'd0) begin
                aw_gap <= aw_gap - 8'd1; // demo pause: hold AWVALID low between issues
            end else if (!AWVALID && aw_issue_idx < NUM_TRANSACTIONS) begin
                AWID    <= wr_req[aw_issue_idx].id;
                AWADDR  <= wr_req[aw_issue_idx].addr;
                AWLEN   <= wr_req[aw_issue_idx].len;
                AWVALID <= 1'b1;
            end else if (AWVALID && AWREADY) begin
                aw_issue_idx <= aw_issue_idx + 1'b1;
                if ((aw_issue_idx + 1'b1) < NUM_TRANSACTIONS && ISSUE_GAP == 0) begin
                    // Back-to-back worst-case stress test (default): assert next immediately.
                    AWID    <= wr_req[aw_issue_idx + 1'b1].id;
                    AWADDR  <= wr_req[aw_issue_idx + 1'b1].addr;
                    AWLEN   <= wr_req[aw_issue_idx + 1'b1].len;
                    AWVALID <= 1'b1;
                end else begin
                    // Done, or (demo) pause ISSUE_GAP cycles before the next issue.
                    AWVALID <= 1'b0;
                    aw_gap  <= ISSUE_GAP[7:0];
                end
            end
            // With ISSUE_GAP = 0 and START_DELAY = 0 this is the original worst-case
            // throughput test, stressing outstanding-transaction tracking, FIFO
            // management, ID handling, queue-full and backpressure. A non-zero
            // ISSUE_GAP spaces out transactions so the trace is easier to watch.
        end
        
        // AW Debug
        if (DEBUG && AWVALID && AWREADY) begin
            $display("[%t] %-3s AW >> txn=M%0d.W%0d addr=0x%08h len=%0d  (issued %0d/%0d)",
                $time, (MASTER_ID ? "M1" : "M0"), MASTER_ID, AWID, AWADDR, AWLEN,
                aw_issue_idx + 1, NUM_TRANSACTIONS);
        end
    end

    // ------------------------------------------------------------------
    // W issuer: sends bursts in the same order as AW was issued.
    // This is the simple AXI4-compatible model: AXI4 has no WID.
    // ------------------------------------------------------------------
    logic [TXN_IDX_W:0] w_txn_idx;
    logic [7:0]         w_beat_idx;

    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            WVALID <= 1'b0;
            WDATA  <= '0;
            WLAST  <= 1'b0;
            w_txn_idx  <= '0;
            w_beat_idx <= '0;
        end else begin
            if (!WVALID && w_txn_idx < NUM_TRANSACTIONS) begin //
            // Start sending transaction 0, then 1, etc, in AW issue order.
                WVALID <= 1'b1;
                WDATA  <= make_wdata(w_txn_idx, 0);
                WLAST  <= (8'd0 == wr_req[w_txn_idx].len);
                w_beat_idx <= 8'd0;
            end else if (WVALID && WREADY) begin
                // Continue same transaction until DONE,
                if (w_beat_idx == wr_req[w_txn_idx].len) begin
                    // Only AFTER finishing txn N ? move to txn N+1.
                    w_txn_idx <= w_txn_idx + 1'b1;
                    w_beat_idx <= 8'd0;
                    if ((w_txn_idx + 1'b1) < NUM_TRANSACTIONS) begin
                        WVALID <= 1'b1;
                        WDATA  <= make_wdata(w_txn_idx + 1'b1, 0);
                        WLAST  <= (8'd0 == wr_req[w_txn_idx + 1'b1].len);
                    end else begin
                        WVALID <= 1'b0;
                        WLAST  <= 1'b0;
                    end
                end else begin
                    w_beat_idx <= w_beat_idx + 8'd1;
                    WDATA <= make_wdata(w_txn_idx, w_beat_idx + 8'd1);
                    WLAST <= ((w_beat_idx + 8'd1) == wr_req[w_txn_idx].len);
                end
            end
        end

        // W Debug (only show end of burst to reduce noise)
        if (DEBUG && WVALID && WREADY && WLAST) begin
            $display("[%t] %-3s W  >> txn=M%0d.W%0d beats=%0d last_wdata=0x%h",
                $time, (MASTER_ID ? "M1" : "M0"), MASTER_ID, w_txn_idx, w_beat_idx + 1, WDATA);
        end
    end

    // ------------------------------------------------------------------
    // B checker: responses should return with matching local BID in order.
    // ------------------------------------------------------------------
    logic [TXN_IDX_W:0] b_recv_idx;

    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            BREADY <= 1'b1;
            b_recv_idx <= '0;
            protocol_error_bresp <= 1'b0;
            protocol_error_bid   <= 1'b0;
        end else begin
            BREADY <= 1'b1;
            if (BVALID && BREADY) begin
                if (BRESP != 2'b00) protocol_error_bresp <= 1'b1;
                if (b_recv_idx >= NUM_TRANSACTIONS || BID !== wr_req[b_recv_idx].id) begin
                    protocol_error_bid <= 1'b1;
                end
                b_recv_idx <= b_recv_idx + 1'b1;
            end
        end

        //  B Debug
        if (DEBUG && BVALID && BREADY) begin
            $display("[%t] %-3s B  << txn=M%0d.W%0d resp=%-6s exp=M%0d.W%0d (%0d/%0d)%s%s",
                $time, (MASTER_ID ? "M1" : "M0"), MASTER_ID, BID, resp_str(BRESP),
                MASTER_ID, (b_recv_idx < NUM_TRANSACTIONS) ? wr_req[b_recv_idx].id : 'x,
                b_recv_idx + 1, NUM_TRANSACTIONS,
                tag_if(BRESP != 2'b00, " <-- ERROR RESP"),
                tag_if(b_recv_idx >= NUM_TRANSACTIONS || BID !== wr_req[b_recv_idx].id, " <-- ID MISMATCH"));
        end
    end

    // ------------------------------------------------------------------
    // AR issuer: sends multiple read addresses independently of R completion.
    // ------------------------------------------------------------------
    logic [TXN_IDX_W:0] ar_issue_idx;
    logic [7:0]         ar_gap; // demo pacing: idle-cycle countdown between issues

    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            ARVALID <= 1'b0;
            ARID    <= '0;
            ARADDR  <= '0;
            ARLEN   <= '0;
            ar_issue_idx <= '0;
            ar_gap  <= START_DELAY[7:0]; // 0 by default; offsets this master's first issue
        end else begin
            if (ar_gap != 8'd0) begin
                ar_gap <= ar_gap - 8'd1; // demo pause: hold ARVALID low between issues
            end else if (!ARVALID && ar_issue_idx < NUM_TRANSACTIONS) begin
                ARID    <= rd_req[ar_issue_idx].id;
                ARADDR  <= rd_req[ar_issue_idx].addr;
                ARLEN   <= rd_req[ar_issue_idx].len;
                ARVALID <= 1'b1;
            end else if (ARVALID && ARREADY) begin
                ar_issue_idx <= ar_issue_idx + 1'b1;
                if ((ar_issue_idx + 1'b1) < NUM_TRANSACTIONS && ISSUE_GAP == 0) begin
                    // Back-to-back worst-case stress test (default): assert next immediately.
                    ARID    <= rd_req[ar_issue_idx + 1'b1].id;
                    ARADDR  <= rd_req[ar_issue_idx + 1'b1].addr;
                    ARLEN   <= rd_req[ar_issue_idx + 1'b1].len;
                    ARVALID <= 1'b1;
                end else begin
                    // Done, or (demo) pause ISSUE_GAP cycles before the next issue.
                    ARVALID <= 1'b0;
                    ar_gap  <= ISSUE_GAP[7:0];
                end
            end
        end

        // AR Debug
        if (DEBUG && ARVALID && ARREADY) begin
            $display("[%t] %-3s AR >> txn=M%0d.R%0d addr=0x%08h len=%0d  (issued %0d/%0d)",
                $time, (MASTER_ID ? "M1" : "M0"), MASTER_ID, ARID, ARADDR, ARLEN,
                ar_issue_idx + 1, NUM_TRANSACTIONS);
        end
    end

    // ------------------------------------------------------------------
    // R checker: checks RRESP, RID and RLAST position per read transaction.
    // ------------------------------------------------------------------
    logic [TXN_IDX_W:0] r_txn_idx;
    logic [7:0]         r_beat_idx;
    logic [3:0]         rready_stall_cnt;

    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            RREADY <= 1'b1;
            r_txn_idx <= '0;
            r_beat_idx <= 8'd0;
            rready_stall_cnt <= 4'd0;
            protocol_error_rresp <= 1'b0;
            protocol_error_rid   <= 1'b0;
            protocol_error_rlast <= 1'b0;
        end else begin
            rready_stall_cnt <= rready_stall_cnt + 4'd1;
            RREADY <= (rready_stall_cnt[2:0] != 3'b101);

            if (RVALID && RREADY) begin
                if (RRESP != 2'b00) protocol_error_rresp <= 1'b1;
                if (r_txn_idx >= NUM_TRANSACTIONS || RID !== rd_req[r_txn_idx].id) begin
                    protocol_error_rid <= 1'b1;
                end
                if (RLAST !== (r_beat_idx == rd_req[r_txn_idx].len)) begin
                    protocol_error_rlast <= 1'b1;
                end

                if (r_beat_idx == rd_req[r_txn_idx].len) begin
                    r_txn_idx <= r_txn_idx + 1'b1;
                    r_beat_idx <= 8'd0;
                end else begin
                    r_beat_idx <= r_beat_idx + 8'd1;
                end
            end
        end

        // R Debug (only last beat)
        if (DEBUG && RVALID && RREADY && RLAST) begin
            $display("[%t] %-3s R  << txn=M%0d.R%0d resp=%-6s beats=%0d last_rdata=0x%h (%0d/%0d)%s%s",
                $time, (MASTER_ID ? "M1" : "M0"), MASTER_ID, RID, resp_str(RRESP), r_beat_idx + 1, RDATA,
                r_txn_idx + 1, NUM_TRANSACTIONS,
                tag_if(RRESP != 2'b00, " <-- ERROR RESP"),
                tag_if(r_txn_idx >= NUM_TRANSACTIONS || RID !== rd_req[r_txn_idx].id, " <-- ID MISMATCH"));
        end
    end

endmodule

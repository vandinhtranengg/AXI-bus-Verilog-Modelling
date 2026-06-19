// ======================================================================
//  simple_axi_slave.sv
//
//  PURPOSE
//    Simplified in-order AXI4 slave for simulation. Accepts write and read
//    transactions, returns OKAY responses with synthetic read data, and
//    checks WLAST placement.
//
//  FEATURES
//    - Queues up to MAX_OUTSTANDING_TRANSACTIONS write and read commands
//      independently (separate AW, AR and B FIFOs).
//    - Propagates AWID->BID and ARID->RID unchanged (the route bit the
//      arbiter added rides along in the ID and is echoed back).
//    - Generates BRESP/RRESP = OKAY and deterministic RDATA = f(addr, beat).
//    - Generates RLAST from the captured ARLEN and checks WLAST against the
//      captured AWLEN (protocol_error_wlast on mismatch).
//    - Backpressure: AWREADY/ARREADY drop when a queue is full; WREADY holds
//      off the final write beat until the B queue has room.
//
//  DATA PATH  (two independent FIFO paths)
//
//    WRITE:  AW --> [aw_q] --> [W consume / WLAST check] --> [b_q] --> B out
//    READ:   AR --> [ar_q] --> [R generate: RDATA/RLAST/RRESP] -----> R out
//
//    Each path serves commands strictly in acceptance order.
//
//  PARAMETERS
//    ID_WIDTH    - includes the arbiter's route bit (the interconnect drives
//                  ID_WIDTH = master-side ID_WIDTH + 1).
//    ADDR_WIDTH, DATA_WIDTH - bus geometry (DATA_WIDTH >= ADDR_WIDTH+8 so the
//                  synthetic read data can carry addr + beat).
//    MAX_OUTSTANDING_TRANSACTIONS - depth of the AW/AR/B queues.
//    DEBUG        - 1 enables the per-channel trace prints.
//    RESP_LATENCY - cycles the slave waits before presenting each B response
//                   and before the first R beat of each burst. 0 = respond as
//                   fast as possible (default); >0 models a slow slave and
//                   lets transactions accumulate in the queues. Ordering is
//                   preserved either way.
//
//  ASSUMPTIONS & LIMITS
//    - Intentionally in-order: B follows AW/W acceptance order and R follows
//      AR acceptance order. This is a legal AXI4 subset and matches the
//      in-order arbiter/masters; out-of-order completion is not modelled.
//
//  DEBUG TRACE  (full legend in the testbench header)
//    [time] SLV CH dir txn=M<route>.<id>  <fields>
//    The route bit is the top bit of the slave-side ID; <id> is the low bits.
// ======================================================================
module simple_axi_slave #(
    parameter int ID_WIDTH = 5, // Includes arbiter route bit.
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 128,
    parameter int MAX_OUTSTANDING_TRANSACTIONS = 4,
    parameter bit DEBUG = 1'b0, // 1 = enable prints, 0 = disable
    // Response latency: extra ACLK cycles the slave waits before presenting a
    // B response (after the write burst completes) and before the first R beat
    // (after an AR is ready to be served). 0 = respond as fast as possible
    // (default, original behavior); >0 models a slow slave, which lets multiple
    // transactions stack up in the queues. Ordering is unchanged.
    parameter int RESP_LATENCY = 0
)(
    input  wire                         ACLK,
    input  wire                         ARESETn,

    // AXI WRITE ADDRESS
    input  wire                         AWVALID,
    output reg                          AWREADY,
    input  wire [ID_WIDTH-1:0]          AWID,
    input  wire [ADDR_WIDTH-1:0]        AWADDR,
    input  wire [7:0]                   AWLEN,

    // AXI WRITE DATA
    input  wire                         WVALID,
    output reg                          WREADY,
    input  wire [DATA_WIDTH-1:0]        WDATA,
    input  wire                         WLAST,

    // AXI WRITE RESPONSE
    output reg                          BVALID,
    input  wire                         BREADY,
    output reg [ID_WIDTH-1:0]           BID,
    output reg [1:0]                    BRESP,

    // AXI READ ADDRESS
    input  wire                         ARVALID,
    output reg                          ARREADY,
    input  wire [ID_WIDTH-1:0]          ARID,
    input  wire [ADDR_WIDTH-1:0]        ARADDR,
    input  wire [7:0]                   ARLEN,

    // AXI READ DATA
    output reg                          RVALID,
    input  wire                         RREADY,
    output reg [ID_WIDTH-1:0]           RID,
    output reg [DATA_WIDTH-1:0]         RDATA,
    output reg                          RLAST,
    output reg [1:0]                    RRESP,

    output reg                          protocol_error_wlast
);

    localparam int PTR_WIDTH = (MAX_OUTSTANDING_TRANSACTIONS <= 1) ? 1 : $clog2(MAX_OUTSTANDING_TRANSACTIONS);
    localparam int CNT_WIDTH = $clog2(MAX_OUTSTANDING_TRANSACTIONS + 1);
    localparam logic [CNT_WIDTH-1:0] MAX_OUTSTANDING_COUNT = MAX_OUTSTANDING_TRANSACTIONS;

    initial begin
        if (MAX_OUTSTANDING_TRANSACTIONS < 1) begin
            $fatal(1, "MAX_OUTSTANDING_TRANSACTIONS must be >= 1");
        end
        if (DATA_WIDTH < ADDR_WIDTH + 8) begin
            $fatal(1, "DATA_WIDTH must be at least ADDR_WIDTH + 8 for synthetic read data");
        end
    end

    typedef struct packed {
        logic [ID_WIDTH-1:0]   id;
        logic [ADDR_WIDTH-1:0] addr;
        logic [7:0]            len;
    } req_t;

    typedef struct packed {
        logic [ID_WIDTH-1:0] id;
        logic [1:0]          resp;
    } bresp_t;

    req_t aw_q [0:MAX_OUTSTANDING_TRANSACTIONS-1];
    req_t ar_q [0:MAX_OUTSTANDING_TRANSACTIONS-1];
    bresp_t b_q [0:MAX_OUTSTANDING_TRANSACTIONS-1];

    logic [PTR_WIDTH-1:0] aw_head, aw_tail;
    logic [PTR_WIDTH-1:0] ar_head, ar_tail;
    logic [PTR_WIDTH-1:0] b_head,  b_tail;
    logic [CNT_WIDTH-1:0] aw_count, ar_count, b_count;

    logic [7:0] w_beat_count;
    logic [7:0] r_beat_count;
    logic [ADDR_WIDTH-1:0] active_raddr;
    logic [7:0] active_rlen;
    logic [ID_WIDTH-1:0] active_rid;
    logic reading;
    logic [7:0] b_lat_cnt; // counts down RESP_LATENCY before a B response is presented
    logic [7:0] r_lat_cnt; // counts down RESP_LATENCY before a read burst starts

    function automatic logic [PTR_WIDTH-1:0] ptr_inc(input logic [PTR_WIDTH-1:0] ptr);
        if (ptr == MAX_OUTSTANDING_TRANSACTIONS-1) ptr_inc = '0;
        else ptr_inc = ptr + {{(PTR_WIDTH-1){1'b0}}, 1'b1};
    endfunction

    wire aw_full  = (aw_count == MAX_OUTSTANDING_COUNT);
    wire ar_full  = (ar_count == MAX_OUTSTANDING_COUNT);
    wire b_full   = (b_count  == MAX_OUTSTANDING_COUNT);
    wire aw_empty = (aw_count == '0);
    wire ar_empty = (ar_count == '0);
    wire b_empty  = (b_count  == '0);

    wire aw_hs = AWVALID && AWREADY;
    wire ar_hs = ARVALID && ARREADY;
    wire w_hs  = WVALID  && WREADY;
    wire b_hs  = BVALID  && BREADY;
    wire r_hs  = RVALID  && RREADY;

    wire aw_done = w_hs && (w_beat_count == aw_q[aw_head].len);
    // wire b_done  = b_hs;

    // Ready to start a new read burst:
    //   - no read burst currently active
    //   - AND the AR queue contains a request
    //   - AND the R channel can accept a new burst (!RVALID = nothing presented,
    //     or r_hs = current beat finished so it may be replaced by the next).
    wire ar_ready = !reading && !ar_empty && (!RVALID || r_hs);
    // ar_pop also waits out the response-latency timer. With RESP_LATENCY = 0,
    // r_lat_cnt is always 0, so ar_pop == ar_ready (original behavior).
    wire ar_pop   = ar_ready && (r_lat_cnt == '0);

    always_comb begin
        AWREADY = !aw_full;
        ARREADY = !ar_full;
        // Accept W only when at least one write command is pending and B queue
        // has room for a response if this W beat is the final beat.
        WREADY  = !aw_empty && (!((w_beat_count == aw_q[aw_head].len) && b_full));
    end

    function automatic logic [DATA_WIDTH-1:0] make_rdata(
        input logic [ADDR_WIDTH-1:0] addr,
        input logic [7:0] beat
    );
        begin
            make_rdata = {{(DATA_WIDTH-ADDR_WIDTH-8){1'b0}}, addr, beat};
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

    // ------------------------------------------------------------------
    // Write path: AW queue push, W consume, B queue push, B output.
    // ------------------------------------------------------------------
    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            aw_head      <= '0;
            aw_tail      <= '0;
            aw_count     <= '0;
            w_beat_count <= 8'd0;
            b_head       <= '0;
            b_tail       <= '0;
            b_count      <= '0;
            BVALID       <= 1'b0;
            BID          <= '0;
            BRESP        <= 2'b00;
            b_lat_cnt    <= RESP_LATENCY[7:0];
            protocol_error_wlast <= 1'b0;
        end else begin
            // ---------------- AW queue push ----------------
            if (aw_hs) begin
                aw_q[aw_tail].id   <= AWID;
                aw_q[aw_tail].addr <= AWADDR;
                aw_q[aw_tail].len  <= AWLEN;
                aw_tail <= ptr_inc(aw_tail);
            end

            // ---------------- W consume and B queue push ----------------
            if (w_hs) begin
                if (WLAST !== (w_beat_count == aw_q[aw_head].len)) begin
                    protocol_error_wlast <= 1'b1;
                end
                if (w_beat_count == aw_q[aw_head].len) begin
                    b_q[b_tail].id   <= aw_q[aw_head].id;
                    b_q[b_tail].resp <= (WLAST ? 2'b00 : 2'b10);
                    b_tail       <= ptr_inc(b_tail);
                    aw_head      <= ptr_inc(aw_head);
                    w_beat_count <= 8'd0;
                end else begin
                    w_beat_count <= w_beat_count + 8'd1;
                end
            end

            // aw_count = AW accepted but W burst not completed
            // Note: aw_done is combinationally derived from w_hs and w_beat_count,
            // so it reflects the completion of the current beat in the same cycle as w_hs.
            unique case ({aw_hs, aw_done})
                2'b10:   aw_count <= aw_count + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                2'b01:   aw_count <= aw_count - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                default: aw_count <= aw_count;
            endcase

            // b_count = W burst completed but BRESP not yet consumed
            unique case ({aw_done, b_hs})
                2'b10:   b_count <= b_count + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                2'b01:   b_count <= b_count - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                default: b_count <= b_count;
            endcase

            // ---------------- B output ----------------
            // b_count includes the currently presented B response until the
            // BVALID/BREADY handshake completes. Therefore b_head advances
            // only on b_hs, not when BVALID is first asserted.
            if (!BVALID && !b_empty) begin
                if (b_lat_cnt != 8'd0) begin
                    b_lat_cnt <= b_lat_cnt - 8'd1; // model response latency before presenting B
                end else begin
                    BVALID <= 1'b1;
                    BID    <= b_q[b_head].id;
                    BRESP  <= b_q[b_head].resp;
                end
            end else if (b_hs) begin
                BVALID <= 1'b0;
                BRESP  <= 2'b00;
                b_head <= ptr_inc(b_head);
                b_lat_cnt <= RESP_LATENCY[7:0]; // arm latency for the next B
            end
        end

        // AW Debug
        if (DEBUG && ARESETn && aw_hs) begin
            $display("[%t] SLV AW >> txn=M%0d.W%0d addr=0x%08h len=%0d  aw_q=%0d/%0d",
                $time, AWID[ID_WIDTH-1], AWID[ID_WIDTH-2:0], AWADDR, AWLEN,
                aw_count + 1, MAX_OUTSTANDING_TRANSACTIONS);
        end

        // W Debug
        if (DEBUG && ARESETn && w_hs && WLAST) begin
            $display("[%t] SLV W  >> txn=M%0d.W%0d beats=%0d last_wdata=0x%h",
                $time, aw_q[aw_head].id[ID_WIDTH-1], aw_q[aw_head].id[ID_WIDTH-2:0], w_beat_count + 1, WDATA);
        end

        // B Debug
        if (DEBUG && ARESETn && b_hs) begin
            $display("[%t] SLV B  << txn=M%0d.W%0d resp=%-6s b_q=%0d/%0d pending",
                $time, BID[ID_WIDTH-1], BID[ID_WIDTH-2:0], resp_str(BRESP),
                b_count, MAX_OUTSTANDING_TRANSACTIONS);
        end
    end

    // ------------------------------------------------------------------
    // Read path: AR queue push, R generation.
    // ------------------------------------------------------------------
    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            ar_head      <= '0;
            ar_tail      <= '0;
            ar_count     <= '0;
            r_beat_count <= 8'd0;
            active_raddr <= '0;
            active_rlen  <= 8'd0;
            active_rid   <= '0;
            reading      <= 1'b0;
            RVALID       <= 1'b0;
            RID          <= '0;
            RDATA        <= '0;
            RLAST        <= 1'b0;
            RRESP        <= 2'b00;
            r_lat_cnt    <= RESP_LATENCY[7:0];
        end else begin
            // ---------------- AR queue push ----------------
            if (ar_hs) begin
                ar_q[ar_tail].id   <= ARID;
                ar_q[ar_tail].addr <= ARADDR;
                ar_q[ar_tail].len  <= ARLEN;
                ar_tail <= ptr_inc(ar_tail);
            end

            // ---------------- R generation ----------------
            if (ar_pop) begin
                active_rid   <= ar_q[ar_head].id;
                active_raddr <= ar_q[ar_head].addr;
                active_rlen  <= ar_q[ar_head].len;
                reading      <= 1'b1;
                r_beat_count <= 8'd0;
                RVALID <= 1'b1;
                RID    <= ar_q[ar_head].id;
                RDATA  <= make_rdata(ar_q[ar_head].addr, 8'd0);
                RLAST  <= (8'd0 == ar_q[ar_head].len);
                RRESP  <= 2'b00;
                ar_head <= ptr_inc(ar_head);
            end else if (reading && r_hs) begin
                if (r_beat_count == active_rlen) begin
                    reading      <= 1'b0;
                    RVALID       <= 1'b0;
                    RLAST        <= 1'b0;
                    RRESP        <= 2'b00;
                    r_beat_count <= 8'd0;
                end else begin
                    r_beat_count <= r_beat_count + 8'd1;
                    RVALID <= 1'b1;
                    RID    <= active_rid;
                    RDATA  <= make_rdata(active_raddr, r_beat_count + 8'd1);
                    RLAST  <= ((r_beat_count + 8'd1) == active_rlen);
                    RRESP  <= 2'b00;
                end
            end else if (r_hs && !reading) begin
                RVALID <= 1'b0;
                RLAST  <= 1'b0;
            end

            // Response-latency timer: while a read is pending but not yet
            // started, count down before the first R beat is produced. Armed
            // again on each pop so every burst pays the latency.
            if (ar_pop)
                r_lat_cnt <= RESP_LATENCY[7:0];
            else if (ar_ready && r_lat_cnt != 8'd0)
                r_lat_cnt <= r_lat_cnt - 8'd1;

            // ar_count = AR accepted but R burst not completed
            // Note: If both ar_hs and ar_pop are '1' in the same cycle, a new AR is
            // accepted at the same time the previous burst pops - count stays unchanged.
            unique case ({ar_hs, ar_pop})
                2'b10:   ar_count <= ar_count + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                2'b01:   ar_count <= ar_count - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                default: ar_count <= ar_count;
            endcase
        end

        // AR Debug
        if (DEBUG && ARESETn && ar_hs) begin
            $display("[%t] SLV AR >> txn=M%0d.R%0d addr=0x%08h len=%0d  ar_q=%0d/%0d",
                $time, ARID[ID_WIDTH-1], ARID[ID_WIDTH-2:0], ARADDR, ARLEN,
                ar_count + 1, MAX_OUTSTANDING_TRANSACTIONS);
        end

        // R Debug
        if (DEBUG && ARESETn && r_hs && RLAST) begin
            $display("[%t] SLV R  << txn=M%0d.R%0d resp=%-6s beats=%0d last_rdata=0x%h",
                $time, RID[ID_WIDTH-1], RID[ID_WIDTH-2:0], resp_str(RRESP), r_beat_count + 1, RDATA);
        end
    end

endmodule

// ======================================================================
// axi4_simple_2to1_bus_arbiter.sv
//
// Compact 2-master / 1-slave AXI4-style interconnect (round-robin),
// for simulation. Not a full commercial crossbar.
//
//   - Master IDs are widened on the slave side by one routing bit:
//       S_*ID = {master_select, Mx_*ID}
//   - MAX_OUTSTANDING_TRANSACTIONS independently limits accepted-but-
//     incomplete write and read transactions; many may be in flight.
//   - AXI4 has no WID, so W bursts are forwarded strictly in accepted-AW
//     order (W routing follows the head of the write queue).
//   - B/R responses route back to the originating master via the high
//     routing bit of BID/RID, which is then stripped off.
//   - Burst length and ID-order consistency are checked by protocol flags.
//
// Structure
//   Mostly a datapath: combinational request routing plus per-channel
//   outstanding-transaction trackers built from FIFO queues (wr_q, rd_q)
//   and saturating counters (wr_count, wr_w_count, rd_count). Because many
//   transactions can be outstanding at once, the queue depth - not a single
//   "current transaction" enum - captures the in-flight set, so AW/AR
//   acceptance, B checking, and the counters stay reactive datapath.
//   Round-robin priority is a 1-bit last-winner register per channel.
//
//   The two genuinely multi-cycle per-channel sequences are expressed as
//   explicit FSMs that walk the beats of the head burst:
//     * W-data forwarding : WF_IDLE -> WF_BURST  (beat tracking + WLAST check)
//     * R-data return     : RR_IDLE -> RR_BURST  (beat tracking + RID/RLAST check)
//   Their state is redundant with the beat counter (IDLE == count 0, between
//   bursts); beat handling stays gated on the real w_hs/r_hs handshake so a
//   burst still starts the cycle its queue fills - the enum only makes the
//   progression explicit. Logic is grouped into a combinational routing
//   block, a write-path tracker, and a read-path tracker.
//
// Ordering assumption
//   Strict FIFO completion is enforced:
//       AW order --> W order --> B order
//       AR order --> R order
//   A slave that returns B/R out of order trips protocol_error_bid_order
//   or protocol_error_rid_order. Compatible only with in-order slaves.
// ======================================================================


module axi4_simple_2to1_bus_arbiter #(
    parameter int ID_WIDTH = 4,
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 128,
    parameter int MAX_OUTSTANDING_TRANSACTIONS = 4,
    parameter bit DEBUG = 1'b0 // 1 = enable prints, 0 = disable
)(
    input  wire                         ACLK,
    input  wire                         ARESETn,

    // MASTER 0 WRITE ADDRESS
    input  wire                         M0_AWVALID,
    output reg                          M0_AWREADY,
    input  wire [ID_WIDTH-1:0]          M0_AWID,
    input  wire [ADDR_WIDTH-1:0]        M0_AWADDR,
    input  wire [7:0]                   M0_AWLEN,

    // MASTER 0 WRITE DATA
    input  wire                         M0_WVALID,
    output reg                          M0_WREADY,
    input  wire [DATA_WIDTH-1:0]        M0_WDATA,
    input  wire                         M0_WLAST,

    // MASTER 0 WRITE RESPONSE
    output reg                          M0_BVALID,
    input  wire                         M0_BREADY,
    output reg [ID_WIDTH-1:0]           M0_BID,
    output reg [1:0]                    M0_BRESP,

    // MASTER 0 READ ADDRESS
    input  wire                         M0_ARVALID,
    output reg                          M0_ARREADY,
    input  wire [ID_WIDTH-1:0]          M0_ARID,
    input  wire [ADDR_WIDTH-1:0]        M0_ARADDR,
    input  wire [7:0]                   M0_ARLEN,

    // MASTER 0 READ DATA
    output reg                          M0_RVALID,
    input  wire                         M0_RREADY,
    output reg [ID_WIDTH-1:0]           M0_RID,
    output reg [DATA_WIDTH-1:0]         M0_RDATA,
    output reg                          M0_RLAST,
    output reg [1:0]                    M0_RRESP,

    // MASTER 1 WRITE ADDRESS
    input  wire                         M1_AWVALID,
    output reg                          M1_AWREADY,
    input  wire [ID_WIDTH-1:0]          M1_AWID,
    input  wire [ADDR_WIDTH-1:0]        M1_AWADDR,
    input  wire [7:0]                   M1_AWLEN,

    // MASTER 1 WRITE DATA
    input  wire                         M1_WVALID,
    output reg                          M1_WREADY,
    input  wire [DATA_WIDTH-1:0]        M1_WDATA,
    input  wire                         M1_WLAST,

    // MASTER 1 WRITE RESPONSE
    output reg                          M1_BVALID,
    input  wire                         M1_BREADY,
    output reg [ID_WIDTH-1:0]           M1_BID,
    output reg [1:0]                    M1_BRESP,

    // MASTER 1 READ ADDRESS
    input  wire                         M1_ARVALID,
    output reg                          M1_ARREADY,
    input  wire [ID_WIDTH-1:0]          M1_ARID,
    input  wire [ADDR_WIDTH-1:0]        M1_ARADDR,
    input  wire [7:0]                   M1_ARLEN,

    // MASTER 1 READ DATA
    output reg                          M1_RVALID,
    input  wire                         M1_RREADY,
    output reg [ID_WIDTH-1:0]           M1_RID,
    output reg [DATA_WIDTH-1:0]         M1_RDATA,
    output reg                          M1_RLAST,
    output reg [1:0]                    M1_RRESP,

    // SLAVE WRITE ADDRESS. Slave-side ID has one extra route bit for routing and checking.
    output reg                          S_AWVALID,
    input  wire                         S_AWREADY,
    output reg [ID_WIDTH:0]             S_AWID, // High bit is route (master select), low bits are ID.
    output reg [ADDR_WIDTH-1:0]         S_AWADDR,
    output reg [7:0]                    S_AWLEN,

    // SLAVE WRITE DATA
    output reg                          S_WVALID,
    input  wire                         S_WREADY,
    output reg [DATA_WIDTH-1:0]         S_WDATA,
    output reg                          S_WLAST,

    // SLAVE WRITE RESPONSE
    input  wire                         S_BVALID,
    output reg                          S_BREADY,
    input  wire [ID_WIDTH:0]            S_BID, // High bit is route (master select), low bits are ID.
    input  wire [1:0]                   S_BRESP,

    // SLAVE READ ADDRESS
    output reg                          S_ARVALID,
    input  wire                         S_ARREADY,
    output reg [ID_WIDTH:0]             S_ARID, // High bit is route (master select), low bits are ID.
    output reg [ADDR_WIDTH-1:0]         S_ARADDR,
    output reg [7:0]                    S_ARLEN,

    // SLAVE READ DATA
    input  wire                         S_RVALID,
    output reg                          S_RREADY,
    input  wire [ID_WIDTH:0]            S_RID, // High bit is route (master select), low bits are ID.
    input  wire [DATA_WIDTH-1:0]        S_RDATA,
    input  wire                         S_RLAST,
    input  wire [1:0]                   S_RRESP,

    // Protocol status flags.
    output reg                          protocol_error_wlast,
    output reg                          protocol_error_rlast,
    output reg                          protocol_error_bid_order,
    output reg                          protocol_error_rid_order,
    output reg                          protocol_error_outstanding_overflow
);

    //localparam int ROUTE_ID_WIDTH = ID_WIDTH + 1;
    localparam int PTR_WIDTH = (MAX_OUTSTANDING_TRANSACTIONS <= 1) ? 1 : $clog2(MAX_OUTSTANDING_TRANSACTIONS);
    localparam int CNT_WIDTH = $clog2(MAX_OUTSTANDING_TRANSACTIONS + 1);
    localparam logic [CNT_WIDTH-1:0] MAX_OUTSTANDING_COUNT = MAX_OUTSTANDING_TRANSACTIONS;

    initial begin
        if (MAX_OUTSTANDING_TRANSACTIONS < 1) begin
            $fatal(1, "MAX_OUTSTANDING_TRANSACTIONS must be >= 1");
        end
    end

    typedef struct packed {
        logic                 route;
        logic [ID_WIDTH-1:0]  id;
        logic [7:0]           len;
    } txn_t; // Transaction tracking for AW and AR channels. 
    /*  route → which master (0 or 1)
        id → AXI ID
        len → burst length  */

    // Accepted write-address transactions. This queue drives W routing and
    // expected B ID checking. AXI4 has no WID, so W follows AW order.
    txn_t wr_q [0:MAX_OUTSTANDING_TRANSACTIONS-1];
    logic [PTR_WIDTH-1:0] wr_w_head, wr_b_head, wr_tail;
    logic [CNT_WIDTH-1:0] wr_count; // AW accepted, B not completed
    logic [CNT_WIDTH-1:0] wr_w_count; // AW accepted, W burst not completed

    // Accepted read-address transactions. This queue checks returned RID/RLAST.
    txn_t rd_q [0:MAX_OUTSTANDING_TRANSACTIONS-1];
    logic [PTR_WIDTH-1:0] rd_head, rd_tail;
    logic [CNT_WIDTH-1:0] rd_count;

    logic wr_last_winner;
    logic rd_last_winner;

    logic [7:0] wr_beat_count;
    logic [7:0] rd_beat_count;

    // Per-channel burst-tracking FSMs. The state is intentionally redundant
    // with the beat counter (WF_IDLE/RR_IDLE == "beat count is 0, between
    // bursts at the queue head"); it makes the burst progression explicit
    // without changing timing - beat handling stays gated on the real
    // w_hs / r_hs handshake so a burst can start the cycle its queue fills.
    typedef enum logic {WF_IDLE, WF_BURST} wf_state_t; // W-data forwarding
    typedef enum logic {RR_IDLE, RR_BURST} rr_state_t; // R-data return
    wf_state_t wf_state;
    rr_state_t rr_state;

    wire wr_q_full  = (wr_count == MAX_OUTSTANDING_COUNT);
    wire wr_q_empty = (wr_count == '0);
    wire wr_w_empty = (wr_w_count == '0);
    wire rd_q_full  = (rd_count == MAX_OUTSTANDING_COUNT);
    wire rd_q_empty = (rd_count == '0);

    // Arbitration — the round-robin logic is here. If both masters are requesting, the one that won last time loses.
    wire wr_both = M0_AWVALID && M1_AWVALID;
    wire aw_choose_m0 = M0_AWVALID && (!wr_both || wr_last_winner == 1'b1);
    wire aw_choose_m1 = M1_AWVALID && (!wr_both || wr_last_winner == 1'b0);

    wire rd_both = M0_ARVALID && M1_ARVALID;
    wire ar_choose_m0 = M0_ARVALID && (!rd_both || rd_last_winner == 1'b1);
    wire ar_choose_m1 = M1_ARVALID && (!rd_both || rd_last_winner == 1'b0);
    //--------------------------------------------------------------------

    // Handshakes. These indicate when transactions are accepted and drive the sequential tracking logic.
    wire aw_hs = S_AWVALID && S_AWREADY;
    wire w_hs  = S_WVALID  && S_WREADY;
    wire b_hs  = S_BVALID  && S_BREADY;
    wire ar_hs = S_ARVALID && S_ARREADY;
    wire r_hs  = S_RVALID  && S_RREADY;

    wire b_route = S_BID[ID_WIDTH]; // High route bit of slave-side BID indicates which master to route to.
    wire r_route = S_RID[ID_WIDTH]; // High route bit of slave-side RID indicates which master to route to.

    wire wr_done = w_hs && !wr_w_empty && (wr_beat_count == wr_q[wr_w_head].len); // A write burst is done when the last beat of the burst is accepted.
    wire rd_done = r_hs && !rd_q_empty && (rd_beat_count == rd_q[rd_head].len); // A read burst is done when the last beat of the burst is accepted.

    function automatic logic [PTR_WIDTH-1:0] ptr_inc(input logic [PTR_WIDTH-1:0] ptr);
        if (ptr == MAX_OUTSTANDING_TRANSACTIONS-1) ptr_inc = '0;
        else ptr_inc = ptr + {{(PTR_WIDTH-1){1'b0}}, 1'b1};
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
    // Combinational routing
    // ------------------------------------------------------------------
    always_comb begin
        // Defaults
        M0_AWREADY = 1'b0;
        M1_AWREADY = 1'b0;
        S_AWVALID  = 1'b0;
        S_AWID     = '0;
        S_AWADDR   = '0;
        S_AWLEN    = '0;

        M0_WREADY = 1'b0;
        M1_WREADY = 1'b0;
        S_WVALID  = 1'b0;
        S_WDATA   = '0;
        S_WLAST   = 1'b0;

        M0_BVALID = 1'b0;
        M1_BVALID = 1'b0;
        M0_BID    = S_BID[ID_WIDTH-1:0]; // High route bit of slave-side BID is not forwarded to masters, but is used for routing and checking.
        M1_BID    = S_BID[ID_WIDTH-1:0];
        M0_BRESP  = S_BRESP;
        M1_BRESP  = S_BRESP;
        S_BREADY  = 1'b0;

        M0_ARREADY = 1'b0;
        M1_ARREADY = 1'b0;
        S_ARVALID  = 1'b0;
        S_ARID     = '0;
        S_ARADDR   = '0;
        S_ARLEN    = '0;

        M0_RVALID = 1'b0;
        M1_RVALID = 1'b0;
        M0_RID    = S_RID[ID_WIDTH-1:0]; // High route bit of slave-side RID is not forwarded to masters, but is used for routing and checking.
        M1_RID    = S_RID[ID_WIDTH-1:0];
        M0_RDATA  = S_RDATA;
        M1_RDATA  = S_RDATA;
        M0_RLAST  = S_RLAST;
        M1_RLAST  = S_RLAST;
        M0_RRESP  = S_RRESP;
        M1_RRESP  = S_RRESP;
        S_RREADY  = 1'b0;

        // AW channel: accept more than one outstanding transaction, limited by wr_q_full.
        if (!wr_q_full) begin
            if (aw_choose_m0) begin
                S_AWVALID  = M0_AWVALID;
                S_AWID     = {1'b0, M0_AWID}; // High route bit is master select (0 for M0), low bits are ID.
                S_AWADDR   = M0_AWADDR;
                S_AWLEN    = M0_AWLEN;
                M0_AWREADY = S_AWREADY;
            end else if (aw_choose_m1) begin
                S_AWVALID  = M1_AWVALID;
                S_AWID     = {1'b1, M1_AWID}; // High route bit is master select (1 for M1), low bits are ID.
                S_AWADDR   = M1_AWADDR;
                S_AWLEN    = M1_AWLEN;
                M1_AWREADY = S_AWREADY;
            end
        end

        // W channel: route according to the oldest accepted AW transaction.
        if (!wr_w_empty) begin
            if (wr_q[wr_w_head].route == 1'b0) begin
                S_WVALID  = M0_WVALID;
                S_WDATA   = M0_WDATA;
                S_WLAST   = M0_WLAST;
                M0_WREADY = S_WREADY;
            end else begin
                S_WVALID  = M1_WVALID;
                S_WDATA   = M1_WDATA;
                S_WLAST   = M1_WLAST;
                M1_WREADY = S_WREADY;
            end
        end

        // B channel: route by high route bit of slave-side BID.
        if (S_BVALID) begin
            if (b_route == 1'b0) begin
                M0_BVALID = S_BVALID;
                S_BREADY  = M0_BREADY;
            end else begin
                M1_BVALID = S_BVALID;
                S_BREADY  = M1_BREADY;
            end
        end

        // AR channel: accept more than one outstanding transaction, limited by rd_q_full.
        if (!rd_q_full) begin
            if (ar_choose_m0) begin
                S_ARVALID  = M0_ARVALID;
                S_ARID     = {1'b0, M0_ARID};
                S_ARADDR   = M0_ARADDR;
                S_ARLEN    = M0_ARLEN;
                M0_ARREADY = S_ARREADY;
            end else if (ar_choose_m1) begin
                S_ARVALID  = M1_ARVALID;
                S_ARID     = {1'b1, M1_ARID};
                S_ARADDR   = M1_ARADDR;
                S_ARLEN    = M1_ARLEN;
                M1_ARREADY = S_ARREADY;
            end
        end

        // R channel: route by high route bit of slave-side RID.
        if (S_RVALID) begin
            if (r_route == 1'b0) begin
                M0_RVALID = S_RVALID;
                S_RREADY  = M0_RREADY;
            end else begin
                M1_RVALID = S_RVALID;
                S_RREADY  = M1_RREADY;
            end
        end
    end

    // ------------------------------------------------------------------
    // Write path: AW queue push, W beat tracking, B order checking.
    // protocol_error_outstanding_overflow is here because it is driven
    // by both AW and AR overflow — a single register needs one driver.
    // ------------------------------------------------------------------
    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            wr_w_head      <= '0;
            wr_b_head      <= '0;
            wr_tail        <= '0;
            wr_count       <= '0;
            wr_w_count     <= '0;
            wr_last_winner <= 1'b1;
            wr_beat_count  <= 8'd0;
            wf_state       <= WF_IDLE;
            protocol_error_wlast              <= 1'b0;
            protocol_error_bid_order          <= 1'b0;
            protocol_error_outstanding_overflow <= 1'b0;
        end else begin
            // AW accepted: push into write queue for W routing and B order checking.
            if (aw_hs) begin
                if (wr_q_full) begin
                    protocol_error_outstanding_overflow <= 1'b1;
                end else begin
                    wr_q[wr_tail].route <= S_AWID[ID_WIDTH];
                    wr_q[wr_tail].id    <= S_AWID[ID_WIDTH-1:0];
                    wr_q[wr_tail].len   <= S_AWLEN;
                    wr_tail        <= ptr_inc(wr_tail);
                    wr_last_winner <= S_AWID[ID_WIDTH];
                end
                // Note: wr_count updated below to support simultaneous accept/complete.
            end

            // AR overflow also sets the shared flag (AR queue managed in read path block).
            if (ar_hs && rd_q_full) begin
                protocol_error_outstanding_overflow <= 1'b1;
            end

            // W-data forwarding FSM: walks the beats of the head burst and
            // checks WLAST against its AWLEN. Gated on the real w_hs so the
            // first beat is caught the cycle the queue becomes non-empty.
            if (w_hs && !wr_w_empty) begin
                if (S_WLAST !== (wr_beat_count == wr_q[wr_w_head].len)) begin
                    protocol_error_wlast <= 1'b1;
                end
                unique case (wf_state)
                    WF_IDLE: begin // first beat of the head burst
                        if (wr_beat_count == wr_q[wr_w_head].len) begin
                            // single-beat burst: complete and stay idle
                            wr_beat_count <= 8'd0;
                            wr_w_head     <= ptr_inc(wr_w_head);
                        end else begin
                            wr_beat_count <= wr_beat_count + 8'd1;
                            wf_state      <= WF_BURST;
                        end
                    end
                    WF_BURST: begin // mid burst
                        if (wr_beat_count == wr_q[wr_w_head].len) begin
                            wr_beat_count <= 8'd0;
                            wr_w_head     <= ptr_inc(wr_w_head);
                            wf_state      <= WF_IDLE;
                        end else begin
                            wr_beat_count <= wr_beat_count + 8'd1;
                        end
                    end
                    default: wf_state <= WF_IDLE;
                endcase
            end

            // B accepted: check BID matches oldest write transaction, then pop.
            if (b_hs) begin
                if (wr_q_empty) begin
                    protocol_error_bid_order <= 1'b1;
                end else begin
                    if (S_BID !== {wr_q[wr_b_head].route, wr_q[wr_b_head].id}) begin
                        protocol_error_bid_order <= 1'b1;
                    end
                    wr_b_head <= ptr_inc(wr_b_head);
                end
            end

            // wr_count = AW accepted, B not completed
            unique case ({aw_hs, b_hs})
                2'b10:   wr_count <= wr_count + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                2'b01:   wr_count <= wr_count - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                default: wr_count <= wr_count;
            endcase

            // wr_w_count = AW accepted, W burst not completed
            unique case ({aw_hs, wr_done})
                2'b10:   wr_w_count <= wr_w_count + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                2'b01:   wr_w_count <= wr_w_count - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                default: wr_w_count <= wr_w_count;
            endcase
        end

        // AW Debug
        if (DEBUG && aw_hs) begin
            $display("[%0t][ARB][AW] route=M%0d id=%0d addr=0x%h len=%0d (wr_q %0d/%0d, both_req=%0b)",
                $time, S_AWID[ID_WIDTH], S_AWID[ID_WIDTH-1:0], S_AWADDR, S_AWLEN,
                wr_count + 1, MAX_OUTSTANDING_TRANSACTIONS, wr_both);
        end

        // W Debug (only last beat)
        if (DEBUG && w_hs && S_WLAST) begin
            $display("[%0t][ARB][W DONE] route=M%0d id=%0d beats=%0d",
                $time, wr_q[wr_w_head].route, wr_q[wr_w_head].id, wr_beat_count + 1);
        end

        // B Debug
        if (DEBUG && b_hs) begin
            $display("[%0t][ARB][B] route=M%0d id=%0d resp=%s (wr_q %0d/%0d pending)",
                $time, S_BID[ID_WIDTH], S_BID[ID_WIDTH-1:0], resp_str(S_BRESP),
                wr_count, MAX_OUTSTANDING_TRANSACTIONS);
        end
    end

    // ------------------------------------------------------------------
    // Read path: AR queue push, R beat tracking, RID/RLAST order checking.
    // ------------------------------------------------------------------
    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            rd_head        <= '0;
            rd_tail        <= '0;
            rd_count       <= '0;
            rd_last_winner <= 1'b1;
            rd_beat_count  <= 8'd0;
            rr_state       <= RR_IDLE;
            protocol_error_rlast     <= 1'b0;
            protocol_error_rid_order <= 1'b0;
        end else begin
            // AR accepted: push into read queue for R ID/order and RLAST checking.
            if (ar_hs) begin
                if (!rd_q_full) begin
                    rd_q[rd_tail].route <= S_ARID[ID_WIDTH];
                    rd_q[rd_tail].id    <= S_ARID[ID_WIDTH-1:0];
                    rd_q[rd_tail].len   <= S_ARLEN;
                    rd_tail        <= ptr_inc(rd_tail);
                    rd_last_winner <= S_ARID[ID_WIDTH];
                end
            end

            // R-data return FSM: walks the beats of the head read burst and
            // checks RID/RLAST against the oldest accepted AR. Gated on the
            // real r_hs so the first beat is caught immediately.
            if (r_hs) begin
                if (rd_q_empty) begin
                    protocol_error_rid_order <= 1'b1;
                end else begin
                    if (S_RID !== {rd_q[rd_head].route, rd_q[rd_head].id}) begin
                        protocol_error_rid_order <= 1'b1;
                    end
                    if (S_RLAST !== (rd_beat_count == rd_q[rd_head].len)) begin
                        protocol_error_rlast <= 1'b1;
                    end
                    unique case (rr_state)
                        RR_IDLE: begin // first beat of the head burst
                            if (rd_beat_count == rd_q[rd_head].len) begin
                                // single-beat burst: complete and stay idle
                                rd_beat_count <= 8'd0;
                                rd_head       <= ptr_inc(rd_head);
                            end else begin
                                rd_beat_count <= rd_beat_count + 8'd1;
                                rr_state      <= RR_BURST;
                            end
                        end
                        RR_BURST: begin // mid burst
                            if (rd_beat_count == rd_q[rd_head].len) begin
                                rd_beat_count <= 8'd0;
                                rd_head       <= ptr_inc(rd_head);
                                rr_state      <= RR_IDLE;
                            end else begin
                                rd_beat_count <= rd_beat_count + 8'd1;
                            end
                        end
                        default: rr_state <= RR_IDLE;
                    endcase
                end
            end

            // rd_count = AR accepted, R burst not completed
            unique case ({ar_hs, rd_done})
                2'b10:   rd_count <= rd_count + {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                2'b01:   rd_count <= rd_count - {{(CNT_WIDTH-1){1'b0}}, 1'b1};
                default: rd_count <= rd_count;
            endcase
        end

        // AR Debug
        if (DEBUG && ar_hs) begin
            $display("[%0t][ARB][AR] route=%0d id=%0d addr=0x%h",
                $time, S_ARID[ID_WIDTH], S_ARID[ID_WIDTH-1:0], S_ARADDR);
        end

        // R Debug (only last beat)
        if (DEBUG && r_hs && S_RLAST) begin
            $display("[%0t][ARB][R DONE] route=%0d id=%0d",
                $time, S_RID[ID_WIDTH], S_RID[ID_WIDTH-1:0]);
        end
    end

endmodule

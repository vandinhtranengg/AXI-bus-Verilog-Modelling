`timescale 1ns/1ps

// ======================================================================
//  tb_axi_bus_system.sv
//
//  PURPOSE
//    Top-level testbench for the 2-master / 1-slave AXI4 bus system:
//    two simple_axi_master -> axi4_simple_2to1_bus_arbiter -> simple_axi_slave.
//    It drives traffic, self-checks protocol behavior, and reports PASS/FAIL.
//
//  SYSTEM DATA PATH
//
//    master0 --+  AW/W/AR >                            > AW/W/AR +-- slave
//              +-----------  2:1 round-robin arbiter  -----------+   (in-order
//    master1 --+  < B/R      - RR arbitration            B/R >   +--  responder)
//                            - ID widen: S_*ID = {route, id}
//                            - wr_q / rd_q order FIFOs
//              ID_WIDTH bits       ID_WIDTH+1 bits (route bit added)
//
//  WHAT IT CHECKS
//    1. Outstanding AW/AR : both masters issue NUM_TRANSACTIONS without
//       waiting for responses; expects 2*NUM AW and 2*NUM AR at the slave.
//    2. ID routing/order  : slave echoes BID/RID = received AWID/ARID; the
//       arbiter strips the route bit; each master must see its own IDs back.
//    3. Burst & response  : BURST_BEATS-beat bursts; WLAST/RLAST checked at
//       every hop; BRESP/RRESP must be OKAY.
//    4. Ready/valid hold  : payload must not change while VALID is high and
//       READY is low on representative channels.
//
//  HOW IT WORKS
//    - Clock and reset generators, then the three DUT instances.
//    - Scoreboard counters tally B/R/AW/W/AR handshakes on each side.
//    - A completion process waits for all expected counts, prints PASS and
//      calls $finish.
//    - A protocol-error monitor turns any checker flag into a specific
//      $fatal that names the failing check.
//    - A ready/valid stability monitor $fatals on payload change under stall.
//    - A watchdog $fatals if the run never completes; WATCHDOG_NS scales with
//      NUM_TRANSACTIONS and the demo pacing so it does not false-trip.
//
//  DEMO PACING  (readability knobs; do NOT affect correctness)
//    DEMO          - 1 spaces transactions out; 0 = back-to-back stress test.
//    DEMO_GAP_0/_1 - idle cycles between a master's successive issues.
//    DEMO_OFFSET   - master1 start offset, in cycles.
//    See the in-file comments for how these interact with the round-robin.
//
//  DEBUG TRACE LEGEND
//    [time] SRC CH dir txn=M<route>.<W|R><id>  <fields>
//      SRC = M0|M1 (master), ARB (arbiter), SLV (slave)
//      CH  = AW/W/B (write path), AR/R (read path)
//      dir = >> request (master->slave), << response (slave->master)
//      txn = M<route>.<W|R><id> : route = originating master, W/R = write or
//            read path, id = local AXI ID. The same key appears on every hop,
//            and write vs read keys are distinct, so a single transaction is
//            traceable end-to-end - e.g. grep 'txn=M0.W2' (write) or
//            grep 'txn=M0.R2' (read).
//
//  HOW TO RUN
//    The full run exceeds the simulator's default 1000 ns window for large
//    NUM_TRANSACTIONS. Use 'run all' (runs to $finish/$fatal), or set the
//    simulation runtime (e.g. xsim.simulate.runtime) accordingly.
// ======================================================================

module tb_axi_bus_system;

    localparam int ID_WIDTH = 4;
    localparam int ADDR_WIDTH = 32;
    localparam int DATA_WIDTH = 128;
    localparam int NUM_TRANSACTIONS = 16; // Number of AW/AR transactions each master will issue.
    localparam int MAX_OUTSTANDING_TRANSACTIONS = 16;
    localparam int BURST_BEATS = 4;
    // 1 = enable debug prints in masters/arbiter/slave.
    localparam bit DEBUG = 1'b1;

    // DEMO = 1 spaces transactions out and offsets master1 so the trace is
    //          easy to watch (one transaction's events do not pile onto the next).
    // DEMO = 0 restores the original back-to-back worst-case stress test.
    localparam bit DEMO         = 1'b1;
    // To exercise the round-robin arbiter on EVERY transaction, the two masters
    // must contend (assert AW/AR in the same cycle) repeatedly:
    //   - DEMO_OFFSET = 0 so they start together and the first issue collides.
    //   - After each contention the arbiter serves the loser on the very next
    //     cycle, which would offset the two masters by one cycle forever. Making
    //     master0's gap exactly ONE cycle larger than master1's cancels that
    //     offset, so both realign and contend again at every issue.
    localparam int DEMO_GAP_0   = DEMO ? 2 : 0;                 // master0 idle cycles between issues
    localparam int DEMO_OFFSET  = 0;                            // both start together so issues collide -> contention

    // Master 1 is the realistic master: bounded-random pacing on all five
    // channels, driven by per-channel LFSRs seeded from M1_SEED (reproducible).
    // M1_GAP_MAX bounds the random AW/AR idle gap and feeds the watchdog below.
    localparam int M1_SEED        = 2;
    localparam int M1_GAP_MIN     = DEMO ? 1 : 0;
    localparam int M1_GAP_MAX     = DEMO ? 12 : 4;
    localparam int M1_WVALID_PROB = 70;   // % chance/cycle of presenting a queued write beat
    localparam int M1_BREADY_PROB = 80;   // % chance/cycle BREADY asserted
    localparam int M1_RREADY_PROB = 75;   // % chance/cycle RREADY asserted
    localparam int M1_CMD_DEPTH   = 16;    // write/read command-queue depth (max outstanding)
    localparam int M1_WDATA_FILL_LATENCY = 6; // fixed cycles before a write burst's first data beat

    // Slave response latency (cycles before B and before the first R beat).
    // In demo mode it gives the slave a realistic delay and lets transactions
    // stack up in the queues; 0 in stress mode keeps max throughput.
    localparam int SLAVE_RESP_LATENCY = DEMO ? 3 : 0;

    reg ACLK;
    reg ARESETn;

    initial begin
        ACLK = 1'b0;
        forever #5 ACLK = ~ACLK;
    end

    initial begin
        ARESETn = 1'b0;
        repeat (5) @(posedge ACLK);
        ARESETn = 1'b1;
    end

    // Master 0 signals
    wire M0_AWVALID, M0_AWREADY;
    wire [ID_WIDTH-1:0] M0_AWID;
    wire [ADDR_WIDTH-1:0] M0_AWADDR;
    wire [7:0] M0_AWLEN;
    wire M0_WVALID, M0_WREADY;
    wire [DATA_WIDTH-1:0] M0_WDATA;
    wire M0_WLAST;
    wire M0_BVALID, M0_BREADY;
    wire [ID_WIDTH-1:0] M0_BID;
    wire [1:0] M0_BRESP;
    wire M0_ARVALID, M0_ARREADY;
    wire [ID_WIDTH-1:0] M0_ARID;
    wire [ADDR_WIDTH-1:0] M0_ARADDR;
    wire [7:0] M0_ARLEN;
    wire M0_RVALID, M0_RREADY;
    wire [ID_WIDTH-1:0] M0_RID;
    wire [DATA_WIDTH-1:0] M0_RDATA;
    wire M0_RLAST;
    wire [1:0] M0_RRESP;
    wire m0_err_bresp, m0_err_rresp, m0_err_bid, m0_err_rid, m0_err_rlast;

    // Master 1 signals
    wire M1_AWVALID, M1_AWREADY;
    wire [ID_WIDTH-1:0] M1_AWID;
    wire [ADDR_WIDTH-1:0] M1_AWADDR;
    wire [7:0] M1_AWLEN;
    wire M1_WVALID, M1_WREADY;
    wire [DATA_WIDTH-1:0] M1_WDATA;
    wire M1_WLAST;
    wire M1_BVALID, M1_BREADY;
    wire [ID_WIDTH-1:0] M1_BID;
    wire [1:0] M1_BRESP;
    wire M1_ARVALID, M1_ARREADY;
    wire [ID_WIDTH-1:0] M1_ARID;
    wire [ADDR_WIDTH-1:0] M1_ARADDR;
    wire [7:0] M1_ARLEN;
    wire M1_RVALID, M1_RREADY;
    wire [ID_WIDTH-1:0] M1_RID;
    wire [DATA_WIDTH-1:0] M1_RDATA;
    wire M1_RLAST;
    wire [1:0] M1_RRESP;
    wire m1_err_bresp, m1_err_rresp, m1_err_bid, m1_err_rid, m1_err_rlast;

    // Slave-side signals. ID_WIDTH+1 includes arbiter route bit.
    wire S_AWVALID, S_AWREADY;
    wire [ID_WIDTH:0] S_AWID;
    wire [ADDR_WIDTH-1:0] S_AWADDR;
    wire [7:0] S_AWLEN;
    wire S_WVALID, S_WREADY;
    wire [DATA_WIDTH-1:0] S_WDATA;
    wire S_WLAST;
    wire S_BVALID, S_BREADY;
    wire [ID_WIDTH:0] S_BID;
    wire [1:0] S_BRESP;
    wire S_ARVALID, S_ARREADY;
    wire [ID_WIDTH:0] S_ARID;
    wire [ADDR_WIDTH-1:0] S_ARADDR;
    wire [7:0] S_ARLEN;
    wire S_RVALID, S_RREADY;
    wire [ID_WIDTH:0] S_RID;
    wire [DATA_WIDTH-1:0] S_RDATA;
    wire S_RLAST;
    wire [1:0] S_RRESP;

    wire arb_err_wlast, arb_err_rlast, arb_err_bid_order, arb_err_rid_order;
    wire slave_err_wlast;

    simple_axi_master #(
        .ID_WIDTH(ID_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .NUM_TRANSACTIONS(NUM_TRANSACTIONS),
        .DEBUG(DEBUG),
        .ISSUE_GAP(DEMO_GAP_0),
        .START_DELAY(0)
    ) master0 (
        .ACLK(ACLK), .ARESETn(ARESETn),
        .AWVALID(M0_AWVALID), .AWREADY(M0_AWREADY), .AWID(M0_AWID), .AWADDR(M0_AWADDR), .AWLEN(M0_AWLEN),
        .WVALID(M0_WVALID), .WREADY(M0_WREADY), .WDATA(M0_WDATA), .WLAST(M0_WLAST),
        .BVALID(M0_BVALID), .BREADY(M0_BREADY), .BID(M0_BID), .BRESP(M0_BRESP),
        .ARVALID(M0_ARVALID), .ARREADY(M0_ARREADY), .ARID(M0_ARID), .ARADDR(M0_ARADDR), .ARLEN(M0_ARLEN),
        .RVALID(M0_RVALID), .RREADY(M0_RREADY), .RID(M0_RID), .RDATA(M0_RDATA), .RLAST(M0_RLAST), .RRESP(M0_RRESP),
        .MASTER_ID(1'b0),
        .protocol_error_bresp(m0_err_bresp), .protocol_error_rresp(m0_err_rresp),
        .protocol_error_bid(m0_err_bid), .protocol_error_rid(m0_err_rid), .protocol_error_rlast(m0_err_rlast)
    );

    realistic_axi_master #(
        .ID_WIDTH(ID_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .NUM_TRANSACTIONS(NUM_TRANSACTIONS),
        .DEBUG(DEBUG),
        .MASTER_ID(1),
        .SEED(M1_SEED),
        .START_DELAY(DEMO_OFFSET),
        .GAP_MIN(M1_GAP_MIN),
        .GAP_MAX(M1_GAP_MAX),
        .WVALID_PROB(M1_WVALID_PROB),
        .BREADY_PROB(M1_BREADY_PROB),
        .RREADY_PROB(M1_RREADY_PROB),
        .WCMD_DEPTH(M1_CMD_DEPTH),
        .RCMD_DEPTH(M1_CMD_DEPTH),
        .WDATA_FILL_LATENCY(M1_WDATA_FILL_LATENCY)
    ) master1 (
        .ACLK(ACLK), .ARESETn(ARESETn),
        .AWVALID(M1_AWVALID), .AWREADY(M1_AWREADY), .AWID(M1_AWID), .AWADDR(M1_AWADDR), .AWLEN(M1_AWLEN),
        .WVALID(M1_WVALID), .WREADY(M1_WREADY), .WDATA(M1_WDATA), .WLAST(M1_WLAST),
        .BVALID(M1_BVALID), .BREADY(M1_BREADY), .BID(M1_BID), .BRESP(M1_BRESP),
        .ARVALID(M1_ARVALID), .ARREADY(M1_ARREADY), .ARID(M1_ARID), .ARADDR(M1_ARADDR), .ARLEN(M1_ARLEN),
        .RVALID(M1_RVALID), .RREADY(M1_RREADY), .RID(M1_RID), .RDATA(M1_RDATA), .RLAST(M1_RLAST), .RRESP(M1_RRESP),
        .protocol_error_bresp(m1_err_bresp), .protocol_error_rresp(m1_err_rresp),
        .protocol_error_bid(m1_err_bid), .protocol_error_rid(m1_err_rid), .protocol_error_rlast(m1_err_rlast)
    );

    axi4_simple_2to1_bus_arbiter #(
        .ID_WIDTH(ID_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .MAX_OUTSTANDING_TRANSACTIONS(MAX_OUTSTANDING_TRANSACTIONS),
        .DEBUG(DEBUG)
    ) arbiter (
        .ACLK(ACLK), .ARESETn(ARESETn),
        .M0_AWVALID(M0_AWVALID), .M0_AWREADY(M0_AWREADY), .M0_AWID(M0_AWID), .M0_AWADDR(M0_AWADDR), .M0_AWLEN(M0_AWLEN),
        .M0_WVALID(M0_WVALID), .M0_WREADY(M0_WREADY), .M0_WDATA(M0_WDATA), .M0_WLAST(M0_WLAST),
        .M0_BVALID(M0_BVALID), .M0_BREADY(M0_BREADY), .M0_BID(M0_BID), .M0_BRESP(M0_BRESP),
        .M0_ARVALID(M0_ARVALID), .M0_ARREADY(M0_ARREADY), .M0_ARID(M0_ARID), .M0_ARADDR(M0_ARADDR), .M0_ARLEN(M0_ARLEN),
        .M0_RVALID(M0_RVALID), .M0_RREADY(M0_RREADY), .M0_RID(M0_RID), .M0_RDATA(M0_RDATA), .M0_RLAST(M0_RLAST), .M0_RRESP(M0_RRESP),
        .M1_AWVALID(M1_AWVALID), .M1_AWREADY(M1_AWREADY), .M1_AWID(M1_AWID), .M1_AWADDR(M1_AWADDR), .M1_AWLEN(M1_AWLEN),
        .M1_WVALID(M1_WVALID), .M1_WREADY(M1_WREADY), .M1_WDATA(M1_WDATA), .M1_WLAST(M1_WLAST),
        .M1_BVALID(M1_BVALID), .M1_BREADY(M1_BREADY), .M1_BID(M1_BID), .M1_BRESP(M1_BRESP),
        .M1_ARVALID(M1_ARVALID), .M1_ARREADY(M1_ARREADY), .M1_ARID(M1_ARID), .M1_ARADDR(M1_ARADDR), .M1_ARLEN(M1_ARLEN),
        .M1_RVALID(M1_RVALID), .M1_RREADY(M1_RREADY), .M1_RID(M1_RID), .M1_RDATA(M1_RDATA), .M1_RLAST(M1_RLAST), .M1_RRESP(M1_RRESP),
        .S_AWVALID(S_AWVALID), .S_AWREADY(S_AWREADY), .S_AWID(S_AWID), .S_AWADDR(S_AWADDR), .S_AWLEN(S_AWLEN),
        .S_WVALID(S_WVALID), .S_WREADY(S_WREADY), .S_WDATA(S_WDATA), .S_WLAST(S_WLAST),
        .S_BVALID(S_BVALID), .S_BREADY(S_BREADY), .S_BID(S_BID), .S_BRESP(S_BRESP),
        .S_ARVALID(S_ARVALID), .S_ARREADY(S_ARREADY), .S_ARID(S_ARID), .S_ARADDR(S_ARADDR), .S_ARLEN(S_ARLEN),
        .S_RVALID(S_RVALID), .S_RREADY(S_RREADY), .S_RID(S_RID), .S_RDATA(S_RDATA), .S_RLAST(S_RLAST), .S_RRESP(S_RRESP),
        .protocol_error_wlast(arb_err_wlast),
        .protocol_error_rlast(arb_err_rlast),
        .protocol_error_bid_order(arb_err_bid_order),
        .protocol_error_rid_order(arb_err_rid_order)
    );

    simple_axi_slave #(
        .ID_WIDTH(ID_WIDTH+1),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .MAX_OUTSTANDING_TRANSACTIONS(MAX_OUTSTANDING_TRANSACTIONS),
        .DEBUG(DEBUG),
        .RESP_LATENCY(SLAVE_RESP_LATENCY)
    ) slave (
        .ACLK(ACLK), .ARESETn(ARESETn),
        .AWVALID(S_AWVALID), .AWREADY(S_AWREADY), .AWID(S_AWID), .AWADDR(S_AWADDR), .AWLEN(S_AWLEN),
        .WVALID(S_WVALID), .WREADY(S_WREADY), .WDATA(S_WDATA), .WLAST(S_WLAST),
        .BVALID(S_BVALID), .BREADY(S_BREADY), .BID(S_BID), .BRESP(S_BRESP),
        .ARVALID(S_ARVALID), .ARREADY(S_ARREADY), .ARID(S_ARID), .ARADDR(S_ARADDR), .ARLEN(S_ARLEN),
        .RVALID(S_RVALID), .RREADY(S_RREADY), .RID(S_RID), .RDATA(S_RDATA), .RLAST(S_RLAST), .RRESP(S_RRESP),
        .protocol_error_wlast(slave_err_wlast)
    );

    // ------------------------------------------------------------------
    // Scoreboard
    // ------------------------------------------------------------------
    integer m0_b_count, m1_b_count;
    integer m0_r_count, m1_r_count;
    integer m0_rlast_count, m1_rlast_count;
    integer s_aw_count, s_w_count, s_wlast_count, s_ar_count;
    integer s_b_count, s_r_count, s_rlast_count;

    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            m0_b_count <= 0; 
            m1_b_count <= 0;

            m0_r_count <= 0; 
            m1_r_count <= 0;

            m0_rlast_count <= 0; 
            m1_rlast_count <= 0;

            s_aw_count <= 0; 
            s_w_count <= 0; 
            s_wlast_count <= 0; 
            s_ar_count <= 0;

            s_b_count <= 0; 
            s_r_count <= 0; 
            s_rlast_count <= 0;

        end else begin
            if (M0_BVALID && M0_BREADY) m0_b_count <= m0_b_count + 1;
            if (M1_BVALID && M1_BREADY) m1_b_count <= m1_b_count + 1;

            if (M0_RVALID && M0_RREADY) begin
                m0_r_count <= m0_r_count + 1;
                if (M0_RLAST) m0_rlast_count <= m0_rlast_count + 1;
            end
            if (M1_RVALID && M1_RREADY) begin
                m1_r_count <= m1_r_count + 1;
                if (M1_RLAST) m1_rlast_count <= m1_rlast_count + 1;
            end

            if (S_AWVALID && S_AWREADY) s_aw_count <= s_aw_count + 1;
            if (S_WVALID  && S_WREADY)  begin
                s_w_count <= s_w_count + 1;
                if (S_WLAST) s_wlast_count <= s_wlast_count + 1;
            end
            if (S_ARVALID && S_ARREADY) s_ar_count <= s_ar_count + 1;
            if (S_BVALID && S_BREADY) s_b_count <= s_b_count + 1;
            if (S_RVALID && S_RREADY) begin
                s_r_count <= s_r_count + 1;
                if (S_RLAST) s_rlast_count <= s_rlast_count + 1;
            end
        end
    end

    // ------------------------------------------------------------------
    // Immediate protocol-failure monitor
    // ------------------------------------------------------------------
    always_ff @(posedge ACLK) begin
        if (ARESETn) begin
            // Master 0
            if (m0_err_bresp) $fatal(1, "[%0t] M0: BRESP != OKAY", $time);
            if (m0_err_bid)   $fatal(1, "[%0t] M0: BID mismatch (B out-of-order or misrouted)", $time);
            if (m0_err_rresp) $fatal(1, "[%0t] M0: RRESP != OKAY", $time);
            if (m0_err_rid)   $fatal(1, "[%0t] M0: RID mismatch (R out-of-order or misrouted)", $time);
            if (m0_err_rlast) $fatal(1, "[%0t] M0: RLAST misplaced", $time);
            // Master 1
            if (m1_err_bresp) $fatal(1, "[%0t] M1: BRESP != OKAY", $time);
            if (m1_err_bid)   $fatal(1, "[%0t] M1: BID mismatch (B out-of-order or misrouted)", $time);
            if (m1_err_rresp) $fatal(1, "[%0t] M1: RRESP != OKAY", $time);
            if (m1_err_rid)   $fatal(1, "[%0t] M1: RID mismatch (R out-of-order or misrouted)", $time);
            if (m1_err_rlast) $fatal(1, "[%0t] M1: RLAST misplaced", $time);
            // Arbiter
            if (arb_err_wlast)     $fatal(1, "[%0t] ARB: WLAST vs accepted AWLEN mismatch", $time);
            if (arb_err_rlast)     $fatal(1, "[%0t] ARB: RLAST vs accepted ARLEN mismatch", $time);
            if (arb_err_bid_order) $fatal(1, "[%0t] ARB: BID order/route mismatch (slave returned B out-of-order)", $time);
            if (arb_err_rid_order) $fatal(1, "[%0t] ARB: RID order/route mismatch (slave returned R out-of-order)", $time);
            // Slave
            if (slave_err_wlast)   $fatal(1, "[%0t] SLV: WLAST vs accepted AWLEN mismatch", $time);
        end
    end

    // ------------------------------------------------------------------
    // Lightweight ready/valid stability checks
    // ------------------------------------------------------------------
    reg [ID_WIDTH-1:0] m0_awid_prev, m1_awid_prev, m0_arid_prev, m1_arid_prev;
    reg [ADDR_WIDTH-1:0] m0_awaddr_prev, m1_awaddr_prev, m0_araddr_prev, m1_araddr_prev;
    reg [7:0] m0_awlen_prev, m1_awlen_prev, m0_arlen_prev, m1_arlen_prev;
    reg [DATA_WIDTH-1:0] m0_wdata_prev, m1_wdata_prev, s_rdata_prev;
    reg m0_wlast_prev, m1_wlast_prev, s_rlast_prev;
    reg m0_aw_stall_prev, m1_aw_stall_prev;
    reg m0_ar_stall_prev, m1_ar_stall_prev;
    reg m0_w_stall_prev,  m1_w_stall_prev;
    reg s_r_stall_prev;

    always_ff @(posedge ACLK or negedge ARESETn) begin
        if (!ARESETn) begin
            m0_aw_stall_prev <= 1'b0; 
            m1_aw_stall_prev <= 1'b0;

            m0_ar_stall_prev <= 1'b0; 
            m1_ar_stall_prev <= 1'b0;

            m0_w_stall_prev  <= 1'b0; 
            m1_w_stall_prev  <= 1'b0;
            
            s_r_stall_prev   <= 1'b0;
        end else begin
            if (m0_aw_stall_prev && (!M0_AWVALID || M0_AWID !== m0_awid_prev || M0_AWADDR !== m0_awaddr_prev || M0_AWLEN !== m0_awlen_prev)) $fatal(1, "M0 AW changed while stalled");
            if (m1_aw_stall_prev && (!M1_AWVALID || M1_AWID !== m1_awid_prev || M1_AWADDR !== m1_awaddr_prev || M1_AWLEN !== m1_awlen_prev)) $fatal(1, "M1 AW changed while stalled");
            if (m0_ar_stall_prev && (!M0_ARVALID || M0_ARID !== m0_arid_prev || M0_ARADDR !== m0_araddr_prev || M0_ARLEN !== m0_arlen_prev)) $fatal(1, "M0 AR changed while stalled");
            if (m1_ar_stall_prev && (!M1_ARVALID || M1_ARID !== m1_arid_prev || M1_ARADDR !== m1_araddr_prev || M1_ARLEN !== m1_arlen_prev)) $fatal(1, "M1 AR changed while stalled");
            if (m0_w_stall_prev  && (!M0_WVALID || M0_WDATA !== m0_wdata_prev || M0_WLAST !== m0_wlast_prev)) $fatal(1, "M0 W changed while stalled");
            if (m1_w_stall_prev  && (!M1_WVALID || M1_WDATA !== m1_wdata_prev || M1_WLAST !== m1_wlast_prev)) $fatal(1, "M1 W changed while stalled");
            if (s_r_stall_prev   && (!S_RVALID || S_RDATA !== s_rdata_prev || S_RLAST !== s_rlast_prev)) $fatal(1, "S R changed while stalled");

            m0_awid_prev <= M0_AWID; 
            m0_awaddr_prev <= M0_AWADDR; 
            m0_awlen_prev <= M0_AWLEN;

            m1_awid_prev <= M1_AWID; 
            m1_awaddr_prev <= M1_AWADDR; 
            m1_awlen_prev <= M1_AWLEN;

            m0_arid_prev <= M0_ARID; 
            m0_araddr_prev <= M0_ARADDR; 
            m0_arlen_prev <= M0_ARLEN;

            m1_arid_prev <= M1_ARID; 
            m1_araddr_prev <= M1_ARADDR; 
            m1_arlen_prev <= M1_ARLEN;

            m0_wdata_prev <= M0_WDATA; 
            m0_wlast_prev <= M0_WLAST;

            m1_wdata_prev <= M1_WDATA; 
            m1_wlast_prev <= M1_WLAST;

            s_rdata_prev <= S_RDATA; 
            s_rlast_prev <= S_RLAST;

            m0_aw_stall_prev <= M0_AWVALID && !M0_AWREADY;
            m1_aw_stall_prev <= M1_AWVALID && !M1_AWREADY;
            m0_ar_stall_prev <= M0_ARVALID && !M0_ARREADY;
            m1_ar_stall_prev <= M1_ARVALID && !M1_ARREADY;
            m0_w_stall_prev  <= M0_WVALID  && !M0_WREADY;
            m1_w_stall_prev  <= M1_WVALID  && !M1_WREADY;
            s_r_stall_prev   <= S_RVALID   && !S_RREADY;
        end
    end

    // ------------------------------------------------------------------
    // Completion and watchdog
    // ------------------------------------------------------------------
    initial begin
        $timeformat(-9, 1, " ns", 10); // Print all %t timestamps as readable, right-aligned nanoseconds.
        $dumpfile("wave_rr_axi4_id_outstanding.vcd");
        $dumpvars(0, tb_axi_bus_system);
        $display("==================================================================");
        $display(" AXI4 2x1 bus debug trace");
        $display(" Line format:  [time] SRC CH dir txn=M<route>.<W|R><id>  <fields>");
        $display("   SRC = M0|M1 (master), ARB (arbiter), SLV (slave)");
        $display("   CH  = AW/W/B (write path),  AR/R (read path)");
        $display("   dir = >> request (master->slave),  << response (slave->master)");
        $display("   txn = M<route>.<W|R><id> : route = originating master,");
        $display("         W/R = write or read path, id = local AXI ID. Same key on");
        $display("         every hop; write and read keys are distinct, so e.g.");
        $display("         grep 'txn=M0.W2' (write) vs grep 'txn=M0.R2' (read).");
        $display("==================================================================");
        $display("---- Starting AXI4 ID + multiple outstanding scenario ----");
    end

    initial begin
        wait (ARESETn);
        wait (m0_b_count == NUM_TRANSACTIONS &&
              m1_b_count == NUM_TRANSACTIONS &&
              m0_r_count == NUM_TRANSACTIONS * BURST_BEATS &&
              m1_r_count == NUM_TRANSACTIONS * BURST_BEATS &&
              m0_rlast_count == NUM_TRANSACTIONS &&
              m1_rlast_count == NUM_TRANSACTIONS &&
              s_aw_count == 2 * NUM_TRANSACTIONS &&
              s_ar_count == 2 * NUM_TRANSACTIONS &&
              s_w_count  == 2 * NUM_TRANSACTIONS * BURST_BEATS &&
              s_wlast_count == 2 * NUM_TRANSACTIONS &&
              s_b_count == 2 * NUM_TRANSACTIONS &&
              s_r_count == 2 * NUM_TRANSACTIONS * BURST_BEATS &&
              s_rlast_count == 2 * NUM_TRANSACTIONS);
        repeat (10) @(posedge ACLK);

        $display("---- PASS: AXI4 ID + multiple outstanding scenario completed ----");
        $display("M0: B=%0d, R=%0d, RLAST=%0d", m0_b_count, m0_r_count, m0_rlast_count);
        $display("M1: B=%0d, R=%0d, RLAST=%0d", m1_b_count, m1_r_count, m1_rlast_count);
        $display("Slave side: AW=%0d, W=%0d, WLAST=%0d, B=%0d, AR=%0d, R=%0d, RLAST=%0d",
                 s_aw_count, s_w_count, s_wlast_count, s_b_count, s_ar_count, s_r_count, s_rlast_count);
        $finish;
    end

    // Watchdog scales with the workload: ~20 ns/clk * (writes + reads beats),
    // plus the demo pacing (per-master inter-transaction gaps and master1 offset),
    // plus reset/drain margin, so bumping NUM_TRANSACTIONS or DEMO_GAP does not
    // cause false timeouts.
    //
    // Master 1 is now the realistic master with random pacing, so add its
    // worst-case overhead so legitimate random idling is not mistaken for a
    // hang:
    //   - M1_GAP_MAX cycles of idle before each of its AW and AR issues (x2).
    //   - Random W/R backpressure stretches each beat; bound it generously at
    //     ~16 extra cycles per beat (valid while the *_PROB knobs stay >~25%).
    localparam int WATCHDOG_NS = 20 * (2 * NUM_TRANSACTIONS * BURST_BEATS * 4
                                       + DEMO_GAP_0 * NUM_TRANSACTIONS * 2
                                       + SLAVE_RESP_LATENCY * NUM_TRANSACTIONS * 4
                                       + M1_GAP_MAX * NUM_TRANSACTIONS * 2
                                       + NUM_TRANSACTIONS * BURST_BEATS * 16
                                       + M1_WDATA_FILL_LATENCY * NUM_TRANSACTIONS
                                       + DEMO_OFFSET) + 1000;

    initial begin
        #(WATCHDOG_NS);
        $display("---- TIMEOUT: counters (got/expected), mismatched channel is the stalled one ----");
        $display("M0: B=%0d/%0d R=%0d/%0d RLAST=%0d/%0d",
                 m0_b_count, NUM_TRANSACTIONS, m0_r_count, NUM_TRANSACTIONS*BURST_BEATS,
                 m0_rlast_count, NUM_TRANSACTIONS);
        $display("M1: B=%0d/%0d R=%0d/%0d RLAST=%0d/%0d",
                 m1_b_count, NUM_TRANSACTIONS, m1_r_count, NUM_TRANSACTIONS*BURST_BEATS,
                 m1_rlast_count, NUM_TRANSACTIONS);
        $display("S : AW=%0d/%0d W=%0d/%0d WLAST=%0d/%0d B=%0d/%0d AR=%0d/%0d R=%0d/%0d RLAST=%0d/%0d",
                 s_aw_count,    2*NUM_TRANSACTIONS,
                 s_w_count,     2*NUM_TRANSACTIONS*BURST_BEATS,
                 s_wlast_count, 2*NUM_TRANSACTIONS,
                 s_b_count,     2*NUM_TRANSACTIONS,
                 s_ar_count,    2*NUM_TRANSACTIONS,
                 s_r_count,     2*NUM_TRANSACTIONS*BURST_BEATS,
                 s_rlast_count, 2*NUM_TRANSACTIONS);
        $fatal(1, "Timeout at %0t: AXI4 ID + multiple outstanding system did not complete", $time);
    end

endmodule

// ======================================================================
//  realistic_axi_master.sv
//
//  PURPOSE
//    A "more realistic" behavioral AXI4 master for simulation. Same role as
//    simple_axi_master (stimulus generator + in-order self-checker), but built
//    like a real DMA/CPU front-end: a REQUEST GENERATOR adds transactions to a
//    write-command queue and a read-command queue OVER TIME, and the channel
//    engines drain those queues. Requests are not all known up front.
//
//  ARCHITECTURE  (write path; read path is analogous)
//
//      request gen --push {id,addr,len} over time (bounded random gap)--+
//                                                                       v
//                                                       +-----------------------+
//                                                       |  write-command queue  |  depth WCMD_DEPTH
//                                                       +-----------------------+
//                                  aw_head |              dprod_head |             b_head |
//                                          v                        v                    v
//                                     [AW engine]          [data producer]        [B checker]
//                                  issue AW when a cmd     fills write-data FIFO   pop in order,
//                                  is waiting (greedy)     ahead of time           check BID/BRESP
//                                                               |
//                                                               v  [write-data FIFO, WCMD_DEPTH*BURST_BEATS beats]
//                                                          [W engine] drains FIFO w/ random WVALID bubbles
//  
//
//    Write data is BUFFERED and LAGS the request, like a real data source: once
//    a command exists, the data producer waits a fixed fill latency
//    (WDATA_FILL_LATENCY), then pushes its {data,last} beats into the write-data
//    FIFO with random per-beat pacing. The W engine drains that FIFO onto the W
//    channel - so if the source is slower than the channel, W starves and
//    stalls; if faster, the FIFO fills and back-pressures the producer.
//
//    - The generator pushes a new descriptor every so often (GAP_MIN..GAP_MAX
//      idle cycles between pushes) until NUM_TRANSACTIONS are generated, and
//      STALLS when the queue is full (depth = max outstanding) - so the queue
//      fills and drains under backpressure, like a busy master.
//    - One descriptor queue per direction with several read pointers (AW/W/B
//      for writes, AR/R for reads); an entry is freed only after its response
//      (B / final R beat) completes.
//    - All randomness is from per-engine LFSRs seeded from SEED, so every run
//      is REPRODUCIBLE (no $urandom). Change SEED for a new pattern.
//
//  WHAT IS THE SAME (on purpose)
//    - Transaction CONTENT and ORDER match simple_axi_master: NUM_TRANSACTIONS
//      write and read bursts of BURST_BEATS beats, in ascending order, same
//      address scheme. The system (arbiter, slave, checkers) is in-order, so
//      only WHEN requests appear and how they are paced is randomized - never
//      the order. So the testbench's transaction counts are unchanged.
//    - In-order B/R self-checkers verify BID/RID, BRESP/RRESP == OKAY, and
//      RLAST placement, driving the same protocol_error_* outputs.
//
//  PROTOCOL RULES HONORED (the testbench $fatals if these are broken)
//    - VALID held stable with constant payload until READY (AW/AR/W).
//    - Write data presented in AW-issue order with correct WLAST.
//
//  EXTENSIBILITY
//    MASTER_ID is a compile-time PARAMETER; the address map / write-data tag
//    are derived from it. Add masters by instantiating more copies with new
//    MASTER_ID values - each gets its own address region and data tag.
//    Making content random later is a one-line change IN THE GENERATOR, since
//    every consumer reads the stored descriptor.
//
//  PARAMETERS
//    ID_WIDTH, ADDR_WIDTH, DATA_WIDTH, NUM_TRANSACTIONS, DEBUG
//                  - same meaning as simple_axi_master.
//    MASTER_ID     - this master's index (0,1,2,...). Selects address region,
//                    tags write data, printed in traces.
//    WR_BASE_ADDR / RD_BASE_ADDR / MASTER_ADDR_STRIDE / TXN_ADDR_STRIDE
//                  - address map: region = BASE + MASTER_ID*MASTER_ADDR_STRIDE,
//                    addr(i) = region + i*TXN_ADDR_STRIDE.
//    SEED          - per-master LFSR seed; pick distinct values per master.
//    START_DELAY   - idle cycles before the generators begin.
//    GAP_MIN/GAP_MAX - inclusive bound on the random idle gap (cycles) between
//                    successive request pushes by a generator. GAP_MAX caps the
//                    worst case so the testbench watchdog stays valid.
//    WVALID_PROB   - per-cycle % chance (0..100) the W engine presents a beat.
//    BREADY_PROB   - per-cycle % chance (0..100) BREADY is asserted.
//    RREADY_PROB   - per-cycle % chance (0..100) RREADY is asserted.
//    WCMD_DEPTH / RCMD_DEPTH - write/read command-queue depth (= max
//                    outstanding transactions of that kind).
//                    (The write-data FIFO is sized automatically to
//                    WCMD_DEPTH*BURST_BEATS - full coverage - so it never
//                    bottlenecks; the command queue is the sole write limiter.)
//    WDATA_FILL_LATENCY - fixed cycles after a write request is created before
//                    its first data beat is produced (models fetch/produce
//                    latency). Per-beat pacing within a burst is random
//                    (bounded by GAP_MIN/GAP_MAX, same as the generator).
// ======================================================================
module realistic_axi_master #(
    parameter int ID_WIDTH = 4,
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 128,
    parameter int NUM_TRANSACTIONS = 2,
    parameter bit DEBUG = 1'b0,
    // Master identity + address map (extensible to N masters).
    parameter int MASTER_ID = 1,
    parameter logic [ADDR_WIDTH-1:0] WR_BASE_ADDR       = 'h0000_4000,
    parameter logic [ADDR_WIDTH-1:0] RD_BASE_ADDR       = 'h0000_2000,
    parameter logic [ADDR_WIDTH-1:0] MASTER_ADDR_STRIDE = 'h0001_0000,
    parameter logic [ADDR_WIDTH-1:0] TXN_ADDR_STRIDE    = 'h0000_0100,
    // Randomized pacing knobs.
    parameter int SEED        = 1,
    parameter int START_DELAY = 0,
    parameter int GAP_MIN     = 0,
    parameter int GAP_MAX     = 12,
    parameter int WVALID_PROB = 70,
    parameter int BREADY_PROB = 80,
    parameter int RREADY_PROB = 75,
    parameter int WCMD_DEPTH  = 8,
    parameter int WDATA_FILL_LATENCY = 6, // fixed cycles after a write request is
                                          // created before its first data beat is
                                          // produced (models fetch/produce latency)
    parameter int RCMD_DEPTH  = 8
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

    output reg                          protocol_error_bresp,
    output reg                          protocol_error_rresp,
    output reg                          protocol_error_bid,
    output reg                          protocol_error_rid,
    output reg                          protocol_error_rlast
);

    localparam int BURST_BEATS = 4;
    localparam int BURST_LEN   = BURST_BEATS - 1;
    // Counter that must hold 0..NUM_TRANSACTIONS (one extra bit beyond index).
    localparam int GEN_CNT_W = $clog2(NUM_TRANSACTIONS + 1);

    // Write-data FIFO depth (beats), sized for FULL COVERAGE: data for every
    // outstanding write command (WCMD_DEPTH of them, BURST_BEATS beats each) can
    // be buffered at once, so the data FIFO never bottlenecks - the command
    // queue is the sole outstanding-write limiter. Auto-scales with WCMD_DEPTH.
    localparam int WDATA_DEPTH = WCMD_DEPTH * BURST_BEATS;

    // Per-master address regions (elaboration constants).
    localparam logic [ADDR_WIDTH-1:0] WR_REGION_BASE = WR_BASE_ADDR + (MASTER_ID * MASTER_ADDR_STRIDE);
    localparam logic [ADDR_WIDTH-1:0] RD_REGION_BASE = RD_BASE_ADDR + (MASTER_ID * MASTER_ADDR_STRIDE);

    // Per-engine LFSR seeds. XOR-mangled from SEED so they do not run in
    // lockstep, OR 1 so seed 0 never yields the all-zeros lock-up state.
    localparam logic [15:0] SEED_WGEN = (SEED[15:0] ^ 16'hBEEF) | 16'h0001;
    localparam logic [15:0] SEED_RGEN = (SEED[15:0] ^ 16'h1CE9) | 16'h0001;
    localparam logic [15:0] SEED_W    = (SEED[15:0] ^ 16'h7A5C) | 16'h0001;
    localparam logic [15:0] SEED_B    = (SEED[15:0] ^ 16'h3D21) | 16'h0001;
    localparam logic [15:0] SEED_R    = (SEED[15:0] ^ 16'h9F44) | 16'h0001;
    localparam logic [15:0] SEED_DGAP = (SEED[15:0] ^ 16'h58E3) | 16'h0001;

    initial begin
        if (GAP_MAX < GAP_MIN) $fatal(1, "GAP_MAX must be >= GAP_MIN");
        if (WCMD_DEPTH < 1)    $fatal(1, "WCMD_DEPTH must be >= 1");
        if (RCMD_DEPTH < 1)    $fatal(1, "RCMD_DEPTH must be >= 1");
    end

    typedef struct packed {
        logic [ID_WIDTH-1:0]   id;
        logic [ADDR_WIDTH-1:0] addr;
        logic [7:0]            len;
    } desc_t; // a queued transaction descriptor

    function automatic logic [DATA_WIDTH-1:0] make_wdata(input int txn, input int beat);
        logic [DATA_WIDTH-1:0] tag;
        begin
            // Top byte tags the originating master; low bytes carry txn/beat so
            // any beat stays traceable. Works for any MASTER_ID (0..255).
            tag = MASTER_ID;
            make_wdata = (tag << (DATA_WIDTH-8)) | (txn << 8) | beat;
        end
    endfunction

    function automatic string resp_str(input logic [1:0] resp);
        case (resp)
            2'b00:   resp_str = "OKAY";
            2'b01:   resp_str = "EXOKAY";
            2'b10:   resp_str = "SLVERR";
            default: resp_str = "DECERR";
        endcase
    endfunction

    function automatic string tag_if(input bit cond, input string msg);
        tag_if = cond ? msg : "";
    endfunction

    // Maximal-length 16-bit Fibonacci LFSR: x^16 + x^14 + x^13 + x^11 + 1.
    function automatic logic [15:0] lfsr_next(input logic [15:0] s);
        logic fb;
        fb = s[15] ^ s[13] ^ s[12] ^ s[10];
        lfsr_next = {s[14:0], fb};
    endfunction

    // Bounded random idle gap, in cycles, derived from an LFSR sample.
    function automatic logic [15:0] rand_gap(input logic [15:0] s);
        rand_gap = GAP_MIN + (s % (GAP_MAX - GAP_MIN + 1));
    endfunction

    // ==================================================================
    // WRITE PATH: generator -> write-command queue -> AW / W / B engines.
    // Single always_ff so the shared pointers/counters have one driver.
    // ==================================================================
    localparam int WQ_PTR_W = (WCMD_DEPTH <= 1) ? 1 : $clog2(WCMD_DEPTH);

    desc_t wcmd [0:WCMD_DEPTH-1];     // write-command queue
    logic [WQ_PTR_W-1:0] wq_tail;     // generator push point
    logic [WQ_PTR_W-1:0] wq_aw_head;  // next descriptor to issue on AW
    logic [WQ_PTR_W-1:0] wq_dprod_head; // descriptor the data producer is expanding
    logic [WQ_PTR_W-1:0] wq_b_head;   // next expected B

    // Occupancy counters (generated-but-not-yet-X). occ gates queue fullness.
    logic [GEN_CNT_W:0] wq_occ;       // generated, B not yet seen  (== fullness)
    logic [GEN_CNT_W:0] wq_aw_pend;   // generated, AW not yet issued
    logic [GEN_CNT_W:0] wq_dprod_pend; // generated, write data not yet fully produced

    logic [GEN_CNT_W-1:0] wgen_count;   // total write descriptors generated so far
    logic [15:0]          wgen_gap;     // idle countdown between generator pushes
    logic [7:0]           wd_prod_beat; // beat being produced into the data FIFO
    logic [15:0]          dprod_lat;    // fixed fill-latency countdown before a burst's first beat
    logic [15:0]          dprod_gap;    // random per-beat pacing countdown within a burst
    logic [7:0]           w_send_beat;  // beat index of W beats accepted on the channel (debug)
    logic [ID_WIDTH-1:0]  w_cur_id;     // ID of the burst currently on the W channel (debug)
    logic [GEN_CNT_W-1:0] b_recv_count; // write responses accepted (debug/progress)
    logic [15:0]          lfsr_wgen, lfsr_w, lfsr_b, lfsr_dgap;

    wire w_gate = (lfsr_w % 100) < WVALID_PROB;
    wire b_gate = (lfsr_b % 100) < BREADY_PROB;

    // Write-data FIFO: a data producer fills it with {data,last} beats ahead of
    // time (off the generated commands); the W engine drains it with bubbles.
    localparam int WD_PTR_W = (WDATA_DEPTH <= 1) ? 1 : $clog2(WDATA_DEPTH);
    localparam int WD_CNT_W = $clog2(WDATA_DEPTH + 1);

    typedef struct packed {
        logic [DATA_WIDTH-1:0] data;
        logic [ID_WIDTH-1:0]   id;   // owning command's ID (for per-beat tracing)
        logic                  last;
    } wbeat_t;

    wbeat_t wdata [0:WDATA_DEPTH-1];
    logic [WD_PTR_W-1:0] wd_head, wd_tail;
    logic [WD_CNT_W-1:0] wd_count;        // beats buffered in the data FIFO
    wire wd_full  = (wd_count == WDATA_DEPTH);
    wire wd_empty = (wd_count == '0);

    function automatic logic [WQ_PTR_W-1:0] wq_inc(input logic [WQ_PTR_W-1:0] p);
        if (p == WCMD_DEPTH-1) wq_inc = '0;
        else wq_inc = p + {{(WQ_PTR_W-1){1'b0}}, 1'b1};
    endfunction

    function automatic logic [WD_PTR_W-1:0] wd_inc(input logic [WD_PTR_W-1:0] p);
        if (p == WDATA_DEPTH-1) wd_inc = '0;
        else wd_inc = p + {{(WD_PTR_W-1){1'b0}}, 1'b1};
    endfunction

    always_ff @(posedge ACLK or negedge ARESETn) begin
        logic wgen_push, aw_hs, b_hs;
        logic data_push, data_burst_produced, w_pop;
        if (!ARESETn) begin
            AWVALID <= 1'b0;
            AWID    <= '0;
            AWADDR  <= '0;
            AWLEN   <= '0;
            WVALID  <= 1'b0;
            WDATA   <= '0;
            WLAST   <= 1'b0;
            BREADY  <= 1'b0;
            wq_tail       <= '0;
            wq_aw_head    <= '0;
            wq_dprod_head <= '0;
            wq_b_head     <= '0;
            wq_occ        <= '0;
            wq_aw_pend    <= '0;
            wq_dprod_pend <= '0;
            wd_head       <= '0;
            wd_tail       <= '0;
            wd_count      <= '0;
            wgen_count    <= '0;
            wgen_gap      <= START_DELAY[15:0];
            wd_prod_beat  <= 8'd0;
            dprod_lat     <= WDATA_FILL_LATENCY[15:0]; // first burst pays the fill latency
            dprod_gap     <= 16'd0;
            w_send_beat   <= 8'd0;
            w_cur_id      <= '0;
            b_recv_count  <= '0;
            protocol_error_bresp <= 1'b0;
            protocol_error_bid   <= 1'b0;
            lfsr_wgen <= SEED_WGEN;
            lfsr_w    <= SEED_W;
            lfsr_b    <= SEED_B;
            lfsr_dgap <= SEED_DGAP;
        end else begin
            lfsr_wgen <= lfsr_next(lfsr_wgen);
            lfsr_w    <= lfsr_next(lfsr_w);
            lfsr_b    <= lfsr_next(lfsr_b);
            lfsr_dgap <= lfsr_next(lfsr_dgap);

            // ---- handshake / event flags (start-of-cycle state) ----
            wgen_push = (wgen_gap == 16'd0) && (wgen_count < NUM_TRANSACTIONS)
                        && (wq_occ < WCMD_DEPTH);
            aw_hs = AWVALID && AWREADY;
            b_hs  = BVALID && BREADY;
            // Data producer pushes one beat only after the fixed fill latency
            // (dprod_lat) and the random per-beat pacing gap (dprod_gap) have
            // elapsed, a command still needs data, and the FIFO has room.
            data_push = (wq_dprod_pend != '0) && (dprod_lat == 16'd0)
                        && (dprod_gap == 16'd0) && !wd_full;
            data_burst_produced = data_push && (wd_prod_beat == wcmd[wq_dprod_head].len);
            // W engine pops a buffered beat onto the channel (gated by w_gate).
            w_pop = (!(WVALID && !WREADY)) && !wd_empty && w_gate;

            // ---------------- request generator ----------------
            if (wgen_gap != 16'd0) begin
                wgen_gap <= wgen_gap - 16'd1;
            end else if (wgen_push) begin
                wcmd[wq_tail].id   <= wgen_count[ID_WIDTH-1:0];
                wcmd[wq_tail].addr <= WR_REGION_BASE + (wgen_count * TXN_ADDR_STRIDE);
                wcmd[wq_tail].len  <= BURST_LEN[7:0];
                wq_tail    <= wq_inc(wq_tail);
                wgen_count <= wgen_count + 1'b1;
                wgen_gap   <= rand_gap(lfsr_wgen); // bounded gap before the next push
            end

            // ---------------- AW engine (greedy issue) ----------------
            if (!AWVALID) begin
                if (wq_aw_pend != '0) begin
                    AWID    <= wcmd[wq_aw_head].id;
                    AWADDR  <= wcmd[wq_aw_head].addr;
                    AWLEN   <= wcmd[wq_aw_head].len;
                    AWVALID <= 1'b1;
                end
            end else if (AWREADY) begin
                AWVALID    <= 1'b0; // one-cycle gap; re-present next cycle if pending
                wq_aw_head <= wq_inc(wq_aw_head);
            end

            // ---------------- write-data producer ----------------
            // Models a data source that lags the request: once a command exists,
            // wait a FIXED fill latency before its first beat, then push beats
            // into the FIFO with RANDOM per-beat pacing. WDATA is built here, not
            // on the fly in the W engine.
            if ((wq_dprod_pend != '0) && !data_push) begin
                // counting toward the next beat (latency first, then per-beat gap)
                if (dprod_lat != 16'd0)      dprod_lat <= dprod_lat - 16'd1;
                else if (dprod_gap != 16'd0) dprod_gap <= dprod_gap - 16'd1;
            end
            if (data_push) begin
                wdata[wd_tail].data <= make_wdata(wcmd[wq_dprod_head].id, wd_prod_beat);
                wdata[wd_tail].id   <= wcmd[wq_dprod_head].id;
                wdata[wd_tail].last <= (wd_prod_beat == wcmd[wq_dprod_head].len);
                wd_tail <= wd_inc(wd_tail);
                if (wd_prod_beat == wcmd[wq_dprod_head].len) begin
                    wd_prod_beat  <= 8'd0;
                    wq_dprod_head <= wq_inc(wq_dprod_head); // move to next command
                    dprod_lat     <= WDATA_FILL_LATENCY[15:0]; // arm latency for next burst
                    dprod_gap     <= 16'd0;
                end else begin
                    wd_prod_beat <= wd_prod_beat + 8'd1;
                    dprod_gap    <= rand_gap(lfsr_dgap); // random pacing before next beat
                end
            end

            // ---------------- W engine (drain data FIFO w/ random bubbles) ----------------
            if (WVALID && !WREADY) begin
                // stall: hold WDATA/WLAST/WVALID stable (AXI rule)
            end else if (w_pop) begin
                WDATA    <= wdata[wd_head].data;
                WLAST    <= wdata[wd_head].last;
                w_cur_id <= wdata[wd_head].id; // held with WDATA for the per-beat trace
                WVALID   <= 1'b1;
                wd_head  <= wd_inc(wd_head);
            end else begin
                WVALID <= 1'b0; // idle or random hold-off (bubble)
            end

            // Track the beat index of accepted W beats (for per-beat tracing).
            if (WVALID && WREADY) begin
                w_send_beat <= WLAST ? 8'd0 : (w_send_beat + 8'd1);
            end

            // ---------------- B checker (random backpressure) ----------------
            BREADY <= b_gate;
            if (b_hs) begin
                if (BRESP != 2'b00) protocol_error_bresp <= 1'b1;
                if (wq_occ == '0 || BID !== wcmd[wq_b_head].id) begin
                    protocol_error_bid <= 1'b1;
                end
                wq_b_head    <= wq_inc(wq_b_head);
                b_recv_count <= b_recv_count + 1'b1;
            end

            // ---------------- occupancy counters ----------------
            unique case ({wgen_push, b_hs}) // wq_occ: +push, −B-handshake
                2'b10:   wq_occ <= wq_occ + 1'b1;
                2'b01:   wq_occ <= wq_occ - 1'b1;
                default: wq_occ <= wq_occ;  // 00 or 11 → unchanged
            endcase
            unique case ({wgen_push, aw_hs})
                2'b10:   wq_aw_pend <= wq_aw_pend + 1'b1;
                2'b01:   wq_aw_pend <= wq_aw_pend - 1'b1;
                default: wq_aw_pend <= wq_aw_pend;
            endcase
            unique case ({wgen_push, data_burst_produced}) 
                2'b10:   wq_dprod_pend <= wq_dprod_pend + 1'b1;
                2'b01:   wq_dprod_pend <= wq_dprod_pend - 1'b1;
                default: wq_dprod_pend <= wq_dprod_pend;
            endcase
            unique case ({data_push, w_pop}) // write-data FIFO occupancy
                2'b10:   wd_count <= wd_count + 1'b1;
                2'b01:   wd_count <= wd_count - 1'b1;
                default: wd_count <= wd_count;
            endcase
        end

        if (DEBUG && ARESETn && AWVALID && AWREADY) begin
            $display("[%t] %-3s AW >> txn=M%0d.W%0d addr=0x%08h len=%0d  (wq_occ=%0d/%0d)",
                $time, $sformatf("M%0d", MASTER_ID), MASTER_ID, AWID, AWADDR, AWLEN,
                wq_occ, WCMD_DEPTH);
        end
        if (DEBUG && ARESETn && WVALID && WREADY) begin
            $display("[%t] %-3s W  >> txn=M%0d.W%0d beat=%0d wdata=0x%h%s",
                $time, $sformatf("M%0d", MASTER_ID), MASTER_ID, w_cur_id, w_send_beat, WDATA,
                tag_if(WLAST, " (WLAST)"));
        end
        if (DEBUG && ARESETn && BVALID && BREADY) begin
            $display("[%t] %-3s B  << txn=M%0d.W%0d resp=%-6s (%0d/%0d)%s%s",
                $time, $sformatf("M%0d", MASTER_ID), MASTER_ID, BID, resp_str(BRESP),
                b_recv_count + 1, NUM_TRANSACTIONS,
                tag_if(BRESP != 2'b00, " <-- ERROR RESP"),
                tag_if(wq_occ == '0 || BID !== wcmd[wq_b_head].id, " <-- ID MISMATCH"));
        end
    end

    // ==================================================================
    // READ PATH: generator -> read-command queue -> AR / R engines.
    // ==================================================================
    localparam int RQ_PTR_W = (RCMD_DEPTH <= 1) ? 1 : $clog2(RCMD_DEPTH);

    desc_t rcmd [0:RCMD_DEPTH-1];
    logic [RQ_PTR_W-1:0] rq_tail;     // generator push point
    logic [RQ_PTR_W-1:0] rq_ar_head;  // next descriptor to issue on AR
    logic [RQ_PTR_W-1:0] rq_r_head;   // descriptor the R checker is verifying

    logic [GEN_CNT_W:0] rq_occ;     // generated, R burst not yet complete (fullness)
    logic [GEN_CNT_W:0] rq_ar_pend; // generated, AR not yet issued

    logic [GEN_CNT_W-1:0] rgen_count;
    logic [15:0]          rgen_gap;
    logic [7:0]           r_beat_idx; // expected beat within current read burst
    logic [GEN_CNT_W-1:0] r_recv_count;
    logic [15:0]          lfsr_rgen, lfsr_r;

    wire r_gate = (lfsr_r % 100) < RREADY_PROB;

    function automatic logic [RQ_PTR_W-1:0] rq_inc(input logic [RQ_PTR_W-1:0] p);
        if (p == RCMD_DEPTH-1) rq_inc = '0;
        else rq_inc = p + {{(RQ_PTR_W-1){1'b0}}, 1'b1};
    endfunction

    always_ff @(posedge ACLK or negedge ARESETn) begin
        logic rgen_push, ar_hs, r_hs, r_burst_done;
        if (!ARESETn) begin
            ARVALID <= 1'b0;
            ARID    <= '0;
            ARADDR  <= '0;
            ARLEN   <= '0;
            RREADY  <= 1'b0;
            rq_tail    <= '0;
            rq_ar_head <= '0;
            rq_r_head  <= '0;
            rq_occ     <= '0;
            rq_ar_pend <= '0;
            rgen_count <= '0;
            rgen_gap   <= START_DELAY[15:0];
            r_beat_idx <= 8'd0;
            r_recv_count <= '0;
            protocol_error_rresp <= 1'b0;
            protocol_error_rid   <= 1'b0;
            protocol_error_rlast <= 1'b0;
            lfsr_rgen <= SEED_RGEN; 
            lfsr_r <= SEED_R;
        end else begin
            lfsr_rgen <= lfsr_next(lfsr_rgen);
            lfsr_r    <= lfsr_next(lfsr_r);

            rgen_push = (rgen_gap == 16'd0) && (rgen_count < NUM_TRANSACTIONS)
                        && (rq_occ < RCMD_DEPTH);
            ar_hs        = ARVALID && ARREADY;
            r_hs         = RVALID  && RREADY;
            r_burst_done = r_hs && (rq_occ != '0) && (r_beat_idx == rcmd[rq_r_head].len);

            // ---------------- request generator ----------------
            if (rgen_gap != 16'd0) begin
                rgen_gap <= rgen_gap - 16'd1;
            end else if (rgen_push) begin
                rcmd[rq_tail].id   <= rgen_count[ID_WIDTH-1:0];
                rcmd[rq_tail].addr <= RD_REGION_BASE + (rgen_count * TXN_ADDR_STRIDE);
                rcmd[rq_tail].len  <= BURST_LEN[7:0];
                rq_tail    <= rq_inc(rq_tail);
                rgen_count <= rgen_count + 1'b1;
                rgen_gap   <= rand_gap(lfsr_rgen);
            end

            // ---------------- AR engine (greedy issue) ----------------
            if (!ARVALID) begin
                if (rq_ar_pend != '0) begin
                    ARID    <= rcmd[rq_ar_head].id;
                    ARADDR  <= rcmd[rq_ar_head].addr;
                    ARLEN   <= rcmd[rq_ar_head].len;
                    ARVALID <= 1'b1;
                end
            end else if (ARREADY) begin
                ARVALID    <= 1'b0;
                rq_ar_head <= rq_inc(rq_ar_head);
            end

            // ---------------- R checker (random backpressure) ----------------
            RREADY <= r_gate;
            if (r_hs) begin
                if (RRESP != 2'b00) protocol_error_rresp <= 1'b1;
                if (rq_occ == '0 || RID !== rcmd[rq_r_head].id) begin
                    protocol_error_rid <= 1'b1;
                end
                if (rq_occ != '0 && (RLAST !== (r_beat_idx == rcmd[rq_r_head].len))) begin
                    protocol_error_rlast <= 1'b1;
                end
                if (rq_occ != '0) begin
                    if (r_beat_idx == rcmd[rq_r_head].len) begin
                        rq_r_head    <= rq_inc(rq_r_head);
                        r_beat_idx   <= 8'd0;
                        r_recv_count <= r_recv_count + 1'b1;
                    end else begin
                        r_beat_idx <= r_beat_idx + 8'd1;
                    end
                end
            end

            // ---------------- occupancy counters ----------------
            unique case ({rgen_push, r_burst_done}) // rq_occ: +push, −burst_done (final R beat)
                2'b10:   rq_occ <= rq_occ + 1'b1;
                2'b01:   rq_occ <= rq_occ - 1'b1;
                default: rq_occ <= rq_occ;          // 00 or 11 → unchanged
            endcase
            unique case ({rgen_push, ar_hs})        // rq_ar_pend: +push, −AR-handshake
                2'b10:   rq_ar_pend <= rq_ar_pend + 1'b1;
                2'b01:   rq_ar_pend <= rq_ar_pend - 1'b1;
                default: rq_ar_pend <= rq_ar_pend; // 00 or 11 → unchanged
            endcase
        end

        if (DEBUG && ARESETn && ARVALID && ARREADY) begin
            $display("[%t] %-3s AR >> txn=M%0d.R%0d addr=0x%08h len=%0d  (rq_occ=%0d/%0d)",
                $time, $sformatf("M%0d", MASTER_ID), MASTER_ID, ARID, ARADDR, ARLEN,
                rq_occ, RCMD_DEPTH);
        end
        if (DEBUG && ARESETn && RVALID && RREADY) begin
            $display("[%t] %-3s R  << txn=M%0d.R%0d beat=%0d resp=%-6s rdata=0x%h%s%s%s",
                $time, $sformatf("M%0d", MASTER_ID), MASTER_ID, RID, r_beat_idx, resp_str(RRESP), RDATA,
                tag_if(RLAST, " (RLAST)"),
                tag_if(RRESP != 2'b00, " <-- ERROR RESP"),
                tag_if(rq_occ == '0 || RID !== rcmd[rq_r_head].id, " <-- ID MISMATCH"));
        end
    end

endmodule

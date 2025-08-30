`include "VX_cache_define.vh"

/*
 * VX_amo_unit: RISC-V Atomic Memory Operation Unit
 *
 * Implements the logic for RISC-V 'A' extension instructions.
 * It intercepts requests to a cache bank, handles AMO operations through a state machine,
 * and passes non-AMO requests through.
 */
module VX_amo_unit import VX_gpu_pkg::*; #(
    parameter BANK_ID           = 0,
    parameter INSTANCE_ID       = "VX_amo_unit",
    // Bus Parameters
    parameter TAG_WIDTH         = 0,
    parameter WORD_WIDTH        = 0,
    parameter ADDR_WIDTH        = 0,
    // parameter MEM_FLAGS_WIDTH   = 0,
    parameter WORD_SEL_WIDTH    = 0,
    parameter REQ_SEL_WIDTH     = 0,
    parameter CS_LINE_ADDR_WIDTH = 0,
    parameter WORD_SIZE         = 0
) (
    input wire clk,
    input wire reset,

    // Interface from Core/XBar
    input  wire                         core_req_valid,
    input  wire [TAG_WIDTH-1:0]         core_req_tag,
    input  wire [`UP(ADDR_WIDTH)-1:0]   core_req_addr,
    input  wire                         core_req_rw,
    input  wire [WORD_SIZE-1:0]         core_req_byteen,
    input  wire [WORD_WIDTH-1:0]        core_req_data,
    input  wire [`UP(MEM_FLAGS_WIDTH)-1:0] core_req_flags,
    input  wire [WORD_SEL_WIDTH-1:0]    core_req_wsel,
    input  wire [REQ_SEL_WIDTH-1:0]     core_req_idx,
    output wire                         core_req_ready,

    // Interface to Core/XBar
    output wire                         core_rsp_valid,
    output wire [TAG_WIDTH-1:0]         core_rsp_tag,
    output wire [WORD_WIDTH-1:0]        core_rsp_data,
    output wire [REQ_SEL_WIDTH-1:0]     core_rsp_idx,
    input  wire                         core_rsp_ready,

    // Interface to Cache Bank
    output wire                         cache_req_valid,
    output wire [TAG_WIDTH-1:0]         cache_req_tag,
    output wire [`UP(ADDR_WIDTH)-1:0]   cache_req_addr,
    output wire                         cache_req_rw,
    output wire [WORD_SIZE-1:0]         cache_req_byteen,
    output wire [WORD_WIDTH-1:0]        cache_req_data,
    output wire [`UP(MEM_FLAGS_WIDTH)-1:0] cache_req_flags,
    output wire [WORD_SEL_WIDTH-1:0]    cache_req_wsel,
    output wire [REQ_SEL_WIDTH-1:0]     cache_req_idx,
    input  wire                         cache_req_ready,

    // Interface from Cache Bank
    input  wire                         cache_rsp_valid,
    input  wire [TAG_WIDTH-1:0]         cache_rsp_tag,
    input  wire [WORD_WIDTH-1:0]        cache_rsp_data,
    input  wire [REQ_SEL_WIDTH-1:0]     cache_rsp_idx,
    output wire                         cache_rsp_ready
);
    // AMO-related flags from the incoming request
    wire is_amo = core_req_valid && core_req_flags[MEM_REQ_FLAG_AMO];
    wire [4:0] amo_op = core_req_flags[MEM_REQ_FLAG_AMO_OP_END:MEM_REQ_FLAG_AMO_OP_BEGIN];

    // FSM state definition
    typedef enum logic [2:0] {
        AMO_IDLE,
        AMO_READ,
        AMO_WAIT_READ,
        AMO_COMPUTE,
        AMO_WRITE,
        AMO_WAIT_WRITE,
        AMO_RESPOND
    } amo_state_t;

    amo_state_t amo_state, amo_state_next;

    // Latched AMO request buffer
    struct packed {
        logic [TAG_WIDTH-1:0]         tag;
        logic [`UP(ADDR_WIDTH)-1:0]   addr;
        logic [WORD_SIZE-1:0]         byteen;
        logic [WORD_WIDTH-1:0]        data;
        logic [WORD_SEL_WIDTH-1:0]    wsel;
        logic [REQ_SEL_WIDTH-1:0]     idx;
        logic [4:0]                   amo_op;
    } amo_req_buf;

    // AMO calculated values
    logic [WORD_WIDTH-1:0] amo_old_data;
    logic [WORD_WIDTH-1:0] amo_new_data;

    // LR/SC reservation logic
    logic [`UP(ADDR_WIDTH)-1:0]     lr_reserved_addr;
    logic                           lr_reserved_valid;
    wire                            sc_addr_match = (lr_reserved_addr == amo_req_buf.addr);
    wire                            sc_success    = lr_reserved_valid && sc_addr_match;

    // FSM sequential logic
    always_ff @(posedge clk) begin
        if (reset) begin
            amo_state <= AMO_IDLE;
        end else begin
            amo_state <= amo_state_next;
        end
    end

    // FSM combinational logic
    always_comb begin
        amo_state_next = amo_state;
        case (amo_state)
            AMO_IDLE: begin
                if (is_amo && core_req_ready) begin
                    // For all AMO operations, we start with a read.
                    // For LR and R-M-W ops, it's to get the old value.
                    // For SC, it's a "read-for-ownership" to gain write permission to the cache line.
                    amo_state_next = AMO_READ;
                end
            end
            AMO_READ: begin
                if (cache_req_valid && cache_req_ready) begin
                    amo_state_next = AMO_WAIT_READ;
                end
            end
            AMO_WAIT_READ: begin
                if (cache_rsp_valid && cache_rsp_ready) begin
                    // After reading, decide the next step based on the operation type
                    case (amo_req_buf.amo_op)
                        AMO_LR: begin
                            // For LR, after reading, set the reservation and go directly to respond.
                            amo_state_next = AMO_RESPOND;
                        end
                        AMO_SC: begin
                            // For SC, after gaining ownership, go directly to write.
                            amo_state_next = AMO_WRITE;
                        end
                        default: begin
                            // Other Read-Modify-Write instructions go to compute.
                            amo_state_next = AMO_COMPUTE;
                        end
                    endcase
                end
            end
            AMO_COMPUTE: begin
                // Only arithmetic/logic operations enter this state.
                // After computation, proceed to write.
                amo_state_next = AMO_WRITE;
            end
            AMO_WRITE: begin
                if (cache_req_valid && cache_req_ready) begin
                    amo_state_next = AMO_WAIT_WRITE;
                end
            end
            AMO_WAIT_WRITE: begin
                // After writing, go to respond.
                amo_state_next = AMO_RESPOND;
            end
            AMO_RESPOND: begin
                if (core_rsp_valid && core_rsp_ready) begin
                    amo_state_next = AMO_IDLE;
                end
            end
        endcase
    end

    // Latch AMO request
    always_ff @(posedge clk) begin
        if (reset) begin
            amo_req_buf <= '0;
        end else if (amo_state == AMO_IDLE && is_amo && core_req_ready) begin
            amo_req_buf.tag        <= core_req_tag;
            amo_req_buf.addr       <= core_req_addr;
            amo_req_buf.byteen     <= core_req_byteen;
            amo_req_buf.data       <= core_req_data;
            amo_req_buf.wsel       <= core_req_wsel;
            amo_req_buf.idx        <= core_req_idx;
            amo_req_buf.amo_op     <= amo_op;
        end
    end

    // Latch old data from cache read
    always_ff @(posedge clk) begin
        if (reset) begin
            amo_old_data <= '0;
        end else if (amo_state == AMO_WAIT_READ && cache_rsp_valid && cache_rsp_ready) begin
            amo_old_data <= cache_rsp_data;
        end
    end

    // LR/SC reservation logic
    always_ff @(posedge clk) begin
        if (reset) begin
            lr_reserved_addr  <= '0;
            lr_reserved_valid <= 1'b0;
        end else begin
            // Set reservation on LR completion
            if (amo_state == AMO_WAIT_READ && amo_state_next == AMO_RESPOND && amo_req_buf.amo_op == AMO_LR) begin
                lr_reserved_addr  <= amo_req_buf.addr;
                lr_reserved_valid <= 1'b1;
            end
            // Clear reservation on any intervening write to the reserved address
            else if (lr_reserved_valid && (amo_state == AMO_WRITE) && (amo_req_buf.addr == lr_reserved_addr) && cache_req_valid && cache_req_ready) begin
                lr_reserved_valid <= 1'b0;
            end
        end
    end

    // AMO computation logic
    always_comb begin
        amo_new_data = '0;
        case (amo_req_buf.amo_op)
            AMO_AMOADD:  amo_new_data = amo_old_data + amo_req_buf.data;
            AMO_AMOSWAP: amo_new_data = amo_req_buf.data;
            AMO_AMOXOR:  amo_new_data = amo_old_data ^ amo_req_buf.data;
            AMO_AMOAND:  amo_new_data = amo_old_data & amo_req_buf.data;
            AMO_AMOOR:   amo_new_data = amo_old_data | amo_req_buf.data;
            AMO_AMOMIN:  amo_new_data = ($signed(amo_old_data) < $signed(amo_req_buf.data)) ? amo_old_data : amo_req_buf.data;
            AMO_AMOMAX:  amo_new_data = ($signed(amo_old_data) > $signed(amo_req_buf.data)) ? amo_old_data : amo_req_buf.data;
            AMO_AMOMINU: amo_new_data = (amo_old_data < amo_req_buf.data) ? amo_old_data : amo_req_buf.data;
            AMO_AMOMAXU: amo_new_data = (amo_old_data > amo_req_buf.data) ? amo_old_data : amo_req_buf.data;
            AMO_SC:   amo_new_data = sc_success ? amo_req_buf.data : amo_old_data;
            default:   amo_new_data = amo_old_data; // For LR
        endcase
    end

    // Core Request Interface
    assign core_req_ready = (amo_state == AMO_IDLE) ? (is_amo ? 1'b1 : cache_req_ready) : 1'b0;

    // Core Response Interface
    assign core_rsp_valid = (amo_state == AMO_IDLE) ? cache_rsp_valid : (amo_state == AMO_RESPOND);
    assign core_rsp_tag   = (amo_state == AMO_IDLE) ? cache_rsp_tag : amo_req_buf.tag;
    assign core_rsp_idx   = (amo_state == AMO_IDLE) ? cache_rsp_idx : amo_req_buf.idx;
    assign core_rsp_data  = (amo_state == AMO_IDLE) ? cache_rsp_data :
                            (amo_req_buf.amo_op == AMO_SC) ? {{WORD_WIDTH-1{1'b0}}, !sc_success} : // SC returns 0 for success, 1 for failure
                            amo_old_data; // Other AMOs return old value

    // Cache Request Interface
    assign cache_req_valid  = (amo_state == AMO_IDLE) ? core_req_valid :
                              (amo_state == AMO_READ)  ? 1'b1 :
                              (amo_state == AMO_WRITE) ? 1'b1 : 0;
    assign cache_req_addr   = (amo_state == AMO_IDLE) ? core_req_addr : amo_req_buf.addr;
    assign cache_req_rw     = (amo_state == AMO_IDLE) ? core_req_rw :
                              (amo_state == AMO_READ)  ? 0 : // Read operation
                              (amo_state == AMO_WRITE) ? (amo_req_buf.amo_op == AMO_SC ? sc_success : 1'b1) : 0; // Don't write on SC failure
    assign cache_req_byteen = (amo_state == AMO_IDLE) ? core_req_byteen : amo_req_buf.byteen;
    assign cache_req_data   = (amo_state == AMO_IDLE) ? core_req_data :
                              (amo_state == AMO_WRITE) ? amo_new_data : '0;
    assign cache_req_tag    = (amo_state == AMO_IDLE) ? core_req_tag : amo_req_buf.tag;
    assign cache_req_wsel   = (amo_state == AMO_IDLE) ? core_req_wsel : amo_req_buf.wsel;
    assign cache_req_idx    = (amo_state == AMO_IDLE) ? core_req_idx : amo_req_buf.idx;
    assign cache_req_flags  = (amo_state == AMO_IDLE) ? core_req_flags : '{default:0}; // Strip AMO flags for cache bank

    // Cache Response Interface
    assign cache_rsp_ready = (amo_state == AMO_IDLE) ? core_rsp_ready : (amo_state == AMO_WAIT_READ);

`ifdef SIMULATION
    always_ff @(posedge clk) begin
        if (is_amo || amo_state != AMO_IDLE) begin
            $display("%t: [VX_amo_unit-%0d] State: %s, is_amo=%b, core_req_v=%b, core_req_r=%b, cache_req_v=%b, cache_req_r=%b, core_rsp_v=%b, core_rsp_r=%b, cache_rsp_v=%b, cache_rsp_r=%b, amo_op=%h, addr=%h, wdata=%h, rdata=%h, new_data=%h, sc_succ=%b",
                $time, BANK_ID, amo_state.name(), is_amo, core_req_valid, core_req_ready, cache_req_valid, cache_req_ready, core_rsp_valid, core_rsp_ready, cache_rsp_valid, cache_rsp_ready, amo_req_buf.amo_op, amo_req_buf.addr, amo_req_buf.data, amo_old_data, amo_new_data, sc_success);
        end
    end
`endif

endmodule

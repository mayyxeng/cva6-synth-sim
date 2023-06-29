
module ariane_bare_tb #(
    parameter int unsigned AXI_USER_WIDTH    = ariane_pkg::AXI_USER_WIDTH,
    parameter int unsigned AXI_USER_EN       = ariane_pkg::AXI_USER_EN,
    parameter int unsigned AXI_ADDRESS_WIDTH = 64,
    parameter int unsigned AXI_DATA_WIDTH    = 64,
    parameter bit          InclSimDTM        = 1'b1,
    parameter int unsigned NUM_WORDS         = 2**14,         // memory size
    parameter bit          StallRandomOutput = 1'b0,
    parameter bit          StallRandomInput  = 1'b0
) (
// `ifdef VERIPOPLAR
   input wire clk_i
// `endif
);
// `ifndef VERIPOPLAR
//     logic clk_i = 0;
//     always #5 clk_i = ~clk_i;
// `endif

    logic [31:0] cycle_count = 0;
    logic verbose;
    logic [31:0] MAX_CYCLES;
    logic COMMIT_TRACE;
    initial begin
        if(!$value$plusargs("MAX_CYCLES=%d", MAX_CYCLES)) MAX_CYCLES = 1000;
        if ($test$plusargs("TRACE")) begin
            $display("Tracing to vlt_dump.vcd...");
            $dumpfile("vlt_dump.vcd");
            $dumpvars();
        end
        verbose = $test$plusargs("VERBOSE");
        COMMIT_TRACE = verbose | $test$plusargs("TRACE");
    end
    logic rst_n ;
    initial rst_n = 0;

    always_ff @(posedge clk_i) rst_n <= cycle_count > 10;
    always_ff @(posedge clk_i) cycle_count <= cycle_count + 1;
    always_ff @(posedge clk_i) begin
        if (cycle_count == MAX_CYCLES) begin
            $display("@%d MAX_CYCLES reached", cycle_count);
            if ($test$plusargs("FAIL_ON_TIMEOUT")) begin
                $stop;
            end else begin
                $finish;
            end
        end
    end

    // --------------
    //  BUS
    // --------------

    AXI_BUS #(
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
        .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    ) bus();


    logic                         req;
    logic                         we;
    logic [AXI_ADDRESS_WIDTH-1:0] addr;
    logic [AXI_DATA_WIDTH/8-1:0]  be;
    logic [AXI_DATA_WIDTH-1:0]    wdata;
    logic [AXI_DATA_WIDTH-1:0]    rdata;
    logic [AXI_USER_WIDTH-1:0]    wuser;
    logic [AXI_USER_WIDTH-1:0]    ruser;

    axi2mem #(
        .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    ) i_axi2mem (
        .clk_i  ( clk_i        ),
        .rst_ni ( rst_n        ),
        .slave  ( bus          ),
        .req_o  ( req          ),
        .we_o   ( we           ),
        .addr_o ( addr         ),
        .be_o   ( be           ),
        .user_o ( wuser        ),
        .data_o ( wdata        ),
        .user_i ( ruser        ),
        .data_i ( rdata        )
    );

    function logic isInRange(logic [AXI_ADDRESS_WIDTH - 1: 0] in_addr,
                                logic [AXI_ADDRESS_WIDTH - 1 : 0] base,
                                logic [AXI_ADDRESS_WIDTH - 1 : 0] length);
        return (in_addr >= base) && (in_addr < base + length);
    endfunction
    function logic[AXI_ADDRESS_WIDTH - 1 : 0] adjustedRange(
                                logic [AXI_ADDRESS_WIDTH - 1: 0] in_addr,
                                logic [AXI_ADDRESS_WIDTH - 1 : 0] base,
                                logic [AXI_ADDRESS_WIDTH - 1 : 0] length);
        return isInRange(in_addr, base, length) ? in_addr - base : '0;
    endfunction
    wire [AXI_DATA_WIDTH - 1 : 0] rom_rdata;

    bootrom i_bootrom (
        .clk_i      ( clk_i     ),
        .req_i      ( isInRange(addr, ariane_soc::ROMBase, ariane_soc::ROMLength) && req),
        .addr_i     ( adjustedRange(addr, ariane_soc::ROMBase, ariane_soc::ROMLength)   ),
        .rdata_o    ( rom_rdata )
    );

    wire [AXI_ADDRESS_WIDTH - 1 : 0] sram_addr = adjustedRange(addr, ariane_soc::DRAMBase, ariane_soc::DRAMLength);
    wire [AXI_DATA_WIDTH - 1 : 0] sram_rdata;

    preloadable_sram #(
        .DATA_WIDTH ( AXI_DATA_WIDTH ),
        .USER_WIDTH ( AXI_USER_WIDTH ),
        .USER_EN    ( AXI_USER_EN    ),
        .NUM_WORDS  ( NUM_WORDS      )
    ) i_main_sram (
        .clk_i      ( clk_i                                                                       ),
        .rst_ni     ( rst_n                                                                       ),
        .req_i      ( isInRange(addr, ariane_soc::DRAMBase, ariane_soc::DRAMLength) && req        ),
        .we_i       ( isInRange(addr, ariane_soc::DRAMBase, ariane_soc::DRAMLength) && we         ),
        .addr_i     ( sram_addr[$clog2(NUM_WORDS)-1+$clog2(AXI_DATA_WIDTH/8):$clog2(AXI_DATA_WIDTH/8)] ),
        .wuser_i    ( wuser                                                                       ),
        .wdata_i    ( wdata                                                                       ),
        .be_i       ( be                                                                          ),
        .ruser_o    ( ruser                                                                       ),
        .rdata_o    ( sram_rdata                                                                  )
    );
    logic [AXI_ADDRESS_WIDTH - 1 : 0] addr_q;
    always_ff @(posedge clk_i) addr_q <= addr;
    assign rdata = isInRange(addr_q, ariane_soc::DRAMBase, ariane_soc::DRAMLength) ? sram_rdata : rom_rdata;

    wire gpio_req = req && isInRange(addr, ariane_soc::GPIOBase, ariane_soc::GPIOLength);
    wire gpio_we = req && we;
    wire gpio_be = be;
    always_ff @(posedge clk_i) begin
        if (gpio_req) begin
            if (gpio_we && be == {(AXI_DATA_WIDTH / 8){1'b1}}) begin
                if (wdata[0] == 0) begin
                    $display("@%d Wrong toHost code %h", cycle_count, wdata);
                    $stop;
                end else if (wdata != 1) begin
                    $display("@%d test failed! toHost = %h", cycle_count, wdata);
                    $stop;
                end else begin
                    $display("@%d test passed!", cycle_count);
                    $finish;
                end
            end
        end
    end
    // ---------------
    // Core
    // ---------------
    ariane_axi::req_t    axi_ariane_req;
    ariane_axi::resp_t   axi_ariane_resp;

    ariane #(
        .ArianeCfg  ( ariane_soc::ArianeSocCfg )
    ) i_ariane (
        .clk_i                ( clk_i               ),
        .rst_ni               ( rst_n               ),
        .boot_addr_i          ( ariane_soc::ROMBase ), // start fetching from ROM
        .hart_id_i            ( {56'h0, hart_id}    ),
        .irq_i                ( '0                  ),
        .ipi_i                ( '0                  ),
        .time_irq_i           ( timer_irq           ),
        .debug_req_i          ( 1'b0                ),
        .axi_req_o            ( axi_ariane_req      ),
        .axi_resp_i           ( axi_ariane_resp     )
    );

    assign i_ariane.i_cva6.vlt_trace_commit = COMMIT_TRACE;

    // AXI_BUS #(
    //     .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    //     .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    //     .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    //     .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    // ) ariane_axi_bus();


    // axi_multicut_intf #(
    //     .ADDR_WIDTH ( AXI_ADDRESS_WIDTH ),
    //     .DATA_WIDTH ( AXI_DATA_WIDTH ),
    //     .ID_WIDTH   ( ariane_soc::IdWidth ),
    //     .USER_WIDTH ( AXI_USER_WIDTH ),
    //     .NUM_CUTS   ( 1 )
    // ) i_bus_cut (
    //     .clk_i(clk_i),
    //     .rst_ni(rst_ni),
    //     .in(ariane_axi_bus),
    //     .out(bus)
    // );



    `AXI_ASSIGN_FROM_REQ(bus, axi_ariane_req)
    `AXI_ASSIGN_TO_RESP(axi_ariane_resp, bus)

endmodule


module preloadable_sram #(
    parameter DATA_WIDTH = 64,
    parameter USER_WIDTH = 1,
    parameter USER_EN    = 0,
    parameter NUM_WORDS  = 1024,
    parameter SIM_INIT   = "none",
    parameter OUT_REGS   = 0     // enables output registers in FPGA macro (read lat = 2)
)(
   input  logic                          clk_i,
   input  logic                          rst_ni,
   input  logic                          req_i,
   input  logic                          we_i,
   input  logic [$clog2(NUM_WORDS)-1:0]  addr_i,
   input  logic [USER_WIDTH-1:0]         wuser_i,
   input  logic [DATA_WIDTH-1:0]         wdata_i,
   input  logic [(DATA_WIDTH+7)/8-1:0]   be_i,
   output logic [USER_WIDTH-1:0]         ruser_o,
   output logic [DATA_WIDTH-1:0]         rdata_o
);

localparam DATA_WIDTH_ALIGNED = ((DATA_WIDTH+63)/64)*64;
localparam USER_WIDTH_ALIGNED = DATA_WIDTH_ALIGNED; // To be fine tuned to reduce memory size
localparam BE_WIDTH_ALIGNED   = (((DATA_WIDTH+7)/8+7)/8)*8;

logic [DATA_WIDTH_ALIGNED-1:0]  wdata_aligned;
logic [USER_WIDTH_ALIGNED-1:0]  wuser_aligned;
logic [BE_WIDTH_ALIGNED-1:0]    be_aligned;
logic [DATA_WIDTH_ALIGNED-1:0]  rdata_aligned;
logic [USER_WIDTH_ALIGNED-1:0]  ruser_aligned;

// align to 64 bits for inferrable macro below
always_comb begin : p_align
    wdata_aligned                    ='0;
    wuser_aligned                    ='0;
    be_aligned                       ='0;
    wdata_aligned[DATA_WIDTH-1:0]    = wdata_i;
    wuser_aligned[USER_WIDTH-1:0]    = wuser_i;
    be_aligned[BE_WIDTH_ALIGNED-1:0] = be_i;

    rdata_o = rdata_aligned[DATA_WIDTH-1:0];
    ruser_o = ruser_aligned[USER_WIDTH-1:0];
end

  for (genvar k = 0; k<(DATA_WIDTH+63)/64; k++) begin : gen_cut
      // unused byte-enable segments (8bits) are culled by the tool
      preloadable_sram_bank #(
        .NumWords(NUM_WORDS),           // Number of Words in data array
        .DataWidth(64),                 // Data signal width
        .ByteWidth(32'd8),              // Width of a data byte
        .NumPorts(32'd1),               // Number of read and write ports
      ) i_tc_sram_wrapper (
          .clk_i    ( clk_i                     ),
          .rst_ni   ( rst_ni                    ),
          .req_i    ( req_i                     ),
          .we_i     ( we_i                      ),
          .be_i     ( be_aligned[k*8 +: 8]      ),
          .wdata_i  ( wdata_aligned[k*64 +: 64] ),
          .addr_i   ( addr_i                    ),
          .rdata_o  ( rdata_aligned[k*64 +: 64] )
      );
      if (USER_EN > 0) begin : gen_mem_user
        preloadable_sram_bank #(
          .NumWords(NUM_WORDS),           // Number of Words in data array
          .DataWidth(64),                 // Data signal width
          .ByteWidth(32'd8),              // Width of a data byte
          .NumPorts(32'd1),               // Number of read and write ports
        ) i_tc_sram_wrapper_user (
            .clk_i    ( clk_i                     ),
            .rst_ni   ( rst_ni                    ),
            .req_i    ( req_i                     ),
            .we_i     ( we_i                      ),
            .be_i     ( be_aligned[k*8 +: 8]      ),
            .wdata_i  ( wuser_aligned[k*64 +: 64] ),
            .addr_i   ( addr_i                    ),
            .rdata_o  ( ruser_aligned[k*64 +: 64] )
        );
      end else begin
        assign ruser_aligned[k*64 +: 64] = '0;
      end
  end
endmodule : preloadable_sram

module  preloadable_sram_bank #(
    parameter int unsigned NumWords     = 32'd1024, // Number of Words in data array
    parameter int unsigned DataWidth    = 32'd128,  // Data signal width
    parameter int unsigned ByteWidth    = 32'd8,    // Width of a data byte
    parameter int unsigned NumPorts     = 32'd2,    // Number of read and write ports
    // DEPENDENT PARAMETERS, DO NOT OVERWRITE!
    parameter int unsigned AddrWidth = (NumWords > 32'd1) ? $clog2(NumWords) : 32'd1,
    parameter int unsigned BeWidth   = (DataWidth + ByteWidth - 32'd1) / ByteWidth, // ceil_div
    parameter type         addr_t    = logic [AddrWidth-1:0],
    parameter type         data_t    = logic [DataWidth-1:0],
    parameter type         be_t      = logic [BeWidth-1:0],
    parameter type         str_t     = logic [7:0][255:0]
) (
    input logic                  clk_i,   // Clock
    input logic                  rst_ni,  // Asynchronous active low reset
    // input ports
    input logic   [NumPorts-1:0] req_i,   // request
    input logic   [NumPorts-1:0] we_i,    // write enable
    input addr_t  [NumPorts-1:0] addr_i,  // request address
    input data_t  [NumPorts-1:0] wdata_i, // write data
    input be_t    [NumPorts-1:0] be_i,    // write byte enable
    // output ports
    output data_t [NumPorts-1:0] rdata_o  // read data
);

    // memory array
    data_t storage [NumWords-1:0];
    // register the read address
    addr_t [NumPorts-1:0] r_addr_q;

    initial begin: load_data
        str_t filename = '0;
        if (!$value$plusargs("HEXFILE=%s", filename)) begin
            $display("+HEXFILE=RISCV.hex is required to initialize the main memory");
            $stop;
        end
        $readmemh(filename, storage);
    end

    always_ff @(posedge clk_i) begin

        for (int unsigned port = 0; port < NumPorts; port = port + 1) begin
            if (req_i[port]) begin
                if (we_i[port]) begin // do write
                    for (int unsigned i = 0; i < BeWidth; i = i + 1) begin
                        if (be_i[port][i]) begin
                            storage[addr_i[port]][i * ByteWidth +: ByteWidth] <= wdata_i[port][i * ByteWidth +: ByteWidth];
                        end
                    end
                end else begin // do read
                    r_addr_q[port] <= addr_i[port];
                end
            end
        end
    end

    always_comb begin
        for (int unsigned port = 0; port < NumPorts; port = port + 1) begin
            rdata_o[port] = storage[r_addr_q[port]];
        end
    end
endmodule : preloadable_sram_bank

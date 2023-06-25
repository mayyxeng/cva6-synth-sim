
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
        // if ($test$plusargs("TRACE")) begin
        //     $display("Tracing to vlt_dump.vcd...");
        //     $dumpfile("vlt_dump.vcd");
        //     $dumpvars();
        // end
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

    // AXI slave bus interface, connects to the core
    AXI_BUS #(
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH   ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH      ),
        .AXI_ID_WIDTH   ( ariane_soc::IdWidth ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH      )
    ) slave[ariane_soc::NrSlaves-1:0](); // one is unconnected, since there is no DM

    AXI_BUS #(
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
        .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    ) master[ariane_soc::NB_PERIPHERALS-1:0]();
    // need only 3 masters: DRAM, ROM and GPIO but we instantiate more for no reason


    // ---------------
    // BOOT ROM
    // ---------------
    logic                         rom_req;
    logic [AXI_ADDRESS_WIDTH-1:0] rom_addr;
    logic [AXI_DATA_WIDTH-1:0]    rom_rdata;

    axi2mem #(
        .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    ) i_axi2rom (
        .clk_i  ( clk_i                   ),
        .rst_ni ( rst_n                   ),
        .slave  ( master[ariane_soc::ROM] ),
        .req_o  ( rom_req                 ),
        .we_o   (                         ),
        .addr_o ( rom_addr                ),
        .be_o   (                         ),
        .user_o (                         ),
        .data_o (                         ),
        .user_i ( '0                      ),
        .data_i ( rom_rdata               )
    );

    bootrom i_bootrom (
        .clk_i      ( clk_i     ),
        .req_i      ( rom_req   ),
        .addr_i     ( rom_addr  ),
        .rdata_o    ( rom_rdata )
    );

    // ----------------
    // GPIO: a proxy for toHost fromHost
    // ----------------

    logic                         gpio_req;
    logic                         gpio_we;
    logic [AXI_ADDRESS_WIDTH-1:0] gpio_addr;
    logic [AXI_DATA_WIDTH/8-1:0]  gpio_be;
    logic [AXI_DATA_WIDTH-1:0]    gpio_wdata;
    logic [AXI_DATA_WIDTH-1:0]    gpio_rdata;
    logic [AXI_USER_WIDTH-1:0]    gpio_wuser;
    logic [AXI_USER_WIDTH-1:0]    gpio_ruser;

    axi2mem #(
        .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    ) i_axi2gpio (
        .clk_i  ( clk_i                   ),
        .rst_ni ( rst_n                   ),
        .slave  ( master[ariane_soc::GPIO]),
        .req_o  ( gpio_req                ),
        .we_o   ( gpio_we                 ),
        .addr_o ( gpio_addr               ),
        .be_o   ( gpio_be                 ),
        .user_o ( gpio_wuser              ),
        .data_o ( gpio_wdata              ),
        .user_i ( gpio_ruser              ),
        .data_i ( gpio_rdata              )
    );

    always_ff @(posedge clk_i) begin
        if (gpio_req) begin
            if (gpio_we && gpio_be == {(AXI_DATA_WIDTH / 8){1'b1}}) begin
                if (gpio_wdata[0] == 0) begin
                    $display("@%d Wrong toHost code %h", cycle_count, gpio_wdata);
                    $stop;
                end else if (gpio_wdata != 1) begin
                    $display("@%d test failed! toHost = %h", cycle_count, gpio_wdata);
                    $stop;
                end else begin
                    $display("@%d test passed!", cycle_count);
                    $finish;
                end
            end
        end
    end
    // ------------------------------
    // Memory + Exclusive Access
    // ------------------------------

    // AXI_BUS #(
    //     .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    //     .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    //     .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    //     .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    // ) dram_up ();

    // AXI_BUS #(
    //     .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    //     .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    //     .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    //     .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    // ) dram ();


    logic                         req;
    logic                         we;
    logic [AXI_ADDRESS_WIDTH-1:0] addr;
    logic [AXI_DATA_WIDTH/8-1:0]  be;
    logic [AXI_DATA_WIDTH-1:0]    wdata;
    logic [AXI_DATA_WIDTH-1:0]    rdata;
    logic [AXI_USER_WIDTH-1:0]    wuser;
    logic [AXI_USER_WIDTH-1:0]    ruser;

    // axi_cut_intf #(
    //     .BYPASS ( 1'b0 ),
    //     .ADDR_WIDTH ( AXI_ADDRESS_WIDTH ),
    //     .DATA_WIDTH ( AXI_DATA_WIDTH ),
    //     .ID_WIDTH ( ariane_soc::IdWidthSlave ),
    //     .USER_WIDTH ( AXI_USER_WIDTH )
    // ) i_axi_cut_bus_side (
    //     .clk_i ( clk_i ),
    //     .rst_ni ( rst_ni ),
    //     .in ( master[ariane_soc::DRAM] ),
    //     .out ( dram_up )
    // );

    // axi_riscv_atomics_wrap #(
    //     .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    //     .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    //     .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    //     .AXI_USER_WIDTH ( AXI_USER_WIDTH           ),
    //     .AXI_MAX_WRITE_TXNS ( 1  ),
    //     .RISCV_WORD_WIDTH   ( 64 )
    // ) i_axi_riscv_atomics (
    //     .clk_i  ( clk_i                    ),
    //     .rst_ni ( rst_n                    ),
    //     .slv    ( master[ariane_soc::DRAM] ),
    //     .mst    ( dram                     )
    // );

    // AXI_BUS #(
    //     .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
    //     .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
    //     .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
    //     .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    // ) dram_delayed();

    // axi_delayer_intf #(
    //     .AXI_ID_WIDTH        ( ariane_soc::IdWidthSlave ),
    //     .AXI_ADDR_WIDTH      ( AXI_ADDRESS_WIDTH        ),
    //     .AXI_DATA_WIDTH      ( AXI_DATA_WIDTH           ),
    //     .AXI_USER_WIDTH      ( AXI_USER_WIDTH           ),
    //     .STALL_RANDOM_INPUT  ( StallRandomInput         ),
    //     .STALL_RANDOM_OUTPUT ( StallRandomOutput        ),
    //     .FIXED_DELAY_INPUT   ( 0                        ),
    //     .FIXED_DELAY_OUTPUT  ( 0                        )
    // ) i_axi_delayer (
    //     .clk_i  ( clk_i        ),
    //     .rst_ni ( ndmreset_n   ),
    //     .slv    ( dram         ),
    //     .mst    ( dram_delayed )
    // );
    // axi_cut_intf #(
    //     .BYPASS ( 1'b0 ),
    //     .ADDR_WIDTH ( AXI_ADDRESS_WIDTH ),
    //     .DATA_WIDTH ( AXI_DATA_WIDTH ),
    //     .ID_WIDTH ( ariane_soc::IdWidthSlave ),
    //     .USER_WIDTH ( AXI_USER_WIDTH )
    // ) i_axi_cut_dram_side (
    //     .clk_i ( clk_i ),
    //     .rst_ni ( rst_ni ),
    //     .in ( dram_atom ),
    //     .out ( dram_mem )
    // );
    // logic t_r_vld, t_r_rdy, t_ar_vld, t_ar_rdy;
    // logic [AXI_ADDRESS_WIDTH - 1 : 0] t_ar_addr;
    // logic [AXI_DATA_WIDTH - 1 : 0] t_r_data;

    // always_ff @(posedge clk_i) begin
    //     if (verbose) begin
    //         t_r_vld <= dram.r_valid;
    //         t_r_rdy <= dram.r_ready;
    //         t_ar_vld <= dram.ar_valid;
    //         t_ar_rdy <= dram.ar_ready;
    //         t_ar_addr <= dram.ar_addr;
    //         t_r_data <= dram.r_data;
    //         if(t_r_vld && t_r_rdy) begin
    //             $display("@%d rdata: %h", cycle_count, t_r_data);
    //         end
    //         if (t_ar_vld && t_ar_rdy) begin
    //             $display("@%d raddr: %h", cycle_count, t_ar_addr);
    //         end
    //     end
    // end

    axi2mem #(
        .AXI_ID_WIDTH   ( ariane_soc::IdWidthSlave ),
        .AXI_ADDR_WIDTH ( AXI_ADDRESS_WIDTH        ),
        .AXI_DATA_WIDTH ( AXI_DATA_WIDTH           ),
        .AXI_USER_WIDTH ( AXI_USER_WIDTH           )
    ) i_axi2mem (
        .clk_i  ( clk_i        ),
        .rst_ni ( rst_n        ),
        .slave  ( master[ariane_soc::DRAM]         ),
        .req_o  ( req          ),
        .we_o   ( we           ),
        .addr_o ( addr         ),
        .be_o   ( be           ),
        .user_o ( wuser        ),
        .data_o ( wdata        ),
        .user_i ( ruser        ),
        .data_i ( rdata        )
    );

    preloadable_sram #(
        .DATA_WIDTH ( AXI_DATA_WIDTH ),
        .USER_WIDTH ( AXI_USER_WIDTH ),
        .USER_EN    ( AXI_USER_EN    ),
        .NUM_WORDS  ( NUM_WORDS      )
    ) i_main_sram (
        .clk_i      ( clk_i                                                                       ),
        .rst_ni     ( rst_n                                                                       ),
        .req_i      ( req                                                                         ),
        .we_i       ( we                                                                          ),
        .addr_i     ( addr[$clog2(NUM_WORDS)-1+$clog2(AXI_DATA_WIDTH/8):$clog2(AXI_DATA_WIDTH/8)] ),
        .wuser_i    ( wuser                                                                       ),
        .wdata_i    ( wdata                                                                       ),
        .be_i       ( be                                                                          ),
        .ruser_o    ( ruser                                                                       ),
        .rdata_o    ( rdata                                                                       )
    );

    // ---------------
    // AXI Xbar
    // ---------------

    axi_pkg::xbar_rule_64_t [ariane_soc::NB_PERIPHERALS-1:0] addr_map;

    assign addr_map = '{
        '{ idx: ariane_soc::Debug,    start_addr: ariane_soc::DebugBase,    end_addr: ariane_soc::DebugBase + ariane_soc::DebugLength       },
        '{ idx: ariane_soc::ROM,      start_addr: ariane_soc::ROMBase,      end_addr: ariane_soc::ROMBase + ariane_soc::ROMLength           },
        '{ idx: ariane_soc::CLINT,    start_addr: ariane_soc::CLINTBase,    end_addr: ariane_soc::CLINTBase + ariane_soc::CLINTLength       },
        '{ idx: ariane_soc::PLIC,     start_addr: ariane_soc::PLICBase,     end_addr: ariane_soc::PLICBase + ariane_soc::PLICLength         },
        '{ idx: ariane_soc::UART,     start_addr: ariane_soc::UARTBase,     end_addr: ariane_soc::UARTBase + ariane_soc::UARTLength         },
        '{ idx: ariane_soc::Timer,    start_addr: ariane_soc::TimerBase,    end_addr: ariane_soc::TimerBase + ariane_soc::TimerLength       },
        '{ idx: ariane_soc::SPI,      start_addr: ariane_soc::SPIBase,      end_addr: ariane_soc::SPIBase + ariane_soc::SPILength           },
        '{ idx: ariane_soc::Ethernet, start_addr: ariane_soc::EthernetBase, end_addr: ariane_soc::EthernetBase + ariane_soc::EthernetLength },
        '{ idx: ariane_soc::GPIO,     start_addr: ariane_soc::GPIOBase,     end_addr: ariane_soc::GPIOBase + ariane_soc::GPIOLength         },
        '{ idx: ariane_soc::DRAM,     start_addr: ariane_soc::DRAMBase,     end_addr: ariane_soc::DRAMBase + ariane_soc::DRAMLength         }
    };

    localparam axi_pkg::xbar_cfg_t AXI_XBAR_CFG = '{
        NoSlvPorts: unsigned'(ariane_soc::NrSlaves),
        NoMstPorts: unsigned'(ariane_soc::NB_PERIPHERALS),
        MaxMstTrans: unsigned'(1), // Probably requires update
        MaxSlvTrans: unsigned'(1), // Probably requires update
        FallThrough: 1'b0,
        LatencyMode: axi_pkg::CUT_ALL_PORTS,
        AxiIdWidthSlvPorts: unsigned'(ariane_soc::IdWidth),
        AxiIdUsedSlvPorts: unsigned'(ariane_soc::IdWidth),
        UniqueIds: 1'b0,
        AxiAddrWidth: unsigned'(AXI_ADDRESS_WIDTH),
        AxiDataWidth: unsigned'(AXI_DATA_WIDTH),
        NoAddrRules: unsigned'(ariane_soc::NB_PERIPHERALS)
    };

    axi_xbar_intf #(
        .AXI_USER_WIDTH ( AXI_USER_WIDTH          ),
        .Cfg            ( AXI_XBAR_CFG            ),
        .rule_t         ( axi_pkg::xbar_rule_64_t )
    ) i_axi_xbar (
        .clk_i                 ( clk_i      ),
        .rst_ni                ( rst_n      ),
        .test_i                ( '0         ),
        .slv_ports             ( slave      ),
        .mst_ports             ( master     ),
        .addr_map_i            ( addr_map   ),
        .en_default_mst_port_i ( '0         ),
        .default_mst_port_i    ( '0         )
    );

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

    `AXI_ASSIGN_FROM_REQ(slave[0], axi_ariane_req)
    `AXI_ASSIGN_TO_RESP(axi_ariane_resp, slave[0])

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

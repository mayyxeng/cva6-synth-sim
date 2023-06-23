// Verilated -*- C++ -*-
// DESCRIPTION: main() calling loop, created with Verilator --main

#include "verilated.h"
#include "Variane_bare_tb.h"

//======================

int main(int argc, char** argv, char**) {
    // Setup context, defaults, and parse command line
    Verilated::debug(0);
    const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};
    contextp->traceEverOn(true);
    contextp->commandArgs(argc, argv);

    // Construct the Verilated model, from Vtop.h generated from Verilating
    const std::unique_ptr<Variane_bare_tb> topp{new Variane_bare_tb{contextp.get()}};

    topp->clk_i = 0;
    // Simulate until $finish
    while (!contextp->gotFinish()) {
        // Evaluate model
        topp->clk_i = 0;
        topp->eval();
        // Advance time
        contextp->timeInc(1);

        topp->clk_i = 1;
        topp->eval();
        contextp->timeInc(1);
    }

    if (!contextp->gotFinish()) {
        VL_DEBUG_IF(VL_PRINTF("+ Exiting without $finish; no events left\n"););
    }

    // Final model cleanup
    topp->final();
    return 0;
}


# root path
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
root-dir := $(dir $(mkfile_path))


ifndef CVA6_REPO_DIR
$(warning must set CVA6_REPO_DIR to point at the root of CVA6 sources -- doing it for you...)
    export CVA6_REPO_DIR = $(abspath $(root-dir))
endif

verilator ?= verilator

target := cv64a6_imafdc_sv39_openpiton

top := ariane_bare_tb

ifndef TARGET_CFG
    export TARGET_CFG = $(target)
endif

ariane_pkg := \
              corev_apu/register_interface/src/reg_intf.sv           \
              corev_apu/tb/ariane_soc_pkg.sv                         \
              corev_apu/riscv-dbg/src/dm_pkg.sv                      \
              corev_apu/tb/ariane_axi_soc_pkg.sv

src :=  core/include/$(target)_config_pkg.sv                                         \
        corev_apu/src/ariane.sv                                                      \
        $(wildcard corev_apu/bootrom/*.sv)                                           \
        $(wildcard corev_apu/src/axi_riscv_atomics/src/*.sv)                         \
        $(wildcard corev_apu/axi_mem_if/src/*.sv)                                    \
        vendor/pulp-platform/axi/src/axi_cut.sv                                      \
        vendor/pulp-platform/axi/src/axi_join.sv                                     \
        vendor/pulp-platform/axi/src/axi_delayer.sv                                  \
        vendor/pulp-platform/axi/src/axi_to_axi_lite.sv                              \
        vendor/pulp-platform/axi/src/axi_id_prepend.sv                               \
        vendor/pulp-platform/axi/src/axi_atop_filter.sv                              \
        vendor/pulp-platform/axi/src/axi_err_slv.sv                                  \
        vendor/pulp-platform/axi/src/axi_mux.sv                                      \
        vendor/pulp-platform/axi/src/axi_demux.sv                                    \
        vendor/pulp-platform/axi/src/axi_xbar.sv                                     \
        vendor/pulp-platform/common_cells/src/deprecated/fifo_v1.sv                  \
        vendor/pulp-platform/common_cells/src/deprecated/fifo_v2.sv                  \
        corev_apu/tb/ariane_bare_tb.sv


# Search here for include files (e.g.: non-standalone components)
incdir := vendor/pulp-platform/common_cells/include/   \
          vendor/pulp-platform/axi/include/            \
          corev_apu/register_interface/include/

# Iterate over all include directories and write them with +incdir+ prefixed
# +incdir+ works for Verilator and QuestaSim
list_incdir := $(foreach dir, ${incdir}, +incdir+$(dir))

ver-library  ?= obj_dir
# verilator = $(VERIPOPLAR_ROOT)/bin/verilator_bin_dbg --poplar
verilator += --cc
# verilator-specific
verilate_command := $(verilator) verilator_config.vlt                                                            \
                    -f core/Flist.cva6                                                                           \
                    $(filter-out %.vhd, $(ariane_pkg))                                                           \
                    $(filter-out core/fpu_wrap.sv, $(filter-out %.vhd, $(src)))                                  \
                    +define+$(defines)$(if $(TRACE_FAST),+VM_TRACE)$(if $(TRACE_COMPACT),+VM_TRACE+VM_TRACE_FST) \
                    corev_apu/tb/common/mock_uart.sv                                                             \
                    +incdir+corev_apu/axi_node                                                                   \
                    --unroll-count 256                                                                           \
                    -Wall                                                                                        \
                    -Werror-PINMISSING                                                                           \
                    -Werror-IMPLICIT                                                                             \
                    -Wno-fatal                                                                                   \
                    -Wno-PINCONNECTEMPTY                                                                         \
                    -Wno-ASSIGNDLY                                                                               \
                    -Wno-DECLFILENAME                                                                            \
                    -Wno-UNUSED                                                                                  \
                    -Wno-style                                                                                   \
					-Wno-WIDTH                                                                                   \
					-no-timing                                                                                   \
					-Wno-lint                                                                                    \
                    -Wno-WIDTHCONCAT                                                                             \
                    -Wno-BLKANDNBLK                                                                              \
                    --stats --stats-vars                                                                         \
                    $(list_incdir) --top-module $(top)                                                           \
                    --Mdir $(ver-library) -O3                                                                    \
                    --exe corev_apu/tb/ariane_bare_harness.cpp   -CFLAGS -O3 --debug-emitv --debugi-V3Split 3    \
                    --dumpi-V3Split 3 --dumpi-graph 0 -fno-table \
                    --debugi-V3SplitVar 3

conversion_cmd := sv2v  -DVERILATOR                                            \
                        vendor/openhwgroup/cvfpu/src/fpnew_top.sv              \
                        vendor/openhwgroup/cvfpu/src/fpnew_pkg.sv              \
                        vendor/pulp-platform/common_cells/src/cf_math_pkg.sv   \
                       -y vendor/openhwgroup/cvfpu/src/                        \
                       -I vendor/openhwgroup/cvfpu/src/                        \
                       -y vendor/pulp-platform/common_cells/src                \
                       -I vendor/pulp-platform/common_cells/include            \
                       --top fpnew_top --dump-prefix=temp/




default: verilate

verilate:
	@echo "[Verilator] Building Model$(if $(PROFILE), for Profiling,)"
	$(verilate_command)
	$(MAKE) -C obj_dir -f V$(top).mk -j 30

convert:
	$(conversion_cmd)

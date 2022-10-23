#########################################################################################
# pre-process PicoRV into a single blackbox file
#########################################################################################
base_dir=$(abspath ../../../../../..)
vsrc_dir=$(abspath .)
picorv_dir=$(vsrc_dir)/picorv32

# name of output pre-processed verilog file
PREPROC_VERILOG = PicoRVCoreBlackbox.preprocessed.sv

.PHONY: default $(PREPROC_VERILOG)
default: $(PREPROC_VERILOG)

#########################################################################################
# includes and vsrcs
#########################################################################################
PicoRV_PKGS = \
	$(picorv_dir)/include/riscv_pkg.sv \
	$(picorv_dir)/src/riscv-dbg/src/dm_pkg.sv \
	$(picorv_dir)/include/ariane_pkg.sv \
	$(picorv_dir)/include/std_cache_pkg.sv \
	$(picorv_dir)/include/wt_cache_pkg.sv \
	$(picorv_dir)/src/axi/src/axi_pkg.sv \
	$(picorv_dir)/src/register_interface/src/reg_intf.sv \
	$(picorv_dir)/src/register_interface/src/reg_intf_pkg.sv \
	$(picorv_dir)/include/axi_intf.sv \
	$(picorv_dir)/tb/ariane_soc_pkg.sv \
	$(picorv_dir)/include/ariane_axi_pkg.sv \
	$(picorv_dir)/src/fpu/src/fpnew_pkg.sv \
	$(picorv_dir)/src/fpu/src/fpu_div_sqrt_mvp/hdl/defs_div_sqrt_mvp.sv \
	$(picorv_dir)/include/traced_instr_pkg.sv

PicoRV_VSRCS = \
	$(picorv_dir)/src/alu.sv \
	$(picorv_dir)/src/amo_buffer.sv \
	$(picorv_dir)/src/ariane_regfile_ff.sv \
	$(picorv_dir)/src/ariane.sv \
	$(picorv_dir)/src/axi_shim.sv \
	$(picorv_dir)/src/branch_unit.sv \
	$(picorv_dir)/src/commit_stage.sv \
	$(picorv_dir)/src/compressed_decoder.sv \
	$(picorv_dir)/src/controller.sv \
	$(picorv_dir)/src/csr_buffer.sv \
	$(picorv_dir)/src/csr_regfile.sv \
	$(picorv_dir)/src/decoder.sv \
	$(picorv_dir)/src/dromajo_ram.sv \
	$(picorv_dir)/src/ex_stage.sv \
	$(picorv_dir)/src/fpu_wrap.sv \
	$(picorv_dir)/src/id_stage.sv \
	$(picorv_dir)/src/instr_realign.sv \
	$(picorv_dir)/src/issue_read_operands.sv \
	$(picorv_dir)/src/issue_stage.sv \
	$(picorv_dir)/src/load_store_unit.sv \
	$(picorv_dir)/src/load_unit.sv \
	$(picorv_dir)/src/mmu.sv \
	$(picorv_dir)/src/multiplier.sv \
	$(picorv_dir)/src/mult.sv \
	$(picorv_dir)/src/perf_counters.sv \
	$(picorv_dir)/src/ptw.sv \
	$(picorv_dir)/src/re_name.sv \
	$(picorv_dir)/src/scoreboard.sv \
	$(picorv_dir)/src/serdiv.sv \
	$(picorv_dir)/src/store_buffer.sv \
	$(picorv_dir)/src/store_unit.sv \
	$(picorv_dir)/src/tlb.sv \
	$(picorv_dir)/src/fpu/src/fpnew_cast_multi.sv \
	$(picorv_dir)/src/fpu/src/fpnew_classifier.sv \
	$(picorv_dir)/src/fpu/src/fpnew_divsqrt_multi.sv \
	$(picorv_dir)/src/fpu/src/fpnew_fma_multi.sv \
	$(picorv_dir)/src/fpu/src/fpnew_fma.sv \
	$(picorv_dir)/src/fpu/src/fpnew_noncomp.sv \
	$(picorv_dir)/src/fpu/src/fpnew_opgroup_block.sv \
	$(picorv_dir)/src/fpu/src/fpnew_opgroup_fmt_slice.sv \
	$(picorv_dir)/src/fpu/src/fpnew_opgroup_multifmt_slice.sv \
	$(picorv_dir)/src/fpu/src/fpnew_rounding.sv \
	$(picorv_dir)/src/fpu/src/fpnew_top.sv \
	$(picorv_dir)/src/fpu/src/fpu_div_sqrt_mvp/hdl/control_mvp.sv \
	$(picorv_dir)/src/fpu/src/fpu_div_sqrt_mvp/hdl/div_sqrt_top_mvp.sv \
	$(picorv_dir)/src/fpu/src/fpu_div_sqrt_mvp/hdl/iteration_div_sqrt_mvp.sv \
	$(picorv_dir)/src/fpu/src/fpu_div_sqrt_mvp/hdl/norm_div_sqrt_mvp.sv \
	$(picorv_dir)/src/fpu/src/fpu_div_sqrt_mvp/hdl/nrbd_nrsc_mvp.sv \
	$(picorv_dir)/src/fpu/src/fpu_div_sqrt_mvp/hdl/preprocess_mvp.sv \
	$(picorv_dir)/src/frontend/bht.sv \
	$(picorv_dir)/src/frontend/btb.sv \
	$(picorv_dir)/src/frontend/frontend.sv \
	$(picorv_dir)/src/frontend/instr_queue.sv \
	$(picorv_dir)/src/frontend/instr_scan.sv \
	$(picorv_dir)/src/frontend/ras.sv \
	$(picorv_dir)/src/cache_subsystem/picorv_icache.sv \
	$(picorv_dir)/src/cache_subsystem/wt_dcache_wbuffer.sv \
	$(picorv_dir)/src/cache_subsystem/wt_dcache.sv \
	$(picorv_dir)/src/cache_subsystem/wt_dcache_missunit.sv \
	$(picorv_dir)/src/cache_subsystem/wt_dcache_mem.sv \
	$(picorv_dir)/src/cache_subsystem/wt_dcache_ctrl.sv \
	$(picorv_dir)/src/cache_subsystem/wt_cache_subsystem.sv \
	$(picorv_dir)/src/cache_subsystem/wt_axi_adapter.sv \
	$(picorv_dir)/src/axi_riscv_atomics/src/axi_res_tbl.sv \
	$(picorv_dir)/src/axi_riscv_atomics/src/axi_riscv_amos_alu.sv \
	$(picorv_dir)/src/axi_riscv_atomics/src/axi_riscv_amos.sv \
	$(picorv_dir)/src/axi_riscv_atomics/src/axi_riscv_atomics.sv \
	$(picorv_dir)/src/axi_riscv_atomics/src/axi_riscv_atomics_wrap.sv \
	$(picorv_dir)/src/axi_riscv_atomics/src/axi_riscv_lrsc.sv \
	$(picorv_dir)/src/common_cells/src/exp_backoff.sv \
	$(picorv_dir)/src/util/axi_master_connect.sv \
	$(picorv_dir)/src/util/sram.sv \
	$(picorv_dir)/src/fpga-support/rtl/SyncSpRamBeNx64.sv \
	$(picorv_dir)/src/common_cells/src/unread.sv \
	$(picorv_dir)/src/common_cells/src/stream_arbiter.sv \
	$(picorv_dir)/src/common_cells/src/stream_arbiter_flushable.sv \
	$(picorv_dir)/src/common_cells/src/fifo_v3.sv \
	$(picorv_dir)/src/common_cells/src/lzc.sv \
	$(picorv_dir)/src/common_cells/src/popcount.sv \
	$(picorv_dir)/src/common_cells/src/rr_arb_tree.sv \
	$(picorv_dir)/src/common_cells/src/lfsr_8bit.sv \
	$(picorv_dir)/src/common_cells/src/shift_reg.sv \
	$(picorv_dir)/src/pmp/src/pmp.sv \
	$(picorv_dir)/src/pmp/src/pmp_entry.sv

PicoRV_WRAPPER = \
	$(vsrc_dir)/PicoRVCoreBlackbox.sv

ALL_VSRCS = $(PicoRV_PKGS) $(PicoRV_VSRCS) $(PicoRV_WRAPPER)

#########################################################################################
# pre-process using verilator
#########################################################################################

lookup_dirs = $(shell find -L $(picorv_dir) -name target -prune -o -type d -print 2> /dev/null | grep '.*/\($(1)\)$$')
INC_DIR_NAMES ?= include inc
INC_DIRS ?= $(foreach dir_name,$(INC_DIR_NAMES),$(call lookup_dirs,$(dir_name)))

# these flags are specific to Chipyard
EXTRA_PREPROC_DEFINES ?=
PREPROC_DEFINES ?= \
	WT_DCACHE \
	DISABLE_TRACER \
	SRAM_NO_INIT \
	VERILATOR \
	$(EXTRA_PREPROC_DEFINES)

PREPROC_SCRIPT = $(base_dir)/scripts/insert-includes.py

$(PREPROC_VERILOG): $(ALL_VSRCS)
	mkdir -p $(dir $(PREPROC_VERILOG))
	$(foreach def,$(PREPROC_DEFINES),echo "\`define $(def)" >> def.v; )
	$(foreach def,$(PREPROC_DEFINES),echo "\`undef $(def)" >> undef.v; )
	cat def.v $(ALL_VSRCS) undef.v > combined.sv
	sed -i '/l15.tmp.h/d' combined.sv
	sed -i '/define.tmp.h/d' combined.sv
	$(PREPROC_SCRIPT) combined.sv $@ $(INC_DIRS)
	rm -rf combined.sv def.v undef.v

clean:
	rm -rf $(PREPROC_VERILOG)

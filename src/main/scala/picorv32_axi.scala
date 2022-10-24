//******************************************************************************
// Copyright (c) 2019 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// PicoRV Tile Wrapper
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package picorv

import chisel3._
import chisel3.experimental.IntParam
import chisel3.util._

import java.io.{File, PrintWriter}
import scala.io.Source

object MyUtils {
  def addBlackBoxDefines(src: String, dest: String, defines: Map[String, Any]) = {
    val writer = new PrintWriter(new File(dest))
    val source = Source.fromFile(src)
    for ((key, value) <- defines) {
      writer.println(s"`define $key $value")
    }
    source.getLines().foreach(line => writer.println(line))
    source.close()
    for ((key, _) <- defines) {
      writer.println(s"`undef $key")
    }
    writer.close()
    require(new File(dest).exists(), s"Cannot create file $dest!")
  }
}


trait PicoRVCoreIOMem extends Bundle {
  val mem_valid = Output(Bool())
  val mem_instr = Output(Bool())
  val mem_ready = Input(Bool())
  val mem_addr = Output(UInt(32.W))
  val mem_wdata = Output(UInt(32.W))
  val mem_wstrb = Output(UInt(4.W))
  val mem_rdata = Input(UInt(32.W))
  // Look-Ahead Interface
  val mem_la_read = Output(Bool())
  val mem_la_write = Output(Bool())
  val mem_la_addr = Output(UInt(32.W))
  val mem_la_wdata = Output(UInt(32.W))
  val mem_la_wstrb = Output(UInt(4.W))
}

trait PicoRVCoreIOPCPI extends Bundle {
  val pcpi_valid = Output(Bool())
  val pcpi_insn = Output(UInt(32.W))
  val pcpi_rs1 = Output(UInt(32.W))
  val pcpi_rs2 = Output(UInt(32.W))
  val pcpi_wr = Input(Bool())
  val pcpi_rd = Input(UInt(32.W))
  val pcpi_wait = Input(Bool())
  val pcpi_ready = Input(Bool())
}

trait PicoRVCoreIOIRQ extends Bundle {
  val irq = Input(UInt(32.W))
  val eoi = Output(UInt(32.W))
}

trait PicoRVCoreIOTrace extends Bundle {
  val trace_valid = Output(Bool())
  val trace_data = Output(UInt(36.W))
}

/**
 * for (genvar i = 0; i < ariane_pkg::NR_COMMIT_PORTS; ++i) begin : gen_tp_roll
      assign trace_o[(TRACEPORT_SZ*(i+1)/ariane_pkg::NR_COMMIT_PORTS)-1:(TRACEPORT_SZ*i/ariane_pkg::NR_COMMIT_PORTS)] = {
          tp_if[i].tval[39:0],
          tp_if[i].cause[7:0],
          tp_if[i].interrupt,
          tp_if[i].exception,
          { 1'b0, tp_if[i].priv[1:0] },
          tp_if[i].insn[31:0],
          tp_if[i].iaddr[39:0],
          tp_if[i].valid,
          ~tp_if[i].reset,
          tp_if[i].clock
      };
  end
 */
trait PicoRVCoreIOFormal extends Bundle {
  //  output        rvfi_valid,
  // 	output [63:0] rvfi_order,
  // 	output [31:0] rvfi_insn,
  // 	output        rvfi_trap,
  // 	output        rvfi_halt,
  // 	output        rvfi_intr,
  // 	output [ 4:0] rvfi_rs1_addr,
  // 	output [ 4:0] rvfi_rs2_addr,
  // 	output [31:0] rvfi_rs1_rdata,
  // 	output [31:0] rvfi_rs2_rdata,
  // 	output [ 4:0] rvfi_rd_addr,
  // 	output [31:0] rvfi_rd_wdata,
  // 	output [31:0] rvfi_pc_rdata,
  // 	output [31:0] rvfi_pc_wdata,
  // 	output [31:0] rvfi_mem_addr,
  // 	output [ 3:0] rvfi_mem_rmask,
  // 	output [ 3:0] rvfi_mem_wmask,
  // 	output [31:0] rvfi_mem_rdata,
  // 	output [31:0] rvfi_mem_wdata,
  val rvfi_valid = Output(Bool())
  val rvfi_order = Output(UInt(64.W))
  val rvfi_insn = Output(UInt(32.W))
  val rvfi_trap = Output(Bool())
  val rvfi_halt = Output(Bool())
  val rvfi_intr = Output(Bool())
  // val rvfi_mode = Output(UInt(2.W))
  // val rvfi_ixl = Output(UInt(2.W))
  val rvfi_rs1_addr = Output(UInt(5.W))
  val rvfi_rs2_addr = Output(UInt(5.W))
  val rvfi_rs1_rdata = Output(UInt(32.W))
  val rvfi_rs2_rdata = Output(UInt(32.W))
  val rvfi_rd_addr = Output(UInt(5.W))
  val rvfi_rd_wdata = Output(UInt(32.W))
  val rvfi_pc_rdata = Output(UInt(32.W))
  val rvfi_pc_wdata = Output(UInt(32.W))
  val rvfi_mem_addr = Output(UInt(32.W))
  val rvfi_mem_rmask = Output(UInt(4.W))
  val rvfi_mem_wmask = Output(UInt(4.W))
  val rvfi_mem_rdata = Output(UInt(32.W))
  val rvfi_mem_wdata = Output(UInt(32.W))

  // val rvfi_csr_mcycle_rmask = Output(UInt(64.W))
  // val rvfi_csr_mcycle_wmask = Output(UInt(64.W))
  // val rvfi_csr_mcycle_rdata = Output(UInt(64.W))
  // val rvfi_csr_mcycle_wdata = Output(UInt(64.W))
  //
  // val rvfi_csr_minstret_rmask = Output(UInt(64.W))
  // val rvfi_csr_minstret_wmask = Output(UInt(64.W))
  // val rvfi_csr_minstret_rdata = Output(UInt(64.W))
  // val rvfi_csr_minstret_wdata = Output(UInt(64.W))

  val trap = Output(Bool())
}

trait PicoRVCoreIOBase extends Bundle {
  val clk = Input(Clock())
  val resetn = Input(Bool())
}

trait PicoRVCoreIOAXIPorts extends Bundle {
  val mem_axi_awvalid = Output(Bool())
  val mem_axi_awready = Input(Bool())
  val mem_axi_awaddr = Output(UInt(32.W))
  val mem_axi_awprot = Output(UInt(3.W))
  val mem_axi_wvalid = Output(Bool())
  val mem_axi_wready = Input(Bool())
  val mem_axi_wdata = Output(UInt(32.W))
  val mem_axi_wstrb = Output(UInt(4.W))
  val mem_axi_bvalid = Input(Bool())
  val mem_axi_bready = Output(Bool())
  val mem_axi_arvalid = Output(Bool())
  val mem_axi_arready = Input(Bool())
  val mem_axi_araddr = Output(UInt(32.W))
  val mem_axi_arprot = Output(UInt(3.W))
  val mem_axi_rvalid = Input(Bool())
  val mem_axi_rready = Output(Bool())
  val mem_axi_rdata = Input(UInt(32.W))
}

class PicoRVCoreIO extends Bundle
  with PicoRVCoreIOBase
  with PicoRVCoreIOMem
  with PicoRVCoreIOPCPI
  with PicoRVCoreIOIRQ
  with PicoRVCoreIOFormal

class PicoRVCoreAXIIO extends Bundle
  with PicoRVCoreIOBase
  with PicoRVCoreIOAXIPorts
  with PicoRVCoreIOPCPI
  with PicoRVCoreIOIRQ
  with PicoRVCoreIOTrace
  with PicoRVCoreIOFormal

class picorv32_axi
(ENABLE_COUNTERS: Boolean = true,
 ENABLE_COUNTERS64: Boolean = true,
 ENABLE_REGS_16_31: Boolean = true,
 ENABLE_REGS_DUALPORT: Boolean = true,
 // LATCHED_MEM_RDATA: Boolean = false,   // used only in bare core without axi
 TWO_STAGE_SHIFT: Boolean = true,
 BARREL_SHIFTER: Boolean = false,
 TWO_CYCLE_COMPARE: Boolean = false,
 TWO_CYCLE_ALU: Boolean = false,
 COMPRESSED_ISA: Boolean = false,
 CATCH_MISALIGN: Boolean = true,
 CATCH_ILLINSN: Boolean = true,
 ENABLE_PCPI: Boolean = false,
 ENABLE_MUL: Boolean = false,
 ENABLE_FAST_MUL: Boolean = false,
 ENABLE_DIV: Boolean = false,
 ENABLE_IRQ: Boolean = false,
 ENABLE_IRQ_QREGS: Boolean = true,
 ENABLE_IRQ_TIMER: Boolean = true,
 ENABLE_TRACE: Boolean = false,
 REGS_INIT_ZERO: Boolean = false,
 MASKED_IRQ: BigInt = 0x00000000L,
 LATCHED_IRQ: BigInt = 0xffffffffL,
 PROGADDR_RESET: BigInt = 0x00000000L,
 PROGADDR_IRQ: BigInt = 0x00000010L,
 STACKADDR: BigInt = 0xffffffffL,
)
  extends BlackBox(
    Map(
      "ENABLE_COUNTERS" -> IntParam(if (ENABLE_COUNTERS) 1 else 0),
      "ENABLE_COUNTERS64" -> IntParam(if (ENABLE_COUNTERS64) 1 else 0),
      "ENABLE_REGS_16_31" -> IntParam(if (ENABLE_REGS_16_31) 1 else 0),
      "ENABLE_REGS_DUALPORT" -> IntParam(if (ENABLE_REGS_DUALPORT) 1 else 0),
      // "LATCHED_MEM_RDATA" -> IntParam(if (LATCHED_MEM_RDATA) 1 else 0),
      "TWO_STAGE_SHIFT" -> IntParam(if (TWO_STAGE_SHIFT) 1 else 0),
      "BARREL_SHIFTER" -> IntParam(if (BARREL_SHIFTER) 1 else 0),
      "TWO_CYCLE_COMPARE" -> IntParam(if (TWO_CYCLE_COMPARE) 1 else 0),
      "TWO_CYCLE_ALU" -> IntParam(if (TWO_CYCLE_ALU) 1 else 0),
      "COMPRESSED_ISA" -> IntParam(if (COMPRESSED_ISA) 1 else 0),
      "CATCH_MISALIGN" -> IntParam(if (CATCH_MISALIGN) 1 else 0),
      "CATCH_ILLINSN" -> IntParam(if (CATCH_ILLINSN) 1 else 0),
      "ENABLE_PCPI" -> IntParam(if (ENABLE_PCPI) 1 else 0),
      "ENABLE_MUL" -> IntParam(if (ENABLE_MUL) 1 else 0),
      "ENABLE_FAST_MUL" -> IntParam(if (ENABLE_FAST_MUL) 1 else 0),
      "ENABLE_DIV" -> IntParam(if (ENABLE_DIV) 1 else 0),
      "ENABLE_IRQ" -> IntParam(if (ENABLE_IRQ) 1 else 0),
      "ENABLE_IRQ_QREGS" -> IntParam(if (ENABLE_IRQ_QREGS) 1 else 0),
      "ENABLE_IRQ_TIMER" -> IntParam(if (ENABLE_IRQ_TIMER) 1 else 0),
      "ENABLE_TRACE" -> IntParam(if (ENABLE_TRACE) 1 else 0),
      "REGS_INIT_ZERO" -> IntParam(if (REGS_INIT_ZERO) 1 else 0),
      "MASKED_IRQ" -> IntParam(MASKED_IRQ),
      "MASKED_IRQ" -> IntParam(MASKED_IRQ),
      "LATCHED_IRQ" -> IntParam(LATCHED_IRQ),
      "PROGADDR_RESET" -> IntParam(PROGADDR_RESET),
      "PROGADDR_IRQ" -> IntParam(PROGADDR_IRQ),
      "STACKADDR" -> IntParam(STACKADDR)))
    with HasBlackBoxPath {
  // val io = IO(new PicoRVCoreIO)
  val io = IO(new PicoRVCoreAXIIO)
  val chipyardDir = System.getProperty("user.dir")
  val picorvVsrcDir = s"$chipyardDir/generators/picorv/src/main/resources/vsrc"
  val src = s"$picorvVsrcDir/picorv32/picorv32.v"
  val dest = src + ".preprocessed.v"
  MyUtils.addBlackBoxDefines(src, dest, Seq(
    "RISCV_FORMAL" -> ""
  ).toMap)
  addPath(dest)
}

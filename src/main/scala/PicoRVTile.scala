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
import freechips.rocketchip.amba.axi4._
import freechips.rocketchip.config._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.prci.ClockSinkParameters
import freechips.rocketchip.rocket._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util._

/**
 * parameter [ 0:0] ENABLE_COUNTERS = 1,
 * parameter [ 0:0] ENABLE_COUNTERS64 = 1,
 * parameter [ 0:0] ENABLE_REGS_16_31 = 1,
 * parameter [ 0:0] ENABLE_REGS_DUALPORT = 1,
 * parameter [ 0:0] LATCHED_MEM_RDATA = 0,
 * parameter [ 0:0] TWO_STAGE_SHIFT = 1,
 * parameter [ 0:0] BARREL_SHIFTER = 0,
 * parameter [ 0:0] TWO_CYCLE_COMPARE = 0,
 * parameter [ 0:0] TWO_CYCLE_ALU = 0,
 * parameter [ 0:0] COMPRESSED_ISA = 0,
 * parameter [ 0:0] CATCH_MISALIGN = 1,
 * parameter [ 0:0] CATCH_ILLINSN = 1,
 * parameter [ 0:0] ENABLE_PCPI = 0,
 * parameter [ 0:0] ENABLE_MUL = 0,
 * parameter [ 0:0] ENABLE_FAST_MUL = 0,
 * parameter [ 0:0] ENABLE_DIV = 0,
 * parameter [ 0:0] ENABLE_IRQ = 0,
 * parameter [ 0:0] ENABLE_IRQ_QREGS = 1,
 * parameter [ 0:0] ENABLE_IRQ_TIMER = 1,
 * parameter [ 0:0] ENABLE_TRACE = 0,
 * parameter [ 0:0] REGS_INIT_ZERO = 0,
 * parameter [31:0] MASKED_IRQ = 32'h 0000_0000,
 * parameter [31:0] LATCHED_IRQ = 32'h ffff_ffff,
 * parameter [31:0] PROGADDR_RESET = 32'h 0000_0000,
 * parameter [31:0] PROGADDR_IRQ = 32'h 0000_0010,
 * parameter [31:0] STACKADDR = 32'h ffff_ffff
 */
case class PicoRVCoreParams
(bootFreqHz: BigInt = BigInt(5000000),
 ENABLE_COUNTERS: Boolean = true,
 ENABLE_COUNTERS64: Boolean = true,
 ENABLE_REGS_16_31: Boolean = true,
 ENABLE_REGS_DUALPORT: Boolean = true,
 // LATCHED_MEM_RDATA: Boolean = false,
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
 ENABLE_TRACE: Boolean = true,
 REGS_INIT_ZERO: Boolean = false,
 MASKED_IRQ: BigInt = 0x00000000L,
 LATCHED_IRQ: BigInt = 0xffffffffL,
 // PROGADDR_RESET: BigInt = 0x10000L, // use bootrom
 PROGADDR_RESET: BigInt = 0x80000000L, // use ram directly
 // PROGADDR_RESET: BigInt = 0x00000000L,
 PROGADDR_IRQ: BigInt = 0x00000010L,
 STACKADDR: BigInt = 0xffffffffL,
) extends CoreParams {
  /* DO NOT CHANGE BELOW THIS */
  val useVM: Boolean = true
  val useHypervisor: Boolean = false
  val useUser: Boolean = true
  val useSupervisor: Boolean = false
  val useDebug: Boolean = true
  val useAtomics: Boolean = false
  val useAtomicsOnlyForIO: Boolean = false // copied from Rocket
  val useCompressed: Boolean = true
  override val useVector: Boolean = false
  val useSCIE: Boolean = false
  val useRVE: Boolean = false
  val mulDiv: Option[MulDivParams] = Some(MulDivParams()) // copied from Rocket
  val fpu: Option[FPUParams] = None // copied fma latencies from Rocket
  val nLocalInterrupts: Int = 0
  val useNMI: Boolean = false
  val nPMPs: Int = 0 // TODO: Check
  val pmpGranularity: Int = 4 // copied from Rocket
  val nBreakpoints: Int = 0 // TODO: Check
  val useBPWatch: Boolean = false
  val mcontextWidth: Int = 0 // TODO: Check
  val scontextWidth: Int = 0 // TODO: Check
  val nPerfCounters: Int = 29
  val haveBasicCounters: Boolean = true
  val haveFSDirty: Boolean = false
  val misaWritable: Boolean = false
  val haveCFlush: Boolean = false
  val nL2TLBEntries: Int = 32 // copied from Rocket
  val nL2TLBWays: Int = 1
  val mtvecInit: Option[BigInt] = Some(BigInt(0)) // copied from Rocket
  val mtvecWritable: Boolean = true // copied from Rocket
  val instBits: Int = if (useCompressed) 16 else 32
  val lrscCycles: Int = 80 // copied from Rocket
  val decodeWidth: Int = 1 // TODO: Check
  val fetchWidth: Int = 1 // TODO: Check
  val retireWidth: Int = 2
  val nPTECacheEntries: Int = 8 // TODO: Check
}

case class PicoRVTileAttachParams(
                                   tileParams: PicoRVTileParams,
                                   crossingParams: RocketCrossingParams
                                 ) extends CanAttachTile {
  type TileType = PicoRVTile
  val lookup = PriorityMuxHartIdFromSeq(Seq(tileParams))
}

// TODO: BTBParams, DCacheParams, ICacheParams are incorrect in DTB... figure out defaults in PicoRV and put in DTB
case class PicoRVTileParams
(name: Option[String] = Some("picorv_tile"),
 hartId: Int = 0,
 trace: Boolean = false,
 val core: PicoRVCoreParams = PicoRVCoreParams()
) extends InstantiableTileParams[PicoRVTile] {
  val beuAddr: Option[BigInt] = None
  val blockerCtrlAddr: Option[BigInt] = None
  val btb: Option[BTBParams] = Some(BTBParams())
  val boundaryBuffers: Boolean = false
  val dcache: Option[DCacheParams] = Some(DCacheParams())
  val icache: Option[ICacheParams] = Some(ICacheParams())
  val clockSinkParams: ClockSinkParameters = ClockSinkParameters()

  def instantiate(crossing: TileCrossingParamsLike, lookup: LookupByHartIdImpl)(implicit p: Parameters): PicoRVTile = {
    new PicoRVTile(this, crossing, lookup)
  }
}

class PicoRVTile private
(val picorvParams: PicoRVTileParams,
 crossing: ClockCrossingType,
 lookup: LookupByHartIdImpl,
 q: Parameters)
  extends BaseTile(picorvParams, crossing, lookup, q)
    with SinksExternalInterrupts
    with SourcesExternalNotifications {
  /**
   * Setup parameters:
   * Private constructor ensures altered LazyModule.p is used implicitly
   */
  def this(params: PicoRVTileParams, crossing: TileCrossingParamsLike, lookup: LookupByHartIdImpl)(implicit p: Parameters) =
    this(params, crossing.crossingType, lookup, p)

  val intOutwardNode = IntIdentityNode()
  val slaveNode = TLIdentityNode()
  val masterNode = visibilityNode

  tlOtherMastersNode := tlMasterXbar.node
  masterNode :=* tlOtherMastersNode
  DisableMonitors { implicit p => tlSlaveXbar.node :*= slaveNode }

  val cpuDevice: SimpleDevice = new SimpleDevice("cpu", Seq("openhwgroup,picorv", "riscv")) {
    override def parent = Some(ResourceAnchors.cpus)

    override def describe(resources: ResourceBindings): Description = {
      val Description(name, mapping) = super.describe(resources)
      Description(name, mapping ++
        cpuProperties ++
        nextLevelCacheProperty ++
        tileProperties)
    }
  }

  ResourceBinding {
    Resource(cpuDevice, "reg").bind(ResourceAddress(staticIdForMetadataUseOnly))
  }

  override def makeMasterBoundaryBuffers(crossing: ClockCrossingType)(implicit p: Parameters) = crossing match {
    case _: RationalCrossing =>
      if (!picorvParams.boundaryBuffers) TLBuffer(BufferParams.none)
      else TLBuffer(BufferParams.none, BufferParams.flow, BufferParams.none, BufferParams.flow, BufferParams(1))
    case _ => TLBuffer(BufferParams.none)
  }

  override def makeSlaveBoundaryBuffers(crossing: ClockCrossingType)(implicit p: Parameters) = crossing match {
    case _: RationalCrossing =>
      if (!picorvParams.boundaryBuffers) TLBuffer(BufferParams.none)
      else TLBuffer(BufferParams.flow, BufferParams.none, BufferParams.none, BufferParams.none, BufferParams.none)
    case _ => TLBuffer(BufferParams.none)
  }

  override lazy val module = new PicoRVTileModuleImp(this)

  /**
   * Setup AXI4 memory interface.
   * THESE ARE CONSTANTS.
   */
  val portName = "picorv-mem-port-axi4"
  val idBits = 4
  val beatBytes = masterPortBeatBytes
  val sourceBits = 1 // equiv. to userBits (i think)

  val memAXI4Node = AXI4MasterNode(
    Seq(AXI4MasterPortParameters(
      masters = Seq(AXI4MasterParameters(
        name = portName,
        id = IdRange(0, 1 << idBits))))))

  val memoryTap = TLIdentityNode()
  (tlMasterXbar.node
    := memoryTap
    := TLBuffer()
    := TLFIFOFixer(TLFIFOFixer.all) // fix FIFO ordering
    := TLWidthWidget(beatBytes) // reduce size of TL
    := AXI4ToTL() // convert to TL
    := AXI4UserYanker(Some(2)) // remove user field on AXI interface. need but in reality user intf. not needed
    := AXI4Fragmenter() // deal with multi-beat xacts
    := memAXI4Node)

  def connectPicoRVInterrupts(irq: UInt) {
    val (interrupts, _) = intSinkNode.in(0)
    // debug := interrupts(0)
    // msip := interrupts(1)
    // mtip := interrupts(2)
    // m_s_eip := Cat(interrupts(4), interrupts(3))
    irq := interrupts.asUInt
  }
}

class PicoRVTileModuleImp(outer: PicoRVTile) extends BaseTileModuleImp(outer) {
  // annotate the parameters
  Annotated.params(this, outer.picorvParams)

  val debugBaseAddr = BigInt(0x0) // CONSTANT: based on default debug module
  val debugSz = BigInt(0x1000) // CONSTANT: based on default debug module
  val tohostAddr = BigInt(0x80001000L) // CONSTANT: based on default sw (assume within extMem region)
  val fromhostAddr = BigInt(0x80001040L) // CONSTANT: based on default sw (assume within extMem region)

  // have the main memory, bootrom, debug regions be executable
  val bootromParams = p(BootROMLocated(InSubsystem)).get
  val executeRegionBases = Seq(p(ExtMem).get.master.base, bootromParams.address, debugBaseAddr, BigInt(0x0), BigInt(0x0))
  val executeRegionSzs = Seq(p(ExtMem).get.master.size, BigInt(bootromParams.size), debugSz, BigInt(0x0), BigInt(0x0))
  val executeRegionCnt = executeRegionBases.length

  // have the main memory be cached, but don't cache tohost/fromhost addresses
  // TODO: current cache subsystem can only support 1 cacheable region... so cache AFTER the tohost/fromhost addresses
  val wordOffset = 0x40
  val (cacheableRegionBases, cacheableRegionSzs) = if (true /* outer.picorvParams.core.enableToFromHostCaching */ ) {
    val bases = Seq(p(ExtMem).get.master.base, BigInt(0x0), BigInt(0x0), BigInt(0x0), BigInt(0x0))
    val sizes = Seq(p(ExtMem).get.master.size, BigInt(0x0), BigInt(0x0), BigInt(0x0), BigInt(0x0))
    (bases, sizes)
  } else {
    val bases = Seq(fromhostAddr + 0x40, p(ExtMem).get.master.base, BigInt(0x0), BigInt(0x0), BigInt(0x0))
    val sizes = Seq(p(ExtMem).get.master.size - (fromhostAddr + 0x40 - p(ExtMem).get.master.base), tohostAddr - p(ExtMem).get.master.base, BigInt(0x0), BigInt(0x0), BigInt(0x0))
    (bases, sizes)
  }
  val cacheableRegionCnt = cacheableRegionBases.length

  // Add 2 to account for the extra clock and reset included with each
  // instruction in the original trace port implementation. These have since
  // been removed from TracedInstruction.
  val traceInstSz = (new freechips.rocketchip.rocket.TracedInstruction).getWidth + 2

  // connect the picorv core
  val core = Module(new picorv32_axi(
    ENABLE_COUNTERS = outer.picorvParams.core.ENABLE_COUNTERS,
    ENABLE_COUNTERS64 = outer.picorvParams.core.ENABLE_COUNTERS64,
    ENABLE_REGS_16_31 = outer.picorvParams.core.ENABLE_REGS_16_31,
    ENABLE_REGS_DUALPORT = outer.picorvParams.core.ENABLE_REGS_DUALPORT,
    // LATCHED_MEM_RDATA = outer.picorvParams.core.LATCHED_MEM_RDATA,
    TWO_STAGE_SHIFT = outer.picorvParams.core.TWO_STAGE_SHIFT,
    BARREL_SHIFTER = outer.picorvParams.core.BARREL_SHIFTER,
    TWO_CYCLE_COMPARE = outer.picorvParams.core.TWO_CYCLE_COMPARE,
    TWO_CYCLE_ALU = outer.picorvParams.core.TWO_CYCLE_ALU,
    COMPRESSED_ISA = outer.picorvParams.core.COMPRESSED_ISA,
    CATCH_MISALIGN = outer.picorvParams.core.CATCH_MISALIGN,
    CATCH_ILLINSN = outer.picorvParams.core.CATCH_ILLINSN,
    ENABLE_PCPI = outer.picorvParams.core.ENABLE_PCPI,
    ENABLE_MUL = outer.picorvParams.core.ENABLE_MUL,
    ENABLE_FAST_MUL = outer.picorvParams.core.ENABLE_FAST_MUL,
    ENABLE_DIV = outer.picorvParams.core.ENABLE_DIV,
    ENABLE_IRQ = outer.picorvParams.core.ENABLE_IRQ,
    ENABLE_IRQ_QREGS = outer.picorvParams.core.ENABLE_IRQ_QREGS,
    ENABLE_IRQ_TIMER = outer.picorvParams.core.ENABLE_IRQ_TIMER,
    ENABLE_TRACE = outer.picorvParams.core.ENABLE_TRACE,
    REGS_INIT_ZERO = outer.picorvParams.core.REGS_INIT_ZERO,
    MASKED_IRQ = outer.picorvParams.core.MASKED_IRQ,
    LATCHED_IRQ = outer.picorvParams.core.LATCHED_IRQ,
    PROGADDR_RESET = outer.picorvParams.core.PROGADDR_RESET,
    PROGADDR_IRQ = outer.picorvParams.core.PROGADDR_IRQ,
    STACKADDR = outer.picorvParams.core.STACKADDR,
  )).suggestName("picorv32_core_inst")

  core.io.clk := clock
  core.io.resetn := ~reset.asBool
  // core.io.boot_addr_i := outer.resetVectorSinkNode.bundle
  // core.io.hart_id_i := outer.hartIdSinkNode.bundle

  // TODO: connect interrupt
  outer.connectPicoRVInterrupts(core.io.irq)

  if (outer.picorvParams.trace) {
    // unpack the trace io from a UInt into Vec(TracedInstructions)
    //outer.traceSourceNode.bundle <> core.io.trace_o.asTypeOf(outer.traceSourceNode.bundle)

    for (w <- 0 until outer.picorvParams.core.retireWidth) {
      // outer.traceSourceNode.bundle(w).valid := core.io.trace_o(traceInstSz * w + 2)
      // outer.traceSourceNode.bundle(w).iaddr := core.io.trace_o(traceInstSz * w + 42, traceInstSz * w + 3)
      // outer.traceSourceNode.bundle(w).insn := core.io.trace_o(traceInstSz * w + 74, traceInstSz * w + 43)
      // outer.traceSourceNode.bundle(w).priv := core.io.trace_o(traceInstSz * w + 77, traceInstSz * w + 75)
      // outer.traceSourceNode.bundle(w).exception := core.io.trace_o(traceInstSz * w + 78)
      // outer.traceSourceNode.bundle(w).interrupt := core.io.trace_o(traceInstSz * w + 79)
      // outer.traceSourceNode.bundle(w).cause := core.io.trace_o(traceInstSz * w + 87, traceInstSz * w + 80)
      // outer.traceSourceNode.bundle(w).tval := core.io.trace_o(traceInstSz * w + 127, traceInstSz * w + 88)
      outer.traceSourceNode.bundle(w).valid := core.io.rvfi_valid
      outer.traceSourceNode.bundle(w).iaddr := core.io.rvfi_pc_rdata
      outer.traceSourceNode.bundle(w).insn := core.io.rvfi_insn
      outer.traceSourceNode.bundle(w).priv := 3.U
      outer.traceSourceNode.bundle(w).exception := 0.U
      outer.traceSourceNode.bundle(w).interrupt := core.io.rvfi_intr
      outer.traceSourceNode.bundle(w).cause := 0.U
      outer.traceSourceNode.bundle(w).tval := 0.U
    }
    // TODO: add tracer
  } else {
    outer.traceSourceNode.bundle := DontCare
    outer.traceSourceNode.bundle map (t => t.valid := false.B)
  }

  // connect the axi interface
  outer.memAXI4Node.out foreach { case (out, edgeOut) =>
    // connect AXI <==> AXI Lite
    core.io.mem_axi_awready := out.aw.ready
    out.aw.valid := core.io.mem_axi_awvalid
    // set id to 0
    out.aw.bits.id := 0.U
    out.aw.bits.addr := core.io.mem_axi_awaddr
    // aw len is 0, burst for 1
    out.aw.bits.len := 0.U
    // aw size is "b010", every transfer is 4 bytes
    out.aw.bits.size := "b010".U
    // only support burst=INCR
    out.aw.bits.burst := "b01".U
    // lock: Normal access
    out.aw.bits.lock := "b00".U
    // cache: No buffered
    out.aw.bits.cache := "b0000".U
    out.aw.bits.prot := core.io.mem_axi_awprot
    out.aw.bits.qos := "b0000".U

    core.io.mem_axi_wready := out.w.ready
    out.w.valid := core.io.mem_axi_wvalid
    out.w.bits.data := core.io.mem_axi_wdata
    out.w.bits.strb := core.io.mem_axi_wstrb
    // last is 1, use lite
    out.w.bits.last := true.B

    out.b.ready := core.io.mem_axi_bready
    core.io.mem_axi_bvalid := out.b.valid
    // core.io.axi_resp_i_b_bits_id := out.b.bits.id
    // core.io.axi_resp_i_b_bits_resp := out.b.bits.resp

    core.io.mem_axi_arready := out.ar.ready
    out.ar.valid := core.io.mem_axi_arvalid
    out.ar.bits.id := 0.U
    out.ar.bits.addr := core.io.mem_axi_awaddr
    out.ar.bits.len := 0.U
    out.ar.bits.size := "b010".U
    out.ar.bits.burst := "b01".U
    out.ar.bits.lock := "b00".U
    out.ar.bits.cache := "b0000".U
    out.ar.bits.prot := core.io.mem_axi_arprot
    out.ar.bits.qos := "b0000".U

    out.r.ready := core.io.mem_axi_rready
    core.io.mem_axi_rvalid := out.r.valid
    // core.io.axi_resp_i_r_bits_id := out.r.bits.id
    core.io.mem_axi_rdata := out.r.bits.data
    // core.io.axi_resp_i_r_bits_resp := out.r.bits.resp
    // core.io.axi_resp_i_r_bits_last := out.r.bits.last
  }
}

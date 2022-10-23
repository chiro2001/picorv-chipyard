//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

package picorv

import chisel3._
import chisel3.util.{log2Up}

import freechips.rocketchip.config.{Parameters, Config, Field}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.devices.tilelink.{BootROMParams}
import freechips.rocketchip.diplomacy.{SynchronousCrossing, AsynchronousCrossing, RationalCrossing}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._

/**
 * Makes cacheable region include to/from host addresses.
 * Speeds up operation... at the expense of not being able to use
 * to/fromhost communication unless those lines are evicted from L1.
 */
class WithToFromHostCaching extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: PicoRVTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(core = tp.tileParams.core.copy(
      enableToFromHostCaching = true
    )))
  }
})

/**
 * Create multiple copies of a PicoRV tile (and thus a core).
 * Override with the default mixins to control all params of the tiles.
 *
 * @param n amount of tiles to duplicate
 */
class WithNPicoRVCores(n: Int = 1, overrideIdOffset: Option[Int] = None) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => {
    val prev = up(TilesLocated(InSubsystem), site)
    val idOffset = overrideIdOffset.getOrElse(prev.size)
    (0 until n).map { i =>
      PicoRVTileAttachParams(
        tileParams = PicoRVTileParams(hartId = i + idOffset),
        crossingParams = RocketCrossingParams()
      )
    } ++ prev
  }
  case SystemBusKey => up(SystemBusKey, site).copy(beatBytes = 8)
  case XLen => 64
})


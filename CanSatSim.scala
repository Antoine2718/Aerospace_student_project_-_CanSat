// CanSatSim.scala
// Scala 2.12+ or 2.13 script
import scala.math._
import scala.util.Random

object CanSatSim extends App {
  // Parameters (changeable)
  val battCapacity_mAh = if (args.length>0) args(0).toDouble else 1000.0 // mAh
  val steps = 3600 // seconds to simulate
  val dt = 1.0 // seconds per step
  val seed = 42
  val rand = new Random(seed)

  // Per-module currents (mA) approximate
  val MCU_sleep = 1.5
  val MCU_active = 8.0
  val BMP280 = 0.01
  val IMU_sleep = 0.1
  val IMU_active = 4.0
  val GPS_on = 30.0
  val GPS_off = 0.01
  val SD_idle = 5.0
  val SD_write_peak = 150.0
  val LoRa_rx = 12.0
  val LoRa_sleep = 0.5
  val LoRa_tx_peak = 120.0

  // Mode duty cycles: fraction of time spent active (0..1)
  case class Mode(name:String, mcuActivity:Double, imuActivity:Double, gpsOn:Boolean, sdWritePerSec:Double, loraRx:Boolean, loraTxPerSec:Double)
  val STARTUP = Mode("STARTUP", 0.8, 1.0, true, 1.0, true, 0.1)
  val STANDBY = Mode("STANDBY", 0.05, 0.01, false, 0.01, false, 0.001)
  val FLIGHT  = Mode("FLIGHT", 0.8, 1.0, true, 5.0, true, 1.0)
  val RECOVERY = Mode("RECOVERY", 0.1, 0.05, false, 0.1, true, 0.2)

  // Scenario timeline (seconds)
  // 0-300s STARTUP, 300-3600 STANDBY, flight 4200-4260 etc; for demo we'll do a simple scenario
  def modeAt(t:Int): Mode = {
    if (t < 60) STARTUP
    else if (t < 10800) STANDBY // many hours
    else FLIGHT
  }

  var remaining_mAh = battCapacity_mAh
  var t = 0
  var rtc = 0
  var lastPrint = -1
  while (t < steps && remaining_mAh > 0) {
    val mode = modeAt(t)
    // compute instantaneous currents
    val mcu = MCU_sleep*(1-mode.mcuActivity) + MCU_active*mode.mcuActivity
    val imu = IMU_sleep*(1-mode.imuActivity) + IMU_active*mode.imuActivity
    val gps = if (mode.gpsOn) GPS_on else GPS_off
    val sd = SD_idle + (SD_write_peak * mode.sdWritePerSec * dt) // average add of writes per sec
    val radio = if (mode.loraRx) LoRa_rx else LoRa_sleep
    // add occasional TX peaks averaged
    val radioAvg = radio + (LoRa_tx_peak * mode.loraTxPerSec * dt)

    val total_mA = mcu + imu + gps + sd + radioAvg + BMP280
    // subtract from battery (mAh)
    remaining_mAh -= (total_mA * dt / 3600.0) * 1000.0 / 1000.0 // simplify -> mAh decrement
    // print periodic
    if (t % 60 == 0) {
      println(f"t=${t}s mode=${mode.name} total=${total_mA}%.1f mA remaining=${remaining_mAh}%.1f mAh")
    }
    t += 1
  }
  if (remaining_mAh <= 0) println(s"Battery exhausted at t=${t}s")
  else println(s"Simulation ended at t=${t}s with remaining=${remaining_mAh} mAh")
}

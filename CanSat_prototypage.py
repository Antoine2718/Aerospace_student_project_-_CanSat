"""
CanSat simulator (functional style)

Features:
 - Battery model (Li-ion 1S nominal 3.8 V; full 4.2 V). SoC -> voltage mapping.
 - Modules: MCU, BMP280, IMU, MAG, GPS, LoRa (SX127x / RFM69), microSD
 - Modes: STARTUP, STANDBY (ground test), FLIGHT, RECOVERY
 - Scheduler: cooperative tick-based scheduler (tick_ms)
 - Sensor sampling via periodic tasks (imitates interrupts)
 - microSD circular buffer + journaling + grouped writes (flush threshold)
 - Telemetry with CRC16, limited retransmission
 - Watchdog + brown-out + EEPROM snapshot before power-down
 - Power management strategies: sleep durations, duty-cycle, grouped writes
 - Deterministic randomness (seeded) for sensor noise/failures
 - Simulation run scenario: ground test (3 h) then short flight (5 minutes)
"""

from dataclasses import dataclass, replace, field
from typing import Callable, Dict, List, Tuple, Optional
import math
import random
import itertools
import time
import binascii

# --- Utility functions (pure) -----------------------------------------------

def crc16_ccitt(data: bytes, poly: int = 0x1021, init: int = 0xFFFF) -> int:
    """CRC-16-CCITT (pure python)."""
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if (crc & 0x8000):
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

def now_human(ms: int) -> str:
    s = ms // 1000
    h = s // 3600
    m = (s % 3600) // 60
    sec = s % 60
    return f"{h}h{m:02d}m{sec:02d}s"

# --- Types ------------------------------------------------------------------

TickFn = Callable[['SystemState'], 'SystemState']  # a task takes state and returns new state

@dataclass(frozen=True)
class ModuleConfig:
    name: str
    active_current_ma: float  # mA when active
    sleep_current_ma: float   # mA when in low-power
    duty_cycle: float = 0.0   # fraction active when in duty-cycled mode (0..1)

@dataclass(frozen=True)
class BatteryModel:
    capacity_mah: float                    # capacity in mAh
    voltage_full: float = 4.2
    voltage_cutoff: float = 3.0
    nominal_voltage: float = 3.8           # user-provided nominal
    soc: float = 1.0                        # state-of-charge 0..1 (1==full)
    # linear map from SoC to voltage by default; could be replaced by OCV curve
    def voltage(self) -> float:
        # linear mapping between cutoff and full
        return self.voltage_cutoff + (self.voltage_full - self.voltage_cutoff) * self.soc

    def consume_ma_ms(self, current_ma: float, dt_ms: int) -> 'BatteryModel':
        """Return new BatteryModel after consuming current_ma for dt_ms."""
        # convert ma * ms -> mAh consumed
        hours = dt_ms / 1000.0 / 3600.0
        consumed_mah = current_ma * hours
        new_soc = max(0.0, self.soc - (consumed_mah / self.capacity_mah))
        return replace(self, soc=new_soc)

@dataclass(frozen=True)
class MicroSD:
    buffer: Tuple[bytes, ...] = field(default_factory=tuple)   # in-memory circular buffer of records
    buffered_bytes: int = 0
    flush_threshold_bytes: int = 4096  # group writes: flush when exceed
    journal_open: bool = False         # journaling guard
    storage: Tuple[bytes, ...] = field(default_factory=tuple) # stored blocks (simulate writes)

    def append_record(self, rec: bytes) -> 'MicroSD':
        new_buf = self.buffer + (rec,)
        return replace(self, buffer=new_buf, buffered_bytes=self.buffered_bytes + len(rec))

    def should_flush(self) -> bool:
        return self.buffered_bytes >= self.flush_threshold_bytes

    def flush(self) -> 'MicroSD':
        """Simulate journaling: write a journal entry then commit blocks."""
        if not self.buffer:
            return self
        # create a journal record then commit the block
        journal = b"JOURNAL_START"
        committed = self.storage + (journal,) + self.buffer + (b"JOURNAL_END",)
        # clear buffer
        return replace(self, buffer=tuple(), buffered_bytes=0, journal_open=False, storage=committed)

@dataclass(frozen=True)
class EEPROM:
    last_snapshot: Optional[Dict] = None

    def snapshot(self, state_snapshot: Dict) -> 'EEPROM':
        return replace(self, last_snapshot=state_snapshot)

@dataclass(frozen=True)
class TelemetryState:
    tx_queue: Tuple[bytes, ...] = field(default_factory=tuple)
    tx_attempts: Dict[bytes, int] = field(default_factory=dict)  # payload->attempts

    def enqueue(self, packet: bytes) -> 'TelemetryState':
        new_q = self.tx_queue + (packet,)
        new_attempts = dict(self.tx_attempts)
        new_attempts[packet] = 0
        return replace(self, tx_queue=new_q, tx_attempts=new_attempts)

    def dequeue(self) -> 'TelemetryState':
        if not self.tx_queue:
            return self
        new_q = self.tx_queue[1:]
        new_attempts = dict(self.tx_attempts)
        removed = self.tx_queue[0]
        new_attempts.pop(removed, None)
        return replace(self, tx_queue=new_q, tx_attempts=new_attempts)

@dataclass(frozen=True)
class RadioConfig:
    protocol: str  # "LoRa" or "RFM69"
    tx_power_dbm: int
    tx_current_ma: float
    rx_current_ma: float
    sleep_current_ma: float
    tx_duration_ms_per_packet: int

@dataclass(frozen=True)
class SystemState:
    t_ms: int
    mode: str  # STARTUP, STANDBY, FLIGHT, RECOVERY
    battery: BatteryModel
    modules: Dict[str, ModuleConfig]
    microSD: MicroSD
    eeprom: EEPROM
    telem: TelemetryState
    radio: RadioConfig
    watchdog_counter_ms: int
    brownout_threshold_v: float
    logs: Tuple[str, ...]  # console-like logs to inspect simulation

    def log(self, msg: str) -> 'SystemState':
        new_logs = self.logs + (f"[{now_human(self.t_ms)}] {msg}",)
        return replace(self, logs=new_logs)

# --- Defaults / configuration ----------------------------------------------

DEFAULT_MODULES = {
    'MCU': ModuleConfig('MCU', active_current_ma=12.0, sleep_current_ma=2.0),
    'BMP280': ModuleConfig('BMP280', active_current_ma=0.8, sleep_current_ma=0.1),
    'IMU': ModuleConfig('IMU', active_current_ma=3.6, sleep_current_ma=0.01),
    'MAG': ModuleConfig('MAG', active_current_ma=1.0, sleep_current_ma=0.01),
    'GPS': ModuleConfig('GPS', active_current_ma=30.0, sleep_current_ma=0.5, duty_cycle=0.0),
    'LoRa': ModuleConfig('LoRa', active_current_ma=120.0, sleep_current_ma=1.0, duty_cycle=0.0),
    'microSD': ModuleConfig('microSD', active_current_ma=50.0, sleep_current_ma=0.05),
}

DEFAULT_RADIO = RadioConfig(protocol="LoRa", tx_power_dbm=14, tx_current_ma=120.0, rx_current_ma=18.0,
                           sleep_current_ma=1.0, tx_duration_ms_per_packet=100)

# --- Sensor simulations (pure-ish) -----------------------------------------

def sample_bmp280(state: SystemState) -> Tuple[SystemState, Dict]:
    """Return new state + sensor reading dict. Pure (no random side-effect outside deterministic RNG)."""
    t = state.t_ms
    # altitude varies slowly, pressure derived: P ~ 101325 * exp(-h/H), simple model
    base_alt_m = 100.0  # e.g., launch altitude
    # in flight, we may add a bump; read altitude from logs? We'll just add small oscillation
    altitude = base_alt_m + 5.0 * math.sin(2 * math.pi * (t / 60000.0))
    pressure = 101325.0 * math.exp(-altitude / 8400.0)
    temp_c = 20.0 + 0.01 * (t / 1000.0) % 5  # small drift
    reading = {'altitude_m': altitude, 'pressure_pa': pressure, 'temp_c': temp_c}
    state = state.log(f"BMP280 sampled: alt={altitude:.1f}m p={pressure:.1f}Pa t={temp_c:.2f}C")
    return state, reading

def sample_imu(state: SystemState) -> Tuple[SystemState, Dict]:
    # produce accelerometer + gyro with noise
    t = state.t_ms
    ax = 0.0 + 0.02 * math.sin(2*math.pi*(t/100.0)) + random.gauss(0, 0.01)
    ay = 0.0 + 0.02 * math.cos(2*math.pi*(t/100.0)) + random.gauss(0, 0.01)
    az = 9.81 + random.gauss(0, 0.05)
    gx = 0.001 * math.sin(t / 500.0) + random.gauss(0, 0.0005)
    gy = 0.001 * math.cos(t / 500.0) + random.gauss(0, 0.0005)
    gz = random.gauss(0, 0.0002)
    reading = {'accel': (ax, ay, az), 'gyro': (gx, gy, gz)}
    state = state.log(f"IMU sampled: a=({ax:.3f},{ay:.3f},{az:.3f})")
    return state, reading

def sample_mag(state: SystemState) -> Tuple[SystemState, Dict]:
    # simple geomagnetic reading with noise
    bx = 20.0 + random.gauss(0, 0.1)
    by = 5.0 + random.gauss(0, 0.1)
    bz = -40.0 + random.gauss(0, 0.1)
    reading = {'mag': (bx, by, bz)}
    state = state.log(f"MAG sampled: b=({bx:.2f},{by:.2f},{bz:.2f})")
    return state, reading

def sample_gps(state: SystemState) -> Tuple[SystemState, Dict]:
    # Simulated NMEA-like fix with time/lat/lon/alt; in ground tests GPS may be off
    base_lat, base_lon = 51.5074, -0.1278  # example coords (London)
    drift = 0.00001 * (state.t_ms / 1000.0)
    lat = base_lat + drift
    lon = base_lon + drift/2
    alt = 100.0 + 10.0 * math.sin(state.t_ms / 10000.0)
    fix = {'lat': lat, 'lon': lon, 'alt': alt, 'fix': True}
    state = state.log(f"GPS sampled: lat={lat:.6f} lon={lon:.6f} alt={alt:.1f}")
    return state, fix

# --- Telemetry framing -----------------------------------------------------

def make_telem_packet(state: SystemState, payload: bytes) -> bytes:
    """Build a telemetry packet: [timestamp_ms(4)][payload_len(2)][payload][crc16(2)]"""
    ts = state.t_ms & 0xFFFFFFFF
    header = ts.to_bytes(4, 'big') + len(payload).to_bytes(2, 'big')
    pkt_wo_crc = header + payload
    crc = crc16_ccitt(pkt_wo_crc)
    pkt = pkt_wo_crc + crc.to_bytes(2, 'big')
    return pkt

# --- Scheduler (cooperative tick) ------------------------------------------

@dataclass(frozen=True)
class Task:
    name: str
    interval_ms: int
    next_run_ms: int
    fn: TickFn

@dataclass(frozen=True)
class Scheduler:
    tasks: Tuple[Task, ...]
    tick_ms: int

    def tick(self, state: SystemState) -> SystemState:
        t = state.t_ms
        new_tasks = []
        s = state
        for task in self.tasks:
            if t >= task.next_run_ms:
                # execute
                s = task.fn(s)
                # schedule next run
                next_run = t + task.interval_ms
                new_tasks.append(replace(task, next_run_ms=next_run))
            else:
                new_tasks.append(task)
        return replace(s)  # tasks updated elsewhere if needed

    def advance_tasks(self) -> 'Scheduler':
        # helper not used much; keeping pure
        return self

# --- Power accounting (pure) -----------------------------------------------

def compute_module_current(state: SystemState, module_name: str) -> float:
    """Compute instantaneous current (mA) consumed by a module depending on mode & duty cycle."""
    cfg = state.modules[module_name]
    # rules:
    # - MCU always active when scheduling (but may be in sleep between tasks in STANDBY)
    # - GPS active only in FLIGHT (unless forced)
    # - LoRa duty-cycled for transmissions, RX mode consumes rx_current when in recovery
    mode = state.mode
    if module_name == 'MCU':
        if mode == 'STANDBY':
            return cfg.sleep_current_ma
        else:
            return cfg.active_current_ma
    if module_name == 'GPS':
        # active in FLIGHT, otherwise sleep
        if mode == 'FLIGHT':
            return cfg.active_current_ma * max(1.0, cfg.duty_cycle)  # assume constantly on in flight
        else:
            return cfg.sleep_current_ma
    if module_name == 'LoRa':
        # LoRa: in FLIGHT/RECOVERY might be RX or TX occasionally
        if mode == 'RECOVERY':
            # RX duty
            return cfg.active_current_ma * 0.5 + cfg.sleep_current_ma * 0.5
        if mode == 'FLIGHT':
            # mostly sleep, TX bursts
            return cfg.sleep_current_ma + cfg.active_current_ma * cfg.duty_cycle
        else:
            return cfg.sleep_current_ma
    if module_name == 'microSD':
        # active when buffer flush occurs
        return cfg.active_current_ma if state.microSD.buffered_bytes > 0 else cfg.sleep_current_ma
    # sensors: active in FLIGHT/STANDBY sampling, else sleep
    if module_name in ('BMP280', 'IMU', 'MAG'):
        if mode in ('STANDBY', 'FLIGHT'):
            return cfg.active_current_ma
        else:
            return cfg.sleep_current_ma
    return cfg.sleep_current_ma

def total_current_ma(state: SystemState) -> float:
    return sum(compute_module_current(state, name) for name in state.modules.keys())

# --- microSD operations (pure-ish) -----------------------------------------

def sd_append_log(state: SystemState, rec: bytes) -> SystemState:
    sd = state.microSD.append_record(rec)
    s = replace(state, microSD=sd)
    s = s.log(f"Appended {len(rec)} bytes to microSD buffer (buf={sd.buffered_bytes})")
    # flush if threshold reached
    if sd.should_flush():
        sd2 = sd.flush()
        s = replace(s, microSD=sd2)
        s = s.log("microSD flush (grouped write) completed")
    return s

def sd_force_flush(state: SystemState) -> SystemState:
    sd = state.microSD.flush()
    s = replace(state, microSD=sd)
    s = s.log("microSD forced flush (journal+commit)")
    return s

# --- Watchdog / brownout / EEPROM snapshot ---------------------------------

def check_brownout_and_watchdog(state: SystemState) -> SystemState:
    v = state.battery.voltage()
    s = state
    if v <= state.brownout_threshold_v:
        s = s.log(f"Brownout triggered: V={v:.3f} <= threshold {state.brownout_threshold_v:.3f}")
        # attempt snapshot to EEPROM before losing power
        snapshot = {
            't_ms': s.t_ms,
            'mode': s.mode,
            'battery_v': v,
            'microSD_buffered': s.microSD.buffered_bytes
        }
        eeprom2 = s.eeprom.snapshot(snapshot)
        s = replace(s, eeprom=eeprom2)
        s = s.log("EEPROM snapshot stored before power-down")
        # flush microSD journal aggressively
        s = sd_force_flush(s)
        # simulate immediate power-down by setting SoC to 0
        battery2 = replace(s.battery, soc=0.0)
        s = replace(s, battery=battery2)
        return s
    # watchdog: if no reset from tasks -> increment and possibly reset
    wd = state.watchdog_counter_ms + 1
    if wd > 5000:  # 5s cause reset if not kicked
        s = s.log("Watchdog timeout -> soft reset")
        # soft reset: store snapshot + reset state to STARTUP
        snapshot = {
            't_ms': s.t_ms,
            'mode': s.mode,
            'battery_v': s.battery.voltage()
        }
        eeprom2 = s.eeprom.snapshot(snapshot)
        s = replace(s, eeprom=eeprom2, mode='STARTUP', watchdog_counter_ms=0)
        s = s.log("System soft-reset performed; mode=STARTUP")
        return s
    return replace(s, watchdog_counter_ms=wd)

def kick_watchdog(state: SystemState) -> SystemState:
    return replace(state, watchdog_counter_ms=0)

# --- Telemetry (pure-ish) --------------------------------------------------

def prepare_health_frame(state: SystemState) -> Tuple[SystemState, bytes]:
    payload = f"HEALTH;V={state.battery.voltage():.3f};mode={state.mode};buf={state.microSD.buffered_bytes}".encode()
    pkt = make_telem_packet(state, payload)
    s = state.log("Prepared health telemetry frame")
    s = replace(s, telem=state.telem.enqueue(pkt))
    return s, pkt

def tx_next_packet(state: SystemState) -> SystemState:
    if not state.telem.tx_queue:
        return state
    pkt = state.telem.tx_queue[0]
    attempts = state.telem.tx_attempts.get(pkt, 0)
    if attempts >= 3:
        state = state.log("Telemetry packet dropped after 3 attempts")
        state = replace(state, telem=state.telem.dequeue())
        return state
    # simulate TX cost/time/ack: assume successful with p=0.9
    success = random.random() < 0.9
    te = dict(state.telem.tx_attempts)
    te[pkt] = attempts + 1
    telem2 = replace(state.telem, tx_attempts=te)
    state2 = replace(state, telem=telem2)
    state2 = state2.log(f"TX attempt {attempts+1} for packet (len={len(pkt)}) -> {'OK' if success else 'NOACK'}")
    if success:
        state2 = replace(state2, telem=state2.telem.dequeue())
    return state2

# --- Tasks definitions (return new state) ----------------------------------

def task_sensor_acquisition_factory(sampling_interval_ms: int) -> TickFn:
    def tick(state: SystemState) -> SystemState:
        # sample BMP, IMU, MAG depending on mode and whether their module is active
        s = state
        if s.mode in ('STANDBY', 'FLIGHT'):
            s, bmp = sample_bmp280(s)
            s, imu = sample_imu(s)
            s, mag = sample_mag(s)
            # store aggregated record to microSD buffer (binary simplified)
            record = (f"{s.t_ms},{s.mode},BMP:{bmp['altitude_m']:.1f},IMU:{imu['accel'][2]:.2f}\n").encode()
            s = sd_append_log(s, record)
        return kick_watchdog(s)  # kick watchdog for health
    return tick

def task_gps_factory(interval_ms: int) -> TickFn:
    def tick(state: SystemState) -> SystemState:
        s = state
        if s.mode == 'FLIGHT':
            s, gps = sample_gps(s)
            rec = (f"{s.t_ms},GPS,{gps['lat']:.6f},{gps['lon']:.6f},{gps['alt']:.1f}\n").encode()
            s = sd_append_log(s, rec)
        return s
    return tick

def task_telem_health_factory(interval_ms: int) -> TickFn:
    def tick(state: SystemState) -> SystemState:
        s = state
        s, pkt = prepare_health_frame(s)
        # set LoRa duty to transmit rarely; we enqueue and allow separate tx task to do it
        return s
    return tick

def task_telem_tx_factory(interval_ms: int) -> TickFn:
    def tick(state: SystemState) -> SystemState:
        s = state
        # decide whether to attempt TX depending on mode and duty-cycle
        if s.mode in ('RECOVERY', 'FLIGHT'):
            s = tx_next_packet(s)
        return s
    return tick

def task_sd_flush_factory(interval_ms: int) -> TickFn:
    def tick(state: SystemState) -> SystemState:
        s = state
        if s.microSD.should_flush():
            s = sd_force_flush(s)
        return s
    return tick

def task_power_mgmt_factory(interval_ms: int) -> TickFn:
    def tick(state: SystemState) -> SystemState:
        # account for power consumption for this tick and update battery
        dt = interval_ms
        total_ma = total_current_ma(state)
        new_batt = state.battery.consume_ma_ms(total_ma, dt)
        s = replace(state, battery=new_batt, t_ms=state.t_ms + dt)
        s = s.log(f"Power tick: current={total_ma:.2f}mA V={new_batt.voltage():.3f} SoC={new_batt.soc:.4f}")
        # check brownout/watchdog etc
        s = check_brownout_and_watchdog(s)
        return s
    return tick

# --- System orchestration helpers ------------------------------------------

def initial_system(capacity_mah: float = 1200.0,
                   modules_cfg: Dict[str, ModuleConfig] = DEFAULT_MODULES,
                   radio_cfg: RadioConfig = DEFAULT_RADIO,
                   brownout_threshold_v: float = 3.3,
                   nominal_v: float = 3.8,
                   init_soc: Optional[float] = None) -> SystemState:
    # if init_soc not provided, derive from nominal voltage
    if init_soc is None:
        # map nominal voltage to SoC on linear model between cutoff and full
        init_soc = (nominal_v - 3.0) / (4.2 - 3.0)
        init_soc = max(0.0, min(1.0, init_soc))
    battery = BatteryModel(capacity_mah=capacity_mah, nominal_voltage=nominal_v, soc=init_soc)
    state = SystemState(
        t_ms=0, mode='STARTUP', battery=battery, modules=modules_cfg,
        microSD=MicroSD(), eeprom=EEPROM(), telem=TelemetryState(), radio=radio_cfg,
        watchdog_counter_ms=0, brownout_threshold_v=brownout_threshold_v, logs=tuple()
    )
    state = state.log("System initialised (STARTUP)")
    return state

def run_simulation(state: SystemState, sim_duration_ms: int, tick_ms: int,
                   tasks: List[Task]) -> SystemState:
    # run a synchronous loop for sim_duration_ms
    sched = Scheduler(tasks=tuple(tasks), tick_ms=tick_ms)
    s = state
    steps = sim_duration_ms // tick_ms
    for i in range(steps):
        # run each task if scheduled (we don't mutate tasks list; tasks hold next_run timestamps)
        # to maintain pure style, we recreates tasks with next_run updates inside functions that need them.
        # Here we simply call each task according to its interval via modulo of simulated time.
        for task in sched.tasks:
            if s.t_ms >= task.next_run_ms:
                s = task.fn(s)
                # update task next_run (we'll create new task tuple)
                # this is local to scheduler object; to keep functional, update sched.tasks in place for next iterations
                # but Python dataclass immutable - so just compute a new tuple
                new_task = replace(task, next_run_ms=s.t_ms + task.interval_ms)
                # replace in sched.tasks
                tlist = list(sched.tasks)
                idx = tlist.index(task)
                tlist[idx] = new_task
                sched = replace(sched, tasks=tuple(tlist))
        # if no tasks executed, still advance time and account for power via power_mgmt tick
        # ensure power_mgmt task runs every tick by having it in tasks
        # after tasks run, we continue loop
        # safety: check battery empty
        if s.battery.soc <= 0.0:
            s = s.log("Battery depleted -> stopping simulation")
            break
    return s

# --- Compose tasks with sensible intervals ---------------------------------

def make_default_tasks(tick_ms: int = 10) -> List[Task]:
    t = 0
    return [
        Task('sensor_sample', interval_ms=100, next_run_ms=0, fn=task_sensor_acquisition_factory(100)),
        Task('gps_sample', interval_ms=1000, next_run_ms=0, fn=task_gps_factory(1000)),
        Task('telem_health', interval_ms=5000, next_run_ms=0, fn=task_telem_health_factory(5000)),
        Task('telem_tx', interval_ms=1000, next_run_ms=0, fn=task_telem_tx_factory(1000)),
        Task('sd_flush', interval_ms=2000, next_run_ms=0, fn=task_sd_flush_factory(2000)),
        Task('power_mgmt', interval_ms=tick_ms, next_run_ms=0, fn=task_power_mgmt_factory(tick_ms)),
    ]

# --- Example scenario ------------------------------------------------------

def scenario_ground_test_then_flight():
    random.seed(42)  # deterministic
    # start system with 1200 mAh
    state = initial_system(capacity_mah=1200.0, nominal_v=3.8)
    tasks = make_default_tasks(tick_ms=10)

    # 1) STARTUP -> STANDBY (ground tests), GPS off, LoRa Rx rare, optimize for long runtime
    state = replace(state, mode='STANDBY')
    state = state.log("Entering STANDBY (ground test): GPS OFF, microSD grouped writes")
    # Configure module duty cycles for long autonomy
    modules = dict(state.modules)
    modules['LoRa'] = replace(modules['LoRa'], duty_cycle=0.001)  # almost always sleep, rare TX
    modules['GPS'] = replace(modules['GPS'], duty_cycle=0.0)
    state = replace(state, modules=modules)

    # simulate ground test for 3 hours
    ground_ms = 3 * 3600 * 1000
    state = run_simulation(state, sim_duration_ms=ground_ms, tick_ms=10, tasks=tasks)

    # After ground test, simulate a flight phase (short) -> FLIGHT mode
    state = replace(state, mode='FLIGHT')
    state = state.log("Entering FLIGHT mode: GPS ON, logging intensive, LoRa TX occasionally")
    modules = dict(state.modules)
    modules['GPS'] = replace(modules['GPS'], duty_cycle=1.0)
    modules['LoRa'] = replace(modules['LoRa'], duty_cycle=0.01)
    state = replace(state, modules=modules)
    # flight for 8 minutes (reasonable upper bound)
    flight_ms = 8 * 60 * 1000
    state = run_simulation(state, sim_duration_ms=flight_ms, tick_ms=10, tasks=tasks)

    # After flight, enter RECOVERY: beaconing, more telemetry TX attempts, conserve power between beacons
    state = replace(state, mode='RECOVERY')
    state = state.log("Entering RECOVERY mode: beaconing and RX enabled")
    # attempt telemetry beacon bursts for 30 minutes or until battery dead
    recovery_ms = 30 * 60 * 1000
    state = run_simulation(state, sim_duration_ms=recovery_ms, tick_ms=10, tasks=tasks)

    # final flush and snapshot
    state = sd_force_flush(state)
    snapshot = {'t_ms': state.t_ms, 'mode': state.mode, 'battery_v': state.battery.voltage()}
    state = replace(state, eeprom=state.eeprom.snapshot(snapshot))
    state = state.log("End scenario: final snapshot in EEPROM; simulation finished")
    return state

# --- Run example when module executed -------------------------------------

if __name__ == "__main__":
    final_state = scenario_ground_test_then_flight()
    print("=== Simulation summary ===")
    print(f"Simulated time: {now_human(final_state.t_ms)}")
    print(f"Final battery V: {final_state.battery.voltage():.3f} V (SoC={final_state.battery.soc:.3f})")
    print(f"Final mode: {final_state.mode}")
    print(f"microSD stored blocks: {len(final_state.microSD.storage)}")
    if final_state.eeprom.last_snapshot:
        print("EEPROM last snapshot:", final_state.eeprom.last_snapshot)
    print("Last 15 log lines:")
    for line in final_state.logs[-15:]:
        print(line)

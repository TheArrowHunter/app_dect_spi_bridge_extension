#include <string.h>
#include <stddef.h>
#include <inttypes.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/hwinfo.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_dect_clock_sync.h>

#include "bridge.h"

LOG_MODULE_REGISTER(dect, LOG_LEVEL_INF);

// Bestimmtes Logging an- und ausschalten, um die Latenz zu messen. Nur für Testzwecke!
// 1 = aktiv 0 = deaktiviert

#define CONFIG_ENABLE_COASTING_TIMEOUT 0

#define CONFIG_ENABLE_SKEW_LOGGING 0

#define CONFIG_ENABLE_HISTOGRAM_LOGGING 0

#define CONFIG_ENABLE_FIRST_TX_RX_LOGGING 0

#define CONFIG_ENABLE_COLDSTART_LOGGING 0

#define CONFIG_ENABLE_MASTER_CLOCK_SKEW_LOGGING 0

/* 1 Mikrosekunde in 69.12 MHz Modem-Ticks */
#define MAX_ALLOWED_DESYNC_TICKS   59LL //69LL  /* ~1.000 ns */

#include <zephyr/drivers/gpio.h>

/* Button 1 (sw0) aus dem Devicetree des DK-Boards holen */
#define SW0_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

/* Globales Flag für die Sende-Sperre */
static volatile bool tx_suppressed = false;
static struct gpio_callback button_cb_data;

/**
 * @brief Interrupt-Handler: Schaltet die Sende-Sperre bei JEDEM Tastendruck um (Toggle)
 */
static void button1_pressed_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    static int64_t last_press_time = 0;
    int64_t now = k_uptime_get();

    /* Entprellung (Debounce): Tappendrücke innerhalb von 250 ms ignorieren */
    if (now - last_press_time < 250) {
        return;
    }
    last_press_time = now;

    /* Zustand der Sendesperre umkehren (Toggle) */
    tx_suppressed = !tx_suppressed;

    if (tx_suppressed) {
        LOG_WRN("[TEST] Button 1 GEDRÜCKT -> Senden UNTERBROCHEN (Schalter: AUS)");
    } else {
        LOG_INF("[TEST] Button 1 GEDRÜCKT -> Senden WIEDERAUFGENOMMEN (Schalter: EIN)");
    }
}

/**
 * @brief Initialisiert Button 1 mit Interrupt NUR auf der steigenden Flanke (Drücken)
 */
void init_test_button(void)
{
    int ret;

    if (!gpio_is_ready_dt(&button1)) {
        LOG_ERR("Error: GPIO Button device not ready");
        return;
    }

    ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Error %d: failed to configure button pin", ret);
        return;
    }

    /* WICHTIG: Ändern auf GPIO_INT_EDGE_TO_ACTIVE (NUR beim Drücken auslösen, nicht beim Loslassen!) */
    ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Error %d: failed to configure interrupt on button pin", ret);
        return;
    }

    gpio_init_callback(&button_cb_data, button1_pressed_callback, BIT(button1.pin));
    gpio_add_callback(button1.port, &button_cb_data);
    LOG_INF("Button 1 Toggle-Modus (TX On/Off) erfolgreich initialisiert.");
}

/* Drift-Messung über ein Block-Fenster */
#define SKEW_CALC_INTERVAL_CYCLES  900u  /* Alle 300 Zyklen (~1 Sekunde) den Skew neu berechnen */

/* Prädiktive Skew-Kompensation für den Freilauf */
static int64_t learned_skew_ppb = 0;   /* Gelernte Drift in Parts per Billion */
static int64_t skew_anchor_offset = 0;
static uint32_t skew_anchor_cycle = 0;
static bool skew_anchor_valid = false;
static bool skew_valid = false;

/* Trackt den letzten Zyklus, für den der Skew bereits aufaddiert/kompensiert wurde */
static uint32_t last_compensated_cycle = 0;

/* Speicher für den gebrochenen Rest (Remainder) in PPB-Ticks */
static int64_t skew_remainder_ppb_ticks = 0;

// erste Master/Slave Nachricht gesendet empfangen
static bool master_first_tx_logged = false;
static bool master_first_rx_logged = false;
static bool slave_first_rx_logged = false;
static bool slave_first_tx_logged = false;

void log_master_first_tx(uint32_t start_tick)
{
    if (!master_first_tx_logged) {
        master_first_tx_logged = true;
        int64_t uptime_ms = k_uptime_get();
		#if IS_ENABLED(CONFIG_ENABLE_FIRST_TX_RX_LOGGING)
        LOG_INF("[TIMING] Master FIRST TX sent at uptime: %lld ms (Tick: %u)", 
                (long long)uptime_ms, start_tick);
		#endif
    }
}

void log_slave_first_tx(uint32_t start_tick)
{
    if (!slave_first_tx_logged) {
        slave_first_tx_logged = true;
        int64_t uptime_ms = k_uptime_get();
		#if IS_ENABLED(CONFIG_ENABLE_FIRST_TX_RX_LOGGING)
        LOG_INF("[TIMING] Slave FIRST TX (Reply) sent at uptime: %lld ms (Tick: %u)", 
                (long long)uptime_ms, start_tick);
		#endif
    }
}

#define MAX_ACQ_SAMPLES 5

/* Puffer für die Akquisitions-Offsets */
static int64_t acq_offsets[MAX_ACQ_SAMPLES];
static uint8_t acq_sample_count = 0;

/* Payload-Struktur für das allererste TX-Paket */
struct __attribute__((packed)) first_tx_payload {
    uint8_t count;                       /* Anzahl gespeicherter Offsets */
    int64_t offsets[MAX_ACQ_SAMPLES];    /* Die Offsets der Pakete 0, 1, 2... */
    int64_t final_grid_offset;           /* Der letztlich für das Grid genutzte Offset */
};

// Ende Sync-Logik

//Histogramm für die Übertragungszeit der Frames
static uint64_t frame_transmission_time;
static int64_t delay_ticks; 
static uint64_t peer_tx_start;

/* ========================================================================= */
/* START: HOCHPRÄZISES ADAPTIVES HISTOGRAMM                                  */
/* ========================================================================= */

#define HIST_NUM_BINS         20LL      
#define HIST_INITIAL_MIN_NS   1440000LL /* Start: 1.440 ms */
#define HIST_INITIAL_MAX_NS   1448000LL /* Start: 1.448 ms */
#define HIST_PADDING_NS       500LL     /* 0.5 µs Sicherheitsabstand links/rechts */

struct latency_histogram {
    uint32_t bins[HIST_NUM_BINS];
    uint32_t underflow;
    uint32_t overflow;
    uint32_t total_samples;
    int64_t min_measured_ns;
    int64_t max_measured_ns;
    int64_t sum_ns;

    int64_t current_min_ns;
    int64_t current_max_ns;
    int64_t current_bin_width_ns;
};

static struct latency_histogram hist = {
    .min_measured_ns = INT64_MAX,
    .max_measured_ns = INT64_MIN,
    .current_min_ns = HIST_INITIAL_MIN_NS,
    .current_max_ns = HIST_INITIAL_MAX_NS,
    .current_bin_width_ns = (HIST_INITIAL_MAX_NS - HIST_INITIAL_MIN_NS) / HIST_NUM_BINS
};

/* ========================================================================= */
/* ENDE: HOCHPRÄZISES ADAPTIVES HISTOGRAMM                                    */
/* ========================================================================= */

/**
 * @brief Sortiert eine gemessene Latenz (in ns) dynamisch in das Histogramm ein.
 * @param delay_ns Gemessener Delay / Übertragungszeit in Nanosekunden
 */
static void histogram_add_sample(int64_t delay_ns)
{
    /* Plausibilitätsfilter: Ungültige Werte / Boot-Ausreißer ignorieren */
    if (delay_ns < 0LL || delay_ns > 10000000LL) {
        return;
    }

    hist.total_samples++;
    hist.sum_ns += delay_ns;

    if (delay_ns < hist.min_measured_ns) hist.min_measured_ns = delay_ns;
    if (delay_ns > hist.max_measured_ns) hist.max_measured_ns = delay_ns;

    if (delay_ns < hist.current_min_ns) {
        hist.underflow++;
        return;
    }
    if (delay_ns >= hist.current_max_ns) {
        hist.overflow++;
        return;
    }

    int bin_idx = (int)((delay_ns - hist.current_min_ns) / hist.current_bin_width_ns);
    if (bin_idx >= HIST_NUM_BINS) {
        bin_idx = HIST_NUM_BINS - 1;
    }

    hist.bins[bin_idx]++;
}

/**
 * @brief Hilfsfunktion zur Formatierung von Nanosekunden als Millisekunden (z.B. 1.444 ms)
 */
static void log_format_ns_as_ms(const char *prefix, int64_t ns_val, const char *suffix)
{
    int64_t ms_int = ns_val / 1000000LL;
    int64_t ms_frac = (ns_val % 1000000LL) / 1000LL; /* Auf Mikrosekunden/3 Nachkommastellen runden */
    if (ms_frac < 0) ms_frac = -ms_frac;

    LOG_INF("%s%lld.%03lld ms%s", prefix, ms_int, ms_frac, suffix);
}

/**
 * @brief Gibt das Histogramm in Millisekunden übersichtlich im Log aus.
 */
static void histogram_print_and_reset(void)
{
    if (hist.total_samples == 0) {
        LOG_INF("[HISTOGRAM] Keine gültigen Samples vorhanden.");
        return;
    }

    int64_t mean_ns = hist.sum_ns / (int64_t)hist.total_samples;

    int64_t min_ms_int = hist.min_measured_ns / 1000000LL;
    int64_t min_ms_frac = (hist.min_measured_ns % 1000000LL) / 1000LL;
    
    int64_t max_ms_int = hist.max_measured_ns / 1000000LL;
    int64_t max_ms_frac = (hist.max_measured_ns % 1000000LL) / 1000LL;

    int64_t mean_ms_int = mean_ns / 1000000LL;
    int64_t mean_ms_frac = (mean_ns % 1000000LL) / 1000LL;

    LOG_INF("================ ADAPTIVE UPLINK HISTOGRAM ================");
    LOG_INF(" Samples: %u | Min: %lld.%03lld ms | Max: %lld.%03lld ms | Mean: %lld.%03lld ms",
            hist.total_samples, 
            min_ms_int, min_ms_frac,
            max_ms_int, max_ms_frac,
            mean_ms_int, mean_ms_frac);
    
    LOG_INF(" Bin Width: %lld ns | Active Span: %lld ns", 
            (long long)hist.current_bin_width_ns, 
            (long long)(hist.current_max_ns - hist.current_min_ns));
    LOG_INF(" ---------------------------------------------------------------");
    
    if (hist.underflow > 0) {
        LOG_INF("   < %lld.%03lld us : %u samples (UNDERFLOW)", 
                hist.current_min_ns / 1000LL, hist.current_min_ns % 1000LL, hist.underflow);
    }

    for (int i = 0; i < HIST_NUM_BINS; i++) {
        int64_t bin_start_ns = hist.current_min_ns + (i * hist.current_bin_width_ns);
        int64_t bin_end_ns = bin_start_ns + hist.current_bin_width_ns;

        LOG_INF("   [%4lld.%03lld - %4lld.%03lld us] : %u", 
                bin_start_ns / 1000LL, bin_start_ns % 1000LL,
                bin_end_ns / 1000LL, bin_end_ns % 1000LL,
                hist.bins[i]);
    }

    if (hist.overflow > 0) {
        LOG_INF("   >= %lld.%03lld us : %u samples (OVERFLOW)", 
                hist.current_max_ns / 1000LL, hist.current_max_ns % 1000LL, hist.overflow);
    }
    LOG_INF("===============================================================");

    /* ===================================================================== */
    /* FEINES ZOOMEINSTELLEN FÜR DEN NÄCHSTEN DURCHLAUF                       */
    /* ===================================================================== */
    if (hist.min_measured_ns != INT64_MAX && hist.max_measured_ns != INT64_MIN) {
        /* Sehr enges Padding (500 ns) um die gemessenen Grenzen */
        hist.current_min_ns = hist.min_measured_ns - HIST_PADDING_NS;
        hist.current_max_ns = hist.max_measured_ns + HIST_PADDING_NS;

        /* Mindestfenster von 1 µs (1000 ns) halten, damit Bin-Breite >= 50 ns bleibt */
        if (hist.current_max_ns - hist.current_min_ns < 1000LL) {
            hist.current_max_ns = hist.current_min_ns + 1000LL;
        }

        hist.current_bin_width_ns = (hist.current_max_ns - hist.current_min_ns) / HIST_NUM_BINS;
    }

    memset(hist.bins, 0, sizeof(hist.bins));
    hist.underflow = 0;
    hist.overflow = 0;
    hist.total_samples = 0;
    hist.sum_ns = 0;
    hist.min_measured_ns = INT64_MAX;
    hist.max_measured_ns = INT64_MIN;
}

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

#if defined(CONFIG_TDMA_ROLE_MASTER)
#define ROLE_NAME "master"
#else
#define ROLE_NAME "slave"
#endif

// The grid
#define TICKS_PER_MS ((uint64_t)NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ)
#define SUBSLOT 14400LL /* mu = 1: 208.33 us */

/* One PPS period in modem ticks: the COEX1 reference pulse (master only)
 * repeats this often.
 */
#define PPS_PERIOD_TICKS (TICKS_PER_MS * 1000ULL)

#define CYCLE_SUBSLOTS 16LL /* 3.333 ms */
#define CYCLE (CYCLE_SUBSLOTS * SUBSLOT)

/* Burst length as the "subslot index" of the DECT PHY transmission length
 * table. This is the value that goes in the PLCF packet_length field; the
 * transmission occupies SUBSLOT_INDEX + 1 subslots. Must be a bare integer
 * token: it is pasted into the TBS table lookup below.
 */
#define SUBSLOT_INDEX 4
#define BURST_SUBSLOTS ((int64_t)SUBSLOT_INDEX + 1)
#define BURST (BURST_SUBSLOTS * SUBSLOT)

#define MASTER_TX_OFFSET 0LL
/* Half a cycle, so master and slave are exactly out of phase and both
 * directions get a symmetric switch guard.
 */
#define SLAVE_TX_OFFSET (CYCLE / 2LL)
/* Clock-uncertainty margin: the RX window is BURST + 2 * GUARD wide, centred
 * on the peer's TX. It is also the gap between this device's own TX and RX
 * ops, which the MEASURED_SCHED_OPR_TRANSITION asserts below check.
 */
#define GUARD (1LL * SUBSLOT)

/* Signed, because the slave's RX window opens GUARD ticks before the cycle
 * boundary, i.e. at a negative offset within the cycle.
 */
#if defined(CONFIG_TDMA_ROLE_MASTER)
#define MY_TX_OFFSET MASTER_TX_OFFSET
#define PEER_TX_OFFSET SLAVE_TX_OFFSET
#else
#define MY_TX_OFFSET SLAVE_TX_OFFSET
#define PEER_TX_OFFSET MASTER_TX_OFFSET
#endif

#define MY_RX_OFFSET (PEER_TX_OFFSET - GUARD)
#define RX_DURATION (BURST + 2LL * GUARD)

/* Depth of the tx_pdu[] staging ring. TX is submitted just-in-time, so
 * normally only one is in flight; this keeps a spare slot.
 */
#define PREFETCH 1u
#define START_MARGIN (4LL * CYCLE)

/* Slave: how long to coast without hearing the master before giving up the
 * grid and re-acquiring. 3000 cycles = 10 s; at 10 ppm that is 100 us of
 * drift, still inside GUARD (208 us). Drift would exceed GUARD after ~21 s.
 */
#define MAX_COAST_CYCLES 3000u

/* Slave: reject an anchor this far from the prediction as a bad measurement.
 * Must stay well under GUARD (208 us).
 */
#define SYNC_ACCEPT_TICKS 10000LL /* ~145 us */

/* Slave: acquisition window length. */
#define ACQ_DURATION (3LL * CYCLE)
/* Cycles to skip ahead when starting the pipeline, so nothing is scheduled
 * inside the acquisition window that is still running.
 */
#define ACQ_LEAD 6u

BUILD_ASSERT(MY_TX_OFFSET + BURST < MY_RX_OFFSET + CYCLE,
			 "TX burst runs into the next cycle's RX window");
BUILD_ASSERT(MY_RX_OFFSET + RX_DURATION < MY_TX_OFFSET + CYCLE,
			 "RX window runs into the TX burst");
BUILD_ASSERT(MY_RX_OFFSET + RX_DURATION < CYCLE,
			 "RX window is longer than a cycle");
BUILD_ASSERT(SLAVE_TX_OFFSET + BURST + GUARD < CYCLE,
			 "slave burst does not fit in the cycle");

/* Modem latencies measured with nrf_modem_dect_phy_latency_get() in
 * LOW_LATENCY radio mode, the only mode dect_init() activates. They are
 * properties of the DECT PHY firmware, not of MCS or burst length; re-measure
 * only after a firmware upgrade.
 */
#define MEASURED_SCHED_OPR_TRANSITION 25920LL /* ticks, LOW_LATENCY */
#define MEASURED_SCHED_OPR_STARTUP 0LL		  /* ticks, LOW_LATENCY */
#define MEASURED_TX_IDLE_TO_ACTIVE 29030LL	  /* ticks */
#define MEASURED_RX_ACTIVE_TO_IDLE 12441LL	  /* ticks -- active_to_idle_rx */

/* The modem needs at least MEASURED_SCHED_OPR_TRANSITION between the end of
 * one scheduled op and the start of the next. Which op comes first differs by
 * role (master: TX then RX; slave: RX then TX), hence two variants.
 */
#if defined(CONFIG_TDMA_ROLE_MASTER)
BUILD_ASSERT(MY_RX_OFFSET - (MY_TX_OFFSET + BURST) >= MEASURED_SCHED_OPR_TRANSITION,
			 "TX->RX switch narrower than the modem's measured scheduled_operation_transition");
BUILD_ASSERT((MY_TX_OFFSET + CYCLE) - (MY_RX_OFFSET + RX_DURATION) >=
				 MEASURED_SCHED_OPR_TRANSITION,
			 "RX->TX switch narrower than the modem's measured scheduled_operation_transition");
#else
BUILD_ASSERT(MY_TX_OFFSET - (MY_RX_OFFSET + RX_DURATION) >= MEASURED_SCHED_OPR_TRANSITION,
			 "RX->TX switch narrower than the modem's measured scheduled_operation_transition");
BUILD_ASSERT((MY_RX_OFFSET + CYCLE) - (MY_TX_OFFSET + BURST) >=
				 MEASURED_SCHED_OPR_TRANSITION,
			 "TX->RX switch narrower than the modem's measured scheduled_operation_transition");
#endif

/* The first scheduled op after boot must clear the modem's cold-start latency. */
BUILD_ASSERT(START_MARGIN >= MEASURED_TX_IDLE_TO_ACTIVE + MEASURED_SCHED_OPR_STARTUP,
			 "START_MARGIN too small for the modem's measured startup latency");

/* How long before its start_time the TX submission call must be issued.
 *
 * This value is empirical: 5 subslots (~1.04 ms) gave zero submit errors over
 * 1000 packets of real traffic, 4 failed ~10% of the time and 3 failed
 * constantly. The BUILD_ASSERT below only checks the theoretical floor
 * (tearing down an active RX, then starting TX); it is necessary but not
 * sufficient. After changing this, confirm on hardware that submit_err and
 * last_err in dect_log_stats() stay at zero. The margin must also absorb
 * thread dispatch jitter between tx_timer firing and the
 * nrf_modem_dect_phy_tx() call.
 */
#define TX_LEAD_TICKS (5LL * SUBSLOT)

BUILD_ASSERT(TX_LEAD_TICKS >= MEASURED_RX_ACTIVE_TO_IDLE + MEASURED_TX_IDLE_TO_ACTIVE,
			 "TX_LEAD_TICKS below RX-teardown-then-TX-startup");

/* RX has no data-freshness requirement, so it runs this many cycles ahead of
 * TX to keep the modem's RX queue topped up.
 */
#define RX_PREFETCH_CYCLES 2u

// Operation handles
#define OP_TX 1u
#define OP_RX 2u
#define OP_ACQ 3u

#define HANDLE(op, cycle) (((uint32_t)(op) << 24) | ((cycle) & 0x00FFFFFFu))
#define HANDLE_OP(h) ((h) >> 24)
#define HANDLE_CYCLE(h) ((h) & 0x00FFFFFFu)

/* Bits carried by one transmission ("Bits per subslot index with given
 * modulation scheme" in the DECT PHY documentation), named
 * TBS_BITS_<MCS>_<subslot index>. Zero means the combination does not exist.
 * Macros rather than an array so BUILD_ASSERT can check the selection at build
 * time. data_size must equal the table value / 8 exactly.
 */
#define TBS_BITS_0_0 0
#define TBS_BITS_0_1 136
#define TBS_BITS_0_2 264
#define TBS_BITS_0_3 400
#define TBS_BITS_0_4 536
#define TBS_BITS_0_5 664
#define TBS_BITS_0_6 792
#define TBS_BITS_0_7 920
#define TBS_BITS_0_8 1064
#define TBS_BITS_0_9 1192

#define TBS_BITS_1_0 32
#define TBS_BITS_1_1 296
#define TBS_BITS_1_2 552
#define TBS_BITS_1_3 824
#define TBS_BITS_1_4 1096
#define TBS_BITS_1_5 1352
#define TBS_BITS_1_6 1608
#define TBS_BITS_1_7 1864
#define TBS_BITS_1_8 2104
#define TBS_BITS_1_9 2360

#define TBS_BITS_2_0 56
#define TBS_BITS_2_1 456
#define TBS_BITS_2_2 856
#define TBS_BITS_2_3 1256
#define TBS_BITS_2_4 1640
#define TBS_BITS_2_5 2024
#define TBS_BITS_2_6 2360
#define TBS_BITS_2_7 2744
#define TBS_BITS_2_8 3192
#define TBS_BITS_2_9 3576

#define TBS_BITS_3_0 88
#define TBS_BITS_3_1 616
#define TBS_BITS_3_2 1128
#define TBS_BITS_3_3 1672
#define TBS_BITS_3_4 2168
#define TBS_BITS_3_5 2680
#define TBS_BITS_3_6 3192
#define TBS_BITS_3_7 3704
#define TBS_BITS_3_8 4256
#define TBS_BITS_3_9 4768

#define TBS_BITS_4_0 144
#define TBS_BITS_4_1 936
#define TBS_BITS_4_2 1736
#define TBS_BITS_4_3 2488
#define TBS_BITS_4_4 3256
#define TBS_BITS_4_5 4024
#define TBS_BITS_4_6 4832
#define TBS_BITS_4_7 5600
#define TBS_BITS_4_8 0
#define TBS_BITS_4_9 0

/* Two-level indirection so CONFIG_MCS and SUBSLOT_INDEX expand before pasting. */
#define TBS_CAT2(mcs, idx) TBS_BITS_##mcs##_##idx
#define TBS_CAT(mcs, idx) TBS_CAT2(mcs, idx)
#define TBS_BITS TBS_CAT(CONFIG_MCS, SUBSLOT_INDEX)

#define TBS_BYTES (TBS_BITS / 8)

/* A literal so it can size an array; asserted against the struct below. */
#define APP_BYTES 30
#define DECT_MAX_ETH_BYTES (TBS_BYTES - APP_BYTES)

/* One transport block. There is no separate link header: the link is a fixed
 * 1-master/1-slave pair, and integrity is left to the DECT PHY's own CRC (a
 * corrupted transport block never reaches on_pdc()).
 *
 * The one piece of state that does need to cross the link, the master's PPS
 * phase (see ns_since_pps()), rides in the first 4 bytes of eth[] on keepalive
 * bursts (eth_payload_len == 0), which are otherwise all-zero padding.
 */
struct dect_pdu
{
	nrf_tx_rx_header app;
	uint8_t eth[DECT_MAX_ETH_BYTES];
} __packed;

BUILD_ASSERT(TBS_BITS > 0,
			 "this CONFIG_MCS / SUBSLOT_INDEX combination does not exist");
BUILD_ASSERT(TBS_BITS % 8 == 0, "transport block is not a whole number of bytes");
BUILD_ASSERT(sizeof(nrf_tx_rx_header) == APP_BYTES, "protocol.h header size changed");
BUILD_ASSERT(sizeof(struct dect_pdu) == TBS_BYTES, "PDU must equal the TBS");
BUILD_ASSERT(DECT_MAX_ETH_BYTES >= 64,
			 "no room for an Ethernet frame: raise CONFIG_MCS or SUBSLOT_INDEX");

static struct dect_pdu tx_pdu[PREFETCH + 2];

/* Reconstruct a peer's full 64-bit start_time from the low 32 bits it sent,
 * anchored on our own clock. tx_start_lo wraps every 62.1 s; our anchor is
 * within a cycle, so the correction is unambiguous.
 */
static uint64_t unwrap_tx_start(uint32_t lo, uint64_t anchor)
{
	uint64_t full = (anchor & ~0xFFFFFFFFULL) | lo;

	if (full > anchor + 0x80000000ULL)
	{
		full -= 0x100000000ULL;
	}
	else if (anchor > full + 0x80000000ULL)
	{
		full += 0x100000000ULL;
	}
	return full;
}

struct phy_ctrl_field_common
{
	uint32_t packet_length : 4;
	uint32_t packet_length_type : 1;
	uint32_t header_format : 3;
	uint32_t short_network_id : 8;
	uint32_t transmitter_id_hi : 8;
	uint32_t transmitter_id_lo : 8;
	uint32_t df_mcs : 3;
	uint32_t reserved : 1;
	uint32_t transmit_power : 4;
	uint32_t pad : 24;
};

enum tdma_state
{
	STATE_ACQUIRING, /* slave only: wide RX, waiting for the master */
	STATE_RUNNING,	 /* scheduling the grid */
	STATE_DRAINING,	 /* slave only: sync lost, let the pipeline empty */
};

static bool fatal;
static uint16_t device_id;
static uint64_t modem_time; /* refreshed by every event */
/* k_uptime_ticks() sampled together with modem_time, so a modem-time instant
 * can be converted to system uptime.
 */
static int64_t modem_time_anchor_uptime;

/* Grid state. Guarded by sync_lock, because the event handler writes it and
 * the scheduler and report threads read it.
 */
static struct k_spinlock sync_lock;
static uint64_t epoch;
static int64_t clock_offset; /* my ticks - master ticks (slave only) */
static bool sync_valid;
static uint32_t last_sync_cycle;
static enum tdma_state state = STATE_ACQUIRING;

/* PPS phase, see ns_since_pps(). Master: its own phase, measured once at boot
 * with clock_sync_get_pulse_info(). Slave: the master's phase, learned from
 * the master's keepalive bursts.
 */
static uint32_t pps_phase_ticks;
static bool pps_phase_valid;

/* Modem ticks -> nanoseconds, truncated to the 32-bit field width of
 * protocol.h. Split into ms and remainder so the multiply cannot overflow.
 * Raw modem-clock time, used only as ns_since_pps()'s fallback before the PPS
 * phase is known.
 */
static uint32_t modem_ns32(uint64_t ticks)
{
	uint64_t ms = ticks / TICKS_PER_MS;
	uint64_t rem = ticks % TICKS_PER_MS;

	return (uint32_t)(ms * 1000000ULL + (rem * 1000000ULL) / TICKS_PER_MS);
}

/* Full-precision ticks -> nanoseconds, for internal math and logging. */
static int64_t ticks_to_ns(int64_t ticks)
{
	return (ticks * 1000000LL) / (int64_t)TICKS_PER_MS;
}

/* Inverse of ticks_to_ns(), magnitude only: callers split off the sign. */
static uint64_t ns_to_ticks(uint64_t ns)
{
	return (ns * (uint64_t)TICKS_PER_MS) / 1000000ULL;
}

/* Modem ticks -> nanoseconds since the most recent COEX1 PPS edge, i.e. the
 * quantity the Teensys report from their PPS-disciplined PTP clocks.
 *
 * The master's ticks are already in the frame the PPS pulse was measured in,
 * so it folds `ticks` into one PPS period and subtracts the phase. The slave's
 * modem clock free-runs independently, so its ticks are first translated into
 * the master's counter via `offset` (clock_offset).
 *
 * Falls back to raw modem time (modem_ns32()) until the PPS phase is known.
 *
 * `offset` is a parameter rather than the global because callers converting an
 * RX timestamp must use the offset standing *before* that packet's own
 * sync_update(): the new one is defined as (stf - peer_tx), so applying it
 * would cancel the over-the-air delay being measured. It is ignored on the
 * master.
 */
static uint32_t ns_since_pps(uint64_t ticks, int64_t offset)
{
	int64_t master_ticks;
	int64_t phase;

	if (!pps_phase_valid)
	{
		return modem_ns32(ticks);
	}

#if defined(CONFIG_TDMA_ROLE_SLAVE)
	master_ticks = (int64_t)ticks - offset;
#else
	ARG_UNUSED(offset);
	master_ticks = (int64_t)ticks;
#endif

	phase = (master_ticks % (int64_t)PPS_PERIOD_TICKS) - (int64_t)pps_phase_ticks;
	if (phase < 0)
	{
		phase += (int64_t)PPS_PERIOD_TICKS;
	}

	return (uint32_t)ticks_to_ns(phase); /* < 1e9, fits comfortably */
}

/* Converts a k_uptime_ticks() sample into the same "ns since PPS edge" value
 * ns_since_pps() produces, so an event timestamped in system uptime (the SPI
 * RX in spi.c) is comparable to dect_tx_ns / dect_rx_ns.
 *
 * It extrapolates from the (modem_time, modem_time_anchor_uptime) pair, which
 * is refreshed on every PHY event, assuming both clocks run at the same rate.
 * The error is a few hundred ns while the anchor is fresh, but grows after a
 * long gap without PHY events (e.g. while acquiring).
 *
 * The delta is split into sign and magnitude because the tick conversion
 * helpers take unsigned arguments.
 */
uint32_t dect_uptime_to_ns_since_pps(int64_t uptime_ticks)
{
	k_spinlock_key_t key = k_spin_lock(&sync_lock);
	uint64_t anchor_modem = modem_time;
	int64_t anchor_uptime = modem_time_anchor_uptime;
	int64_t offset = clock_offset;

	k_spin_unlock(&sync_lock, key);

	int64_t delta_uptime = uptime_ticks - anchor_uptime;
	bool negative = delta_uptime < 0;
	uint64_t delta_ns = k_ticks_to_ns_floor64(negative ? (uint64_t)(-delta_uptime)
													   : (uint64_t)delta_uptime);
	uint64_t delta_modem = ns_to_ticks(delta_ns);
	uint64_t equiv_modem = negative ? anchor_modem - delta_modem
									: anchor_modem + delta_modem;

	return ns_since_pps(equiv_modem, offset);
}

static uint32_t next_rx_cycle; /* scheduler thread only */
static uint32_t next_tx_cycle; /* tx thread only */
static bool acq_pending;

static atomic_t ops_outstanding; /* submitted OP_TX/OP_RX not yet complete */

static struct
{
	atomic_t tx_ok, tx_err, rx_ok, rx_err;
	atomic_t pcc, pdc, pcc_crc_err, pdc_crc_err;
	atomic_t submit_err, hdr_err, orphan_pdc, outlier;
	atomic_t tid_mismatch, pcc_unclaimed;
	atomic_t last_err;
	/* bridge payload */
	atomic_t tx_frames, tx_oversize;
	atomic_t rx_frames, rx_len_bad, rx_nobuf, rx_qfull;
	atomic_t rx_bytes, tx_bytes;
} stats;

/* Sync quality, accumulated between reports (slave only). */
static struct
{
	int64_t min, max, sum;
	uint32_t n;
	int64_t prev_offset;
	uint32_t prev_cycle;
	bool have_prev;
} resid;

K_SEM_DEFINE(operation_sem, 0, 1); /* startup handshakes */
K_SEM_DEFINE(time_sem, 0, 1);
K_SEM_DEFINE(start_sem, 0, 1); /* dect_init() releases the scheduler */
K_SEM_DEFINE(tick_sem, 0, 1);  /* something completed, re-evaluate */
#if defined(CONFIG_TDMA_ROLE_MASTER)
K_SEM_DEFINE(clock_sync_sem, 0, 1);
#endif

static void submit_rx(uint32_t handle, uint64_t start, uint64_t duration);

static void on_init(const struct nrf_modem_dect_phy_init_event *evt)
{
	if (evt->err)
	{
		LOG_ERR("Init failed, err %d", evt->err);
		fatal = true;
		return;
	}
	k_sem_give(&operation_sem);
}

static void on_configure(const struct nrf_modem_dect_phy_configure_event *evt)
{
	if (evt->err)
	{
		LOG_ERR("Configure failed, err %d", evt->err);
		fatal = true;
		return;
	}
	k_sem_give(&operation_sem);
}

static void on_activate(const struct nrf_modem_dect_phy_activate_event *evt)
{
	if (evt->err)
	{
		LOG_ERR("Activate failed, err %d", evt->err);
		fatal = true;
		return;
	}
	k_sem_give(&operation_sem);
}

static void on_time_get(const struct nrf_modem_dect_phy_time_get_event *evt)
{
	if (evt->err)
	{
		LOG_ERR("Time get failed, err %d", evt->err);
		fatal = true;
		return;
	}
	k_sem_give(&time_sem);
}

#if defined(CONFIG_TDMA_ROLE_MASTER)
/* The master also offers a 1PPS reference derived from the modem clock on
 * COEX1 for external equipment.
 */
static void dect_clock_sync_event_handler(const struct nrf_modem_dect_clock_sync_event *evt)
{
	switch (evt->id)
	{
	case NRF_MODEM_DECT_CLOCK_SYNC_EVT_ENABLE_REF_OUTPUT:
		if (evt->clock_sync_enable_ref_output.err == NRF_MODEM_DECT_CLOCK_SYNC_NO_ERROR)
		{
			LOG_INF("COEX1 1PPS reference output enabled");
		}
		else
		{
			LOG_ERR("COEX1 1PPS reference output failed to enable, err %d",
					evt->clock_sync_enable_ref_output.err);
		}
		k_sem_give(&clock_sync_sem);
		break;
	case NRF_MODEM_DECT_CLOCK_SYNC_EVT_PULSE_INFO:
		/* Modem tick of the PPS edge, mod one PPS period. */
		if (evt->clock_sync_pulse_info.err == NRF_MODEM_DECT_CLOCK_SYNC_NO_ERROR)
		{
			uint64_t pulse_time = evt->clock_sync_pulse_info.pulse_time;

			pps_phase_ticks = (uint32_t)(pulse_time % PPS_PERIOD_TICKS);
			pps_phase_valid = true;
			LOG_INF("PPS phase %u ticks (%lld ns) into the modem-time second",
					pps_phase_ticks, (long long)ticks_to_ns(pps_phase_ticks));
		}
		else
		{
			LOG_ERR("PPS pulse info query failed, err %d",
					evt->clock_sync_pulse_info.err);
		}
		k_sem_give(&clock_sync_sem);
		break;
	default:
		break;
	}
}
#endif

// Cycle Grid
static uint64_t cycle_base_locked(uint32_t cycle)
{
	return epoch + (uint64_t)cycle * (uint64_t)CYCLE;
}

static uint64_t cycle_base(uint32_t cycle)
{
	k_spinlock_key_t key = k_spin_lock(&sync_lock);
	uint64_t base = cycle_base_locked(cycle);

	k_spin_unlock(&sync_lock, key);
	return base;
}

/* Fill a staging PDU from the SPI->DECT queue, or build a keepalive if nothing
 * is waiting. Returns the number of payload bytes attached. The queued frame
 * is freed as soon as it has been copied, since the modem only reads the
 * staging PDU.
 */
static uint16_t build_tx_pdu(struct dect_pdu *pdu, uint32_t cycle, uint64_t start)
{
	struct bridge_frame *frame = NULL;
	uint16_t eth_len = 0;

	if (k_msgq_get(&bridge_dect_tx_q, &frame, K_NO_WAIT) == 0)
	{
		uint16_t len = frame->hdr.t_rx.eth_payload_len;

		if (len > DECT_MAX_ETH_BYTES)
		{
			/* One burst is one transport block and fragmentation is
			 * not implemented, so oversize frames are dropped.
			 */
			atomic_inc(&stats.tx_oversize);
			LOG_WRN("frame of %u B exceeds the %u B burst payload, dropped",
					len, (unsigned)DECT_MAX_ETH_BYTES);
		}
		else
		{
			pdu->app = frame->hdr; /* 30 B, t_rx included */
			memcpy(pdu->eth, frame->eth, len);
			eth_len = len;
		}
		bridge_dect_tx_frame_free(frame);
	}

	if (eth_len == 0)
	{
		/* Keepalive: the grid must radiate every cycle or the slave
		 * loses lock.
		 */
		memset(&pdu->app, 0, sizeof(pdu->app));
	}

	/* tx_start_lo is in modem ticks (the peer's clock recovery works in
	 * ticks); dect_tx_ns is the same instant in nanoseconds. nrf_spi_rx_ns
	 * was stamped by spi.c and arrived with the header copy above.
	 */
	pdu->app.dect_sched_header.cycle = cycle;
	pdu->app.dect_sched_header.tx_start_lo = (uint32_t)start;
	pdu->app.dect_tx_ns = ns_since_pps(start, clock_offset);

	/* Do not radiate the tail of an older, longer frame. */
	memset(pdu->eth + eth_len, 0, DECT_MAX_ETH_BYTES - eth_len);

	if (eth_len == 0)
	{
		#if defined(CONFIG_TDMA_ROLE_SLAVE)
			/* Wenn das der allererste TX-Burst ist, schicken wir das Log-Array mit */
			if (!slave_first_tx_logged) {
				struct first_tx_payload acq_data = {
					.count = acq_sample_count,
					.final_grid_offset = clock_offset
				};
				for (int i = 0; i < acq_sample_count; i++) {
					acq_data.offsets[i] = acq_offsets[i];
				}
				memcpy(pdu->eth, &acq_data, sizeof(acq_data));
			} else {
				/* Regulärer Betrieb: Nur den aktuellen Offset mitschicken */
				/* Slave schickt seinen eigenen gemessenen Offset zum Master */
				int64_t slave_offset = clock_offset;
				memcpy(pdu->eth, &slave_offset, sizeof(slave_offset));
			}
		#else
			/* Master-Keepalive: PPS-Phase für den Slave mitschicken */
			/* Keepalive: eth[] carries the PPS phase, see struct dect_pdu.
			* The master's own value is what matters; the master ignores
			* the slave's copy.
			*/
			memcpy(pdu->eth, &pps_phase_ticks, sizeof(pps_phase_ticks));
		#endif
	}
	return eth_len;
}

static void schedule_tx(uint32_t cycle)
{
	/* Wenn Button 1 gedrückt ist, brechen wir das Senden hier einfach ab! */
    if (tx_suppressed) {
        /* Optional: Zähler hochzählen oder Log zur Kontrolle */
        static uint32_t skip_cnt = 0;
        if (skip_cnt++ % 300 == 0) { /* Alle ~1s ein Log, um RTT nicht zu fluten */
            LOG_WRN("Master TX paused (Button 1 held down)...");
        }
        return; /* PAKET NICHT AN DEN PHY GEBEN -> Funkpause! */
    }

	uint64_t start = (uint64_t)((int64_t)cycle_base(cycle) + MY_TX_OFFSET);
	struct dect_pdu *pdu = &tx_pdu[cycle % ARRAY_SIZE(tx_pdu)];
	uint16_t eth_len;
	int err;

	struct phy_ctrl_field_common header = {
		.header_format = 0x0,
		.packet_length_type = 0x0,		/* subslots */
		.packet_length = SUBSLOT_INDEX, /* = BURST_SUBSLOTS - 1 */
		.short_network_id = (CONFIG_NETWORK_ID & 0xff),
		.transmitter_id_hi = (device_id >> 8),
		.transmitter_id_lo = (device_id & 0xff),
		.transmit_power = CONFIG_TX_POWER,
		.reserved = 0,
		.df_mcs = CONFIG_MCS,
	};

	eth_len = build_tx_pdu(pdu, cycle, start);

	struct nrf_modem_dect_phy_tx_params tx = {
		.start_time = start,
		.handle = HANDLE(OP_TX, cycle),
		.network_id = CONFIG_NETWORK_ID,
		.phy_type = 0, /* PLCF type 1 */
		.lbt_rssi_threshold_max = 0,
		.carrier = CONFIG_CARRIER,
		.lbt_period = 0, /* MUST be 0 on a fixed grid */
		.phy_header = (union nrf_modem_dect_phy_hdr *)&header,
		.data = (uint8_t *)pdu,
		.data_size = TBS_BYTES, /* must equal the TBS exactly */
	};

	#if defined(CONFIG_TDMA_ROLE_MASTER)
		log_master_first_tx((uint32_t)start);
	#else
		log_slave_first_tx((uint32_t)start);
	#endif

	err = nrf_modem_dect_phy_tx(&tx);
	if (err)
	{
		LOG_ERR("tx submit cycle %u: %d", cycle, err);
		atomic_inc(&stats.submit_err);
		return;
	}
	atomic_inc(&ops_outstanding);

	if (eth_len)
	{
		atomic_inc(&stats.tx_frames);
		atomic_add(&stats.tx_bytes, eth_len);
	}
}

static void submit_rx(uint32_t handle, uint64_t start, uint64_t duration)
{
	int err;

	struct nrf_modem_dect_phy_rx_params rx = {
		.start_time = start,
		.handle = handle,
		.network_id = CONFIG_NETWORK_ID,
		.mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
		.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
		.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
		.rssi_level = -60,
		.carrier = CONFIG_CARRIER,
		.duration = duration,
		.filter = {
			.short_network_id = (CONFIG_NETWORK_ID & 0xff),
			.is_short_network_id_used = 1,
			/* PLCF type 1 has no receiver identity field, so this must
			 * be 0 or every packet is filtered out.
			 */
			.receiver_identity = 0,
		},
	};

	err = nrf_modem_dect_phy_rx(&rx);
	if (err)
	{
		LOG_ERR("rx submit handle %08x: %d", handle, err);
		atomic_inc(&stats.submit_err);
		return;
	}
	if (HANDLE_OP(handle) != OP_ACQ)
	{
		atomic_inc(&ops_outstanding);
	}
}

static void schedule_rx(uint32_t cycle)
{
	uint64_t start = (uint64_t)((int64_t)cycle_base(cycle) + MY_RX_OFFSET);

	submit_rx(HANDLE(OP_RX, cycle), start, RX_DURATION);
}

K_SEM_DEFINE(tx_deadline_sem, 0, 1); /* given by tx_timer_expired(), taken by tx_thread() */

/* Fires TX_LEAD_TICKS before the next cycle's TX start_time. Runs in ISR
 * context: only gives the semaphore.
 */
static void tx_timer_expired(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_sem_give(&tx_deadline_sem);
}

K_TIMER_DEFINE(tx_timer, tx_timer_expired, NULL);

/* Arms tx_timer to fire TX_LEAD_TICKS before `cycle`'s TX start_time.
 *
 * modem_time/modem_time_anchor_uptime is only as fresh as the last PHY event.
 * The modem-time target is therefore converted to an absolute system-uptime
 * target and compared with a freshly read k_uptime_ticks(), so the age of the
 * anchor cancels out instead of delaying the timer.
 */
static void arm_tx_timer(uint32_t cycle)
{
	uint64_t target_modem = (uint64_t)((int64_t)cycle_base(cycle) + MY_TX_OFFSET -
									   TX_LEAD_TICKS);
	k_spinlock_key_t mt_key = k_spin_lock(&sync_lock);
	uint64_t anchor_modem = modem_time;
	int64_t anchor_uptime = modem_time_anchor_uptime;

	k_spin_unlock(&sync_lock, mt_key);

	/* k_ns_to_ticks_floor64() takes an unsigned duration, so clamp a negative
	 * delta (target at or before the anchor) before converting.
	 */
	int64_t delta_from_anchor_ns = ticks_to_ns((int64_t)target_modem -
											   (int64_t)anchor_modem);

	if (delta_from_anchor_ns < 0)
	{
		delta_from_anchor_ns = 0;
	}

	int64_t target_uptime = anchor_uptime + k_ns_to_ticks_floor64(delta_from_anchor_ns);
	int64_t delay_ticks = target_uptime - k_uptime_ticks();

	if (delay_ticks < 0)
	{
		delay_ticks = 0; /* already past the arm point, fire now */
	}
	k_timer_start(&tx_timer, K_TICKS(delay_ticks), K_NO_WAIT);
}

#if defined(CONFIG_TDMA_ROLE_SLAVE)
/* Called for every valid packet from the master. `stf` is our measurement of
 * when it arrived; `peer_tx` is what the master commanded, in the master's
 * clock; `cycle` is the shared cycle index.
 */
static void sync_update(uint32_t cycle, uint64_t peer_tx, uint64_t stf)
{
	int64_t offset = (int64_t)stf - (int64_t)peer_tx;
	uint64_t new_epoch = (uint64_t)((int64_t)peer_tx + offset -
									(int64_t)cycle * CYCLE);
	k_spinlock_key_t key = k_spin_lock(&sync_lock);

	if (sync_valid)
	{
		int64_t residual = (int64_t)new_epoch - (int64_t)epoch;

		if (llabs(residual) > SYNC_ACCEPT_TICKS)
		{
			k_spin_unlock(&sync_lock, key);
			atomic_inc(&stats.outlier);
			LOG_WRN("anchor outlier cycle %u, residual %lld ticks",
					cycle, residual);
			return;
		}

		/* --- NEU: Skew über ein Fenster von z.B. 300 Zyklen (1 Sekunde) berechnen --- */
		if (!skew_anchor_valid) {
			skew_anchor_offset = offset;
			skew_anchor_cycle = cycle;
			skew_anchor_valid = true;
		} else {
			uint32_t elapsed_cycles = cycle - skew_anchor_cycle;

			/* Erst nach ausreichend vielen Zyklen den Drift berechnen */
			if (elapsed_cycles >= SKEW_CALC_INTERVAL_CYCLES) {
				int64_t delta_offset = offset - skew_anchor_offset;
				int64_t elapsed_ticks = (int64_t)elapsed_cycles * (int64_t)CYCLE;

				/* Drift in PPB berechnen: (delta_ticks * 1e9) / total_ticks */
				int64_t calculated_skew_ppb = (delta_offset * 1000000000LL) / elapsed_ticks;

				if (!skew_valid) {
					learned_skew_ppb = calculated_skew_ppb;
					skew_valid = true;
				} else {
					/* Exponentielle Glättung über die Blöcke */
					learned_skew_ppb = (learned_skew_ppb * 7 + calculated_skew_ppb) / 8;
				}
				#if IS_ENABLED(CONFIG_ENABLE_SKEW_LOGGING)
				LOG_INF("[SKEW UPDATE] Calculated over %u cycles: delta %lld ticks -> learned_skew: %lld ppb",
				        elapsed_cycles, delta_offset, (long long)learned_skew_ppb);
				#endif
				/* Ankerpunkt für den nächsten Block neu setzen */
				skew_anchor_offset = offset;
				skew_anchor_cycle = cycle;
			}
		}

		/* The residual is the error of the previous prediction and is
		 * the sync-quality measurement reported in dect_log_stats().
		 */
		if (resid.n == 0 || residual < resid.min)
		{
			resid.min = residual;
		}
		if (resid.n == 0 || residual > resid.max)
		{
			resid.max = residual;
		}
		resid.sum += residual;
		resid.n++;
	}

	epoch = new_epoch;
	clock_offset = offset;

	/* BEIDE TRACKER ZURÜCKSETZEN: */
    last_sync_cycle = cycle;         /* Unverändert: Für Statistiken, dect_log_stats() & MAX_COAST_CYCLES */
    last_compensated_cycle = cycle;  /* Neu: Für das schrittweise Aufaddieren im Freilauf */
	
	/* reset bei neuem Hardware-RX */
	skew_remainder_ppb_ticks = 0;

	sync_valid = true;
	k_spin_unlock(&sync_lock, key);
}
#endif /* CONFIG_TDMA_ROLE_SLAVE */

/* Only the PCC event carries timing (stf_start_time); the PDC event has none,
 * so the value is handed from one to the other. PCC always immediately
 * precedes the PDC for the same packet, so a single entry suffices. It is
 * consumed on read so a stale timestamp can never be joined to a fresh
 * payload. Do not replace it with a keyed cache: transaction_id is not unique
 * per packet.
 */
static struct
{
	uint64_t stf_start_time;
	uint16_t transaction_id;
	bool valid;
} pending_pcc;

static void pcc_handoff_put(uint16_t tid, uint64_t stf)
{
	if (pending_pcc.valid)
	{
		/* Previous PCC never claimed by a PDC: its payload failed the CRC. */
		atomic_inc(&stats.pcc_unclaimed);
	}
	pending_pcc.transaction_id = tid;
	pending_pcc.stf_start_time = stf;
	pending_pcc.valid = true;
}

static bool pcc_handoff_take(uint16_t tid, uint64_t *stf)
{
	if (!pending_pcc.valid)
	{
		return false;
	}
	pending_pcc.valid = false; /* always consume */

	if (pending_pcc.transaction_id != tid)
	{
		atomic_inc(&stats.tid_mismatch);
		return false;
	}
	*stf = pending_pcc.stf_start_time;
	return true;
}

static void on_pcc(const struct nrf_modem_dect_phy_pcc_event *evt)
{
	atomic_inc(&stats.pcc);

	if (evt->header_status != NRF_MODEM_DECT_PHY_HDR_STATUS_VALID)
	{
		atomic_inc(&stats.hdr_err);
		return;
	}

	pcc_handoff_put(evt->transaction_id, evt->stf_start_time);
}

/* Copy a received PDU into a pool frame, stamp dect_rx_ns and hand it to the
 * SPI thread. Runs in the modem event handler, so it never blocks.
 *
 * nrf_spi_tx_ns, the other half of the union, is left as inherited: the Teensy
 * captures that timestamp itself and overwrites it.
 *
 * `pps_offset` is the clock_offset from before this packet's sync_update(),
 * see ns_since_pps().
 */
static void deliver_rx(const struct dect_pdu *pdu, uint16_t eth_len, uint64_t rx_ts,
					   int64_t pps_offset)
{
	struct bridge_frame *frame;

	if (bridge_spi_tx_frame_alloc(&frame, K_NO_WAIT) != 0)
	{
		atomic_inc(&stats.rx_nobuf);
		return;
	}

	frame->hdr = pdu->app; /* carries t_rx and dect_tx_ns along */
	memcpy(frame->eth, pdu->eth, eth_len);

	frame->hdr.nrf_rx_header.dect_rx_ns = ns_since_pps(rx_ts, pps_offset);

	if (k_msgq_put(&bridge_spi_tx_q, &frame, K_NO_WAIT) != 0)
	{
		atomic_inc(&stats.rx_qfull);
		bridge_spi_tx_frame_free(frame);
		return;
	}

	atomic_inc(&stats.rx_frames);
	atomic_add(&stats.rx_bytes, eth_len);
	spi_bridge_tx_notify();
}

static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt)
{
	const struct dect_pdu *pdu = (const struct dect_pdu *)evt->data;
	uint64_t stf;
	int64_t pps_offset = clock_offset; /* before sync_update() moves it */
	uint32_t cycle;
	uint32_t tx_start_lo;
	uint16_t len;

	atomic_inc(&stats.pdc);

	/* Every burst is a fixed-size transport block, so a legitimate event is
	 * always TBS_BYTES; this also makes the unconditional eth[0..4) read
	 * below safe.
	 */
	if (evt->len < TBS_BYTES)
	{
		LOG_WRN("PDC: undersized (len %zu)", evt->len);
		return;
	}

	if (!pcc_handoff_take(evt->transaction_id, &stf))
	{
		/* Drop it rather than use a timestamp from another packet. */
		atomic_inc(&stats.orphan_pdc);
		return;
	}

	cycle = pdu->app.dect_sched_header.cycle;
	tx_start_lo = pdu->app.dect_sched_header.tx_start_lo;

	/* 0 means keepalive. Needed early: on a slave, eth[0..4) holds the PPS
	 * phase rather than payload when it is 0.
	 */
	len = pdu->app.t_rx.eth_payload_len;

	/* für Histogramme */
	frame_transmission_time = modem_time;
	peer_tx_start = unwrap_tx_start(tx_start_lo, stf);

	/* Clock tracking. With a fixed 1-master/1-slave pair, anything arriving
	 * here is from the peer.
	 */
#if defined(CONFIG_TDMA_ROLE_SLAVE)
	{
		bool first;
		k_spinlock_key_t key = k_spin_lock(&sync_lock);

		first = !sync_valid;

		/* 1. Berechne den rohen Phasensprung / Fehler für dieses Paket */
        int64_t current_raw_offset = (int64_t)stf - (int64_t)unwrap_tx_start(tx_start_lo, stf);
        int64_t phase_error_ticks = llabs(current_raw_offset - clock_offset);

        /* 2. Prüfe, ob wir außerhalb des 1 µs Fensters liegen */
        if (!first && phase_error_ticks > MAX_ALLOWED_DESYNC_TICKS) {
            LOG_WRN("[SYNC LOSS] Phase Drift > 1us detected: %lld ticks (%lld ns)! Re-synchronizing...",
                    (long long)phase_error_ticks, 
                    (long long)ticks_to_ns(phase_error_ticks));
            
            /* Synchronisation als ungültig markieren & State zurücksetzen */
            sync_valid = false;
            state = STATE_ACQUIRING;
            acq_sample_count = 0; /* Reset für Kaltstart-Messung */
        }

		k_spin_unlock(&sync_lock, key);

		//speichere erste Offsets für die spätere Auswertung um die Synchronisation zu überprüfen
		if (state == STATE_ACQUIRING && acq_sample_count < MAX_ACQ_SAMPLES) {
			int64_t raw_offset = (int64_t)stf - (int64_t)unwrap_tx_start(tx_start_lo, stf);
			acq_offsets[acq_sample_count++] = raw_offset;
			#if IS_ENABLED(CONFIG_ENABLE_COLDSTART_LOGGING)
			LOG_INF("[ACQ DATA] Sample #%u Offset: %lld ticks", 
			        acq_sample_count - 1, (long long)raw_offset);
			#endif
		}

		delay_ticks = (int64_t)frame_transmission_time - ((int64_t)peer_tx_start + pps_offset); //Histogramm

		sync_update(cycle, unwrap_tx_start(tx_start_lo, stf), stf);

		/* The PPS phase rides on keepalives only, so after a master reboot
		 * it is refreshed at the next keepalive gap.
		 */
		if (len == 0)
		{
			memcpy(&pps_phase_ticks, pdu->eth, sizeof(pps_phase_ticks));
			pps_phase_valid = true;
		}
		if (first)
		{
			LOG_INF("locked on master cycle %u, offset %lld ticks",
					cycle, (long long)clock_offset);
			k_sem_give(&tick_sem);
		}
		
		if (!slave_first_rx_logged) {
			slave_first_rx_logged = true;
			int64_t uptime_ms = k_uptime_get();
			#if IS_ENABLED(CONFIG_ENABLE_FIRST_TX_RX_LOGGING)
			LOG_INF("[TIMING] Slave FIRST RX (Master Beacon) at uptime: %lld ms (STF: %u)", 
					(long long)uptime_ms, stf);
			#endif		
		}
		
	}
#else
	{
		/* The master needs no sync, but the same expression the slave
		 * uses gives the two-way solution:
		 *   offset_slave  = -theta + d      offset_master = +theta + d
		 *   d = (offset_master + offset_slave) / 2
		 * where d is propagation plus TX/RX chain asymmetry.
		 */
		k_spinlock_key_t key = k_spin_lock(&sync_lock);

		static int64_t round_trip_ns;

		clock_offset = (int64_t)stf - (int64_t)unwrap_tx_start(tx_start_lo, stf);
		last_sync_cycle = cycle;

		delay_ticks = (int64_t)frame_transmission_time - (int64_t)peer_tx_start; //Histogramm

		// Round-trip delay: the master's own measurement of the two-way delay, in ns.

		int64_t slave_offset = 0;
		memcpy(&slave_offset, pdu->eth, sizeof(slave_offset));

		/* Master berechnet seinen eigenen Offset für diesen Zyklus */
		int64_t master_offset = (int64_t)stf - (int64_t)unwrap_tx_start(tx_start_lo, stf);

		/* Durch die Addition löscht sich der Clock Drift (+theta und -theta) komplett aus! */
		int64_t true_two_way_ticks = master_offset + slave_offset;
		int64_t true_one_way_ticks = true_two_way_ticks / 2;

		int64_t true_one_way_ns = ticks_to_ns(true_one_way_ticks);

		/* 4. Block-Mittelwert über 5000 Samples */
		static int64_t ns_accumulator = 0;
		static uint32_t sample_count = 0;

		ns_accumulator += true_one_way_ns;
		sample_count++;

		// Round-trip delay end 

		/* Auswertung des ALLERERSTEN Slave-Pakets auf dem Master */
		#if IS_ENABLED(CONFIG_ENABLE_COLDSTART_LOGGING)
		if (!master_first_rx_logged) {
			struct first_tx_payload *acq_data = (struct first_tx_payload *)pdu->eth;

			LOG_INF("========== COLDSTART PRE-SYNC TWO-WAY EVALUATION ==========");
			LOG_INF("Slave reported %u acquisition samples before Grid-Lock:", acq_data->count);

			if (acq_data->count > 0) {
				for (int i = 0; i < acq_data->count; i++) {
					int64_t sample_slave_offset = acq_data->offsets[i];
					
					/* Two-Way Delay für dieses Pre-Sync Sample berechnen */
					int64_t sample_two_way_ticks = master_offset + sample_slave_offset;
					int64_t sample_one_way_ticks = sample_two_way_ticks / 2;
					int64_t sample_one_way_ns = ticks_to_ns(sample_one_way_ticks);

					LOG_INF("  [Pre-Sync #%d] S_Off: %lld t | TwoWay: %lld t | OneWay: %lld t (%lld ns)",
							i,
							(long long)sample_slave_offset,
							(long long)sample_two_way_ticks,
							(long long)sample_one_way_ticks,
							(long long)sample_one_way_ns);
				}
			}
			LOG_INF("  [Grid Start] Final Used Offset: %lld ticks", 
			        (long long)acq_data->final_grid_offset);
			LOG_INF("===========================================================");
		}
		#endif
		k_spin_unlock(&sync_lock, key);

		if (!master_first_rx_logged) {
			master_first_rx_logged = true;
			int64_t uptime_ms = k_uptime_get();
			#if IS_ENABLED(CONFIG_ENABLE_FIRST_TX_RX_LOGGING)
			LOG_INF("[TIMING] Master FIRST RX received from Slave at uptime: %lld ms (STF: %llu)", 
					(long long)uptime_ms, (unsigned long long)stf);
			#endif		
		}
		
		// Überprüfung: Wurde der Block von 5000 Samples erreicht?
		#if IS_ENABLED(CONFIG_ENABLE_MASTER_CLOCK_SKEW_BLOCK_LOGGING)
		if (sample_count >= 5000)
		{
			// Exakter arithmetischer Mittelwert des Blocks
			int64_t avg_one_way_ns = ns_accumulator / sample_count;

			LOG_INF("Block Avg (5000 samples): clock skew: %lld ns (raw sample: %lld ticks / %lld ns)",
					(long long)avg_one_way_ns,
					(long long)true_one_way_ticks,
					(long long)true_one_way_ns);

			// Akkumulator und Zähler für den nächsten 5000er-Block zurücksetzen
			ns_accumulator = 0;
			sample_count = 0;
		}
		#endif
		/* Statischer Zähler für die empfangenen Samples seit dem Kaltstart */
		static uint32_t startup_samples = 0;
		static int64_t prev_slave_offset = 0;

		/* 2. Clock Skew / Drift-Berechnung (Änderung des Slave-Offsets pro Zyklus in Ticks) */
		int64_t clock_skew_ticks = slave_offset - prev_slave_offset;
		prev_slave_offset = slave_offset;

		/* 3. High-Frequency Logging für die ersten 10 Samples */
		if (startup_samples < 10)
		{
			// Ausgabe jedes 10. Samples (~33 ms Raster), um das RTT-Terminal nicht zu überfluten
//			if (startup_samples % 10 == 0)
//			{
				// Umrechnung des Skews in ppb (1 Tick ≈ 14.46 ns bei 69.12 MHz, cycle = 3.33ms)
				// skew_ticks * 14.468 ns / 0.00333 s ≈ skew_ticks * 4341 ppb
				#if IS_ENABLED(CONFIG_ENABLE_COLDSTART_LOGGING)
				LOG_INF("[COLDSTART #%04u] M_Off: %lld t | S_Off: %lld t | OneWay: %lld t | OneWay: %lld ns) | Skew: %lld t",
						startup_samples,
						(long long)master_offset,
						(long long)slave_offset,
						(long long)(true_two_way_ticks/2),
						(long long)ticks_to_ns(true_two_way_ticks / 2),
						(long long)clock_skew_ticks);
				#endif
//			}
			
			startup_samples++;
		}
		
	}
#endif

	if (len != 0)
	{
		if (len > DECT_MAX_ETH_BYTES)
		{
			atomic_inc(&stats.rx_len_bad);
		}
		else
		{
			/* modem_time is this PDC event's timestamp: when the full
			 * payload was available, as opposed to stf, the burst start.
			 */
			deliver_rx(pdu, len, modem_time, pps_offset);
		}
	}

	// In Nanosekunden umrechnen und in das Histogramm einsortieren
    int64_t delay_ns = ticks_to_ns(delay_ticks);
    histogram_add_sample(delay_ns);

    // Alle 1000 Pakete (~3,3 Sekunden) das Histogramm im Terminal ausgeben
    static uint32_t sample_counter = 0;
    sample_counter++;
    if (sample_counter >= 1000) {
		#if IS_ENABLED(CONFIG_ENABLE_HISTOGRAM_LOGGING)
        histogram_print_and_reset();
		#endif
        sample_counter = 0;
    }
}

static void on_pcc_crc_err(const struct nrf_modem_dect_phy_pcc_crc_failure_event *evt)
{
	ARG_UNUSED(evt);
	atomic_inc(&stats.pcc_crc_err);
}

static void on_pdc_crc_err(const struct nrf_modem_dect_phy_pdc_crc_failure_event *evt)
{
	ARG_UNUSED(evt);
	atomic_inc(&stats.pdc_crc_err);
}

static void on_op_complete(const struct nrf_modem_dect_phy_op_complete_event *evt,
						   uint64_t event_time)
{
	uint32_t op = HANDLE_OP(evt->handle);

	switch (op)
	{
	case OP_TX:
		if (evt->err)
		{
			atomic_inc(&stats.tx_err);
			atomic_set(&stats.last_err, evt->err);
			LOG_WRN("TX cycle %u err %d", HANDLE_CYCLE(evt->handle),
					evt->err);
		}
		else
		{
			atomic_inc(&stats.tx_ok);
		}
		atomic_dec(&ops_outstanding);
		break;

	case OP_RX:
		/* An RX window that heard nothing completes without error. */
		if (evt->err)
		{
			atomic_inc(&stats.rx_err);
			atomic_set(&stats.last_err, evt->err);
			LOG_WRN("RX cycle %u err %d", HANDLE_CYCLE(evt->handle),
					evt->err);
		}
		else
		{
			atomic_inc(&stats.rx_ok);
		}
		atomic_dec(&ops_outstanding);
		break;

	case OP_ACQ:
		acq_pending = false;
		if (sync_valid)
		{
			/* Start far enough ahead to clear the finished window. */
			k_spinlock_key_t key = k_spin_lock(&sync_lock);
			uint64_t base = epoch;

			k_spin_unlock(&sync_lock, key);

			next_rx_cycle = (uint32_t)((event_time - base) /
									   (uint64_t)CYCLE) +
							ACQ_LEAD;
			next_tx_cycle = next_rx_cycle;
			state = STATE_RUNNING;
			arm_tx_timer(next_tx_cycle);
			LOG_INF("acquired, starting grid at cycle %u", next_rx_cycle);
		}
		else
		{
			LOG_INF("acquisition window elapsed, nothing heard");
		}
		break;

	default:
		return;
	}

	k_sem_give(&tick_sem);
}

static void dect_phy_event_handler(const struct nrf_modem_dect_phy_event *evt)
{
	/* Update the pair atomically: arm_tx_timer() reads both. */
	k_spinlock_key_t mt_key = k_spin_lock(&sync_lock);

	modem_time = evt->time;
	modem_time_anchor_uptime = k_uptime_ticks();
	k_spin_unlock(&sync_lock, mt_key);

	switch (evt->id)
	{
	case NRF_MODEM_DECT_PHY_EVT_INIT:
		on_init(&evt->init);
		break;
	case NRF_MODEM_DECT_PHY_EVT_CONFIGURE:
		on_configure(&evt->configure);
		break;
	case NRF_MODEM_DECT_PHY_EVT_ACTIVATE:
		on_activate(&evt->activate);
		break;
	case NRF_MODEM_DECT_PHY_EVT_TIME:
		on_time_get(&evt->time_get);
		break;
	case NRF_MODEM_DECT_PHY_EVT_COMPLETED:
		on_op_complete(&evt->op_complete, evt->time);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PCC:
		on_pcc(&evt->pcc);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PCC_ERROR:
		on_pcc_crc_err(&evt->pcc_crc_err);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PDC:
		on_pdc(&evt->pdc);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PDC_ERROR:
		on_pdc_crc_err(&evt->pdc_crc_err);
		break;
	default:
		break;
	}
}

static struct nrf_modem_dect_phy_config_params dect_phy_config_params = {
	.band_group_index = ((CONFIG_CARRIER >= 525 && CONFIG_CARRIER <= 551)) ? 1 : 0,
	.harq_rx_process_count = 4,
	.harq_rx_expiry_time_us = 5000000,
};

static uint32_t cycles_since_sync(void)
{
	k_spinlock_key_t key = k_spin_lock(&sync_lock);
	uint32_t n = next_tx_cycle - last_sync_cycle;

	k_spin_unlock(&sync_lock, key);
	return n;
}

static void scheduler_thread(void)
{
	k_sem_take(&start_sem, K_FOREVER);

	while (true)
	{
		switch (state)
		{
		case STATE_ACQUIRING:
			if (!acq_pending)
			{
				acq_pending = true;
				LOG_INF("acquiring...");
				submit_rx(HANDLE(OP_ACQ, 0), 0, ACQ_DURATION);
			}
			break;

		case STATE_RUNNING:
			/* TX is paced by tx_thread()/tx_timer; RX just stays
			 * RX_PREFETCH_CYCLES ahead of it.
			 */
			while (next_rx_cycle < next_tx_cycle + RX_PREFETCH_CYCLES)
			{
				schedule_rx(next_rx_cycle++);
			}
#if defined(CONFIG_TDMA_ROLE_SLAVE)
			k_spinlock_key_t key = k_spin_lock(&sync_lock);
			
			/* 1. Differenz zum letzten bereits kompensierten Zyklus berechnen */
			uint32_t uncompensated_cycles = next_tx_cycle - last_compensated_cycle;

			if (uncompensated_cycles > 1 && skew_valid) {
				/* Drift nur für die NEUEN verpassten Zyklen berechnen */
				uint32_t cycles_to_add = uncompensated_cycles - 1;
				/* 1. Berechne den genauen Zuwachs an PPB-Ticks für die verpassten Zyklen */
				int64_t added_ppb_ticks = learned_skew_ppb * (int64_t)CYCLE * (int64_t)cycles_to_add;

				/* 2. Addiere den Zuwachs auf den Akkumulator-Puffer */
				skew_remainder_ppb_ticks += added_ppb_ticks;

				/* 3. Ermittle, wie viele GANZE Ticks im Puffer stecken (Division durch 1e9) */
				int64_t drift_ticks = skew_remainder_ppb_ticks / 1000000000LL;

				/* 4. Ziehe die verbuchten ganzen Ticks vom Puffer ab (der Rest bleibt für das nächste Mal) */
				skew_remainder_ppb_ticks %= 1000000000LL;
				
				/* 5. Nur wenn mindestens 1 ganzer Tick aufgelaufen ist, Zeitgitter anpassen */
				if (drift_ticks != 0) {
					epoch += drift_ticks;
					clock_offset += drift_ticks;
					
					#if IS_ENABLED(CONFIG_ENABLE_SKEW_LOGGING)
					LOG_WRN("[COASTING] Extrapolated drift: %+lld ticks for cycle %u (learned: %lld ppb)",
							(long long)drift_ticks, next_tx_cycle, (long long)learned_skew_ppb);
					#endif
				}

				last_compensated_cycle = next_tx_cycle - 1;
			}
			k_spin_unlock(&sync_lock, key);

			#if IS_ENABLED(CONFIG_ENABLE_COASTING_TIMEOUT)
			if (cycles_since_sync() > MAX_COAST_CYCLES)
			{
				LOG_ERR("sync lost after %u cycles, draining",
						cycles_since_sync());
				k_timer_stop(&tx_timer);
				state = STATE_DRAINING;
			}
			#endif
#endif
			break;

		case STATE_DRAINING:
			if (atomic_get(&ops_outstanding) == 0)
			{
				k_spinlock_key_t key = k_spin_lock(&sync_lock);

				sync_valid = false;
				k_spin_unlock(&sync_lock, key);
				pending_pcc.valid = false;
				resid.n = 0;
				resid.have_prev = false;
				state = STATE_ACQUIRING;
			}
			break;
		}

		/* The timeout doubles as a watchdog if completions stop. */
		k_sem_take(&tick_sem, K_MSEC(500));
	}
}

K_THREAD_DEFINE(scheduler_tid, 2048, scheduler_thread, NULL, NULL, NULL, 5, 0, 0);

/* Woken by tx_timer_expired() TX_LEAD_TICKS before each cycle's TX start_time.
 * Priority 0, the highest preemptible priority in the app, so nothing else in
 * the app delays the submit. spi_tid sits at the bottom (7): if "bad RX
 * length" warnings or hdr_mm/len_err growth show up in the SPI stats, its
 * priority is the thing to revisit.
 *
 * The modem event handler runs on the system workqueue, which is cooperative
 * by default and would block this thread past the TX_LEAD_TICKS margin under
 * load. CONFIG_SYSTEM_WORKQUEUE_PRIORITY in prj.conf makes it preemptible and
 * places it below every app thread. If TX submissions start missing their
 * deadline, check that setting first.
 */
static void tx_thread(void)
{
	while (true)
	{
		k_sem_take(&tx_deadline_sem, K_FOREVER);

		if (state != STATE_RUNNING)
		{
			continue; /* draining/acquiring: no TX, no re-arm */
		}
		schedule_tx(next_tx_cycle);
		arm_tx_timer(++next_tx_cycle);
	}
}

K_THREAD_DEFINE(tx_tid, 2048, tx_thread, NULL, NULL, NULL, 0, 0, 0);

void dect_log_stats(void)
{
	LOG_INF("%s state %d tx_cyc %u rx_cyc %u out %ld | tx %ld/%ld rx %ld/%ld | "
			"pcc %ld pdc %ld crc %ld/%ld hdr %ld orphan %ld "
			"tid_mm %ld unclaimed %ld | submit_err %ld last_err %ld",
			ROLE_NAME, state, next_tx_cycle, next_rx_cycle,
			atomic_get(&ops_outstanding),
			atomic_get(&stats.tx_ok), atomic_get(&stats.tx_err),
			atomic_get(&stats.rx_ok), atomic_get(&stats.rx_err),
			atomic_get(&stats.pcc), atomic_get(&stats.pdc),
			atomic_get(&stats.pcc_crc_err), atomic_get(&stats.pdc_crc_err),
			atomic_get(&stats.hdr_err), atomic_get(&stats.orphan_pdc),
			atomic_get(&stats.tid_mismatch),
			atomic_get(&stats.pcc_unclaimed),
			atomic_get(&stats.submit_err), atomic_get(&stats.last_err));

	{
		uint32_t rxb = (uint32_t)atomic_set(&stats.rx_bytes, 0);
		uint32_t txb = (uint32_t)atomic_set(&stats.tx_bytes, 0);

		LOG_INF("  air: tx %ld frm %u B/s | rx %ld frm %u B/s | "
				"oversize %ld len_bad %ld nobuf %ld qfull %ld",
				atomic_get(&stats.tx_frames), txb,
				atomic_get(&stats.rx_frames), rxb,
				atomic_get(&stats.tx_oversize),
				atomic_get(&stats.rx_len_bad),
				atomic_get(&stats.rx_nobuf),
				atomic_get(&stats.rx_qfull));
	}

	{
		k_spinlock_key_t key = k_spin_lock(&sync_lock);
		int64_t offset = clock_offset;
		uint32_t cyc = last_sync_cycle;
		bool valid = sync_valid;
		int64_t mn = resid.min, mx = resid.max, sum = resid.sum;
		uint32_t n = resid.n;
		bool have_prev = resid.have_prev;
		int64_t prev_off = resid.prev_offset;
		uint32_t prev_cyc = resid.prev_cycle;

		resid.min = resid.max = resid.sum = 0;
		resid.n = 0;
		resid.prev_offset = offset;
		resid.prev_cycle = cyc;
		resid.have_prev = valid;
		k_spin_unlock(&sync_lock, key);

		if (!valid)
		{
			LOG_INF("  sync: not locked");
		}
		else if (n == 0)
		{
			LOG_INF("  sync: offset %lld, no samples "
					"(outliers %ld)",
					(long long)offset,
					atomic_get(&stats.outlier));
		}
		else
		{
			int64_t mean = sum / (int64_t)n;
			int64_t span_ticks = (int64_t)(cyc - prev_cyc) * CYCLE;
			int64_t ppb = 0;

			if (have_prev && span_ticks > 0)
			{
				ppb = ((offset - prev_off) * 1000000000LL) /
					  span_ticks;
			}

			LOG_INF("  sync: n %u residual min %lld max %lld "
					"mean %lld ticks (%lld..%lld ns) | "
					"offset %lld | drift %lld ppb | "
					"outliers %ld, coast %u",
					n, (long long)mn, (long long)mx,
					(long long)mean,
					(long long)ticks_to_ns(mn),
					(long long)ticks_to_ns(mx),
					(long long)offset, (long long)ppb,
					atomic_get(&stats.outlier),
					cycles_since_sync());
		}
	}
}

int dect_init(void)
{
	int err;

	LOG_INF("DECT NR+ TDMA %s starting", ROLE_NAME);

	err = nrf_modem_lib_init();
	if (err)
	{
		LOG_ERR("modem init failed, err %d", err);
		return err;
	}

#if defined(CONFIG_TDMA_ROLE_MASTER)
	err = nrf_modem_dect_clock_sync_event_handler_set(dect_clock_sync_event_handler);
	if (err)
	{
		LOG_ERR("clock_sync event_handler_set failed, err %d", err);
		return err;
	}

	err = nrf_modem_dect_clock_sync_enable_ref_output();
	if (err)
	{
		LOG_ERR("clock_sync_enable_ref_output failed, err %d", err);
		return err;
	}
	k_sem_take(&clock_sync_sem, K_FOREVER);

	/* Calibrate once which modem tick the PPS pulse falls on, see
	 * ns_since_pps().
	 */
	err = nrf_modem_dect_clock_sync_get_pulse_info(NRF_MODEM_DECT_CLOCK_SYNC_PULSE_OUTGOING);
	if (err)
	{
		LOG_ERR("clock_sync_get_pulse_info failed, err %d", err);
		return err;
	}
	k_sem_take(&clock_sync_sem, K_FOREVER);
	if (!pps_phase_valid)
	{
		return -EIO;
	}
#endif

	err = nrf_modem_dect_phy_event_handler_set(dect_phy_event_handler);
	if (err)
	{
		LOG_ERR("event_handler_set failed, err %d", err);
		return err;
	}

	err = nrf_modem_dect_phy_init();
	if (err)
	{
		LOG_ERR("dect_phy_init failed, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	if (fatal)
	{
		return -EIO;
	}

	err = nrf_modem_dect_phy_configure(&dect_phy_config_params);
	if (err)
	{
		LOG_ERR("dect_phy_configure failed, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	if (fatal)
	{
		return -EIO;
	}

	err = nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	if (err)
	{
		LOG_ERR("dect_phy_activate failed, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	if (fatal)
	{
		return -EIO;
	}

	hwinfo_get_device_id((void *)&device_id, sizeof(device_id));
	LOG_INF("activated, device ID %u, role %s", device_id, ROLE_NAME);

	LOG_INF("burst %d B TBS = %d B bridge hdr + %d B eth (MCS %d, %lld subslots)",
			TBS_BYTES, APP_BYTES, (int)DECT_MAX_ETH_BYTES,
			CONFIG_MCS, (long long)BURST_SUBSLOTS);

	LOG_INF("cycle %lld ticks (%lld ms) | tx +%lld len %lld | rx %+lld len %lld",
			(long long)CYCLE, (long long)(CYCLE / (int64_t)TICKS_PER_MS),
			(long long)MY_TX_OFFSET, (long long)BURST,
			(long long)MY_RX_OFFSET, (long long)RX_DURATION);

#if defined(CONFIG_TDMA_ROLE_MASTER)
	/* Read modem time once; the grid is epoch + cycle * CYCLE from here on. */
	err = nrf_modem_dect_phy_time_get();
	if (err)
	{
		LOG_ERR("dect_phy_time_get failed, err %d", err);
		return err;
	}
	k_sem_take(&time_sem, K_FOREVER);
	if (fatal)
	{
		return -EIO;
	}

	epoch = (uint64_t)(((int64_t)modem_time + START_MARGIN) / CYCLE + 1) *
			(uint64_t)CYCLE;
	next_rx_cycle = 0;
	next_tx_cycle = 0;
	sync_valid = true;
	state = STATE_RUNNING;
	arm_tx_timer(next_tx_cycle);

	LOG_INF("now %" PRIu64 " epoch %" PRIu64, modem_time, epoch);
#else
	/* The slave learns the epoch from the air. */
	state = STATE_ACQUIRING;
#endif

	k_sem_give(&start_sem);
	return 0;
}

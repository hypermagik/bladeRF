#include "main.hpp"

#define CHECK(x)                                        \
  {                                                     \
    int status = x;                                     \
    if (status) {                                       \
      LOG(#x " failed - %s", bladerf_strerror(status)); \
      exit(1);                                          \
    }                                                   \
  }

void device_init() {
  LOG("Opening bladeRF...");

  bladerf_log_set_verbosity(rf_log_level);

  CHECK(bladerf_open(&device, ""));

  const char *env_tuning_mode = getenv("TUNING_MODE_FPGA");
  if (env_tuning_mode != nullptr) {
    tuning_mode = static_cast<bladerf_tuning_mode>(atoi(env_tuning_mode));
  }

  LOG("Setting tuning mode to %s...", tuning_mode == BLADERF_TUNING_MODE_FPGA ? "FPGA" : "host");
  CHECK(bladerf_set_tuning_mode(device, tuning_mode));

  LOG("Setting manual Rx gain mode...");
  CHECK(bladerf_set_gain_mode(device, BLADERF_RX_X2, BLADERF_GAIN_MGC));

  LOG("Setting default FIR...");
  CHECK(bladerf_set_rfic_rx_fir(device, bladerf_rfic_rxfir::BLADERF_RFIC_RXFIR_DEFAULT));
  CHECK(bladerf_set_rfic_tx_fir(device, bladerf_rfic_txfir::BLADERF_RFIC_TXFIR_DEFAULT));

  bladerf_sample_rate rf_actual_rate;
  bladerf_bandwidth rf_actual_bw;

  LOG("Setting Tx sample rate to %u and bandwidth to %u...", rate, bw);
  CHECK(
      bladerf_set_sample_rate(device, BLADERF_CHANNEL_TX(0), static_cast<bladerf_sample_rate>(rate), &rf_actual_rate));
  CHECK(bladerf_set_bandwidth(device, BLADERF_CHANNEL_TX(0), static_cast<bladerf_bandwidth>(bw), &rf_actual_bw));
  CHECK(
      bladerf_set_sample_rate(device, BLADERF_CHANNEL_TX(1), static_cast<bladerf_sample_rate>(rate), &rf_actual_rate));
  CHECK(bladerf_set_bandwidth(device, BLADERF_CHANNEL_TX(1), static_cast<bladerf_bandwidth>(bw), &rf_actual_bw));
  LOG("Actual Tx rate is %u and bandwidth is %u", rf_actual_rate, rf_actual_bw);
  CHECK(
      bladerf_set_sample_rate(device, BLADERF_CHANNEL_RX(0), static_cast<bladerf_sample_rate>(rate), &rf_actual_rate));
  CHECK(bladerf_set_bandwidth(device, BLADERF_CHANNEL_RX(0), static_cast<bladerf_bandwidth>(bw), &rf_actual_bw));
  CHECK(
      bladerf_set_sample_rate(device, BLADERF_CHANNEL_RX(1), static_cast<bladerf_sample_rate>(rate), &rf_actual_rate));
  CHECK(bladerf_set_bandwidth(device, BLADERF_CHANNEL_RX(1), static_cast<bladerf_bandwidth>(bw), &rf_actual_bw));
  LOG("Actual Tx rate is %u and bandwidth is %u", rf_actual_rate, rf_actual_bw);

  LOG("Setting gains to %d...", gain);
  CHECK(bladerf_set_gain(device, BLADERF_CHANNEL_TX(0), static_cast<bladerf_gain>(gain)));
  CHECK(bladerf_set_gain(device, BLADERF_CHANNEL_RX(0), static_cast<bladerf_gain>(gain)));

  LOG("Setting frequency to %u...", frequency);
  CHECK(bladerf_set_frequency(device, BLADERF_CHANNEL_TX(0), static_cast<bladerf_frequency>(frequency)));
  CHECK(bladerf_set_frequency(device, BLADERF_CHANNEL_RX(0), static_cast<bladerf_frequency>(frequency)));

  if (rx_all_events) {
    LOG("Enabling BLADERF_FEATURE_RX_ALL_EVENTS...");
    CHECK(bladerf_enable_feature(device, bladerf_feature::BLADERF_FEATURE_RX_ALL_EVENTS, true));
  }

  CHECK(bladerf_set_rx_mux(device, rx_mux));

  struct bladerf_version fw_version;
  CHECK(bladerf_fw_version(device, &fw_version));
  if (fw_version.major < 2 || fw_version.minor < 5) {
    message_size = 2048;
  }

  LOG("Firmware version: %u.%u.%u, message size: %u", fw_version.major, fw_version.minor, fw_version.patch, message_size);

  switch (format) {
  case bladerf_format::BLADERF_FORMAT_SC16_Q11_META:
    meta_size = 16;
    bytes_per_sample = 4;
    break;
  case bladerf_format::BLADERF_FORMAT_SC16_Q11:
    meta_size = 0;
    bytes_per_sample = 4;
    break;
  case bladerf_format::BLADERF_FORMAT_SC16_Q11_PACKED_META:
    meta_size = 32;
    bytes_per_sample = 3;
    break;
  case bladerf_format::BLADERF_FORMAT_SC16_Q11_PACKED:
    meta_size = 0;
    bytes_per_sample = 3;
    break;
  case bladerf_format::BLADERF_FORMAT_SC8_Q7_META:
    meta_size = 16;
    bytes_per_sample = 2;
    break;
  case bladerf_format::BLADERF_FORMAT_SC8_Q7:
    meta_size = 0;    
    bytes_per_sample = 2;
    break;
  case bladerf_format::BLADERF_FORMAT_PACKET_META:
    break;
  }

  tx.valid_samples_per_buffer = tx.samples_per_buffer - meta_size * tx.samples_per_buffer / message_size;
  rx.valid_samples_per_buffer = rx.samples_per_buffer - meta_size * rx.samples_per_buffer / message_size;

  LOG("Rx samples per buffer: %u, valid Rx samples per buffer: %u", rx.samples_per_buffer, rx.valid_samples_per_buffer);
  LOG("Tx samples per buffer: %u, valid Tx samples per buffer: %u", tx.samples_per_buffer, tx.valid_samples_per_buffer);

  skip_buffers = start_delay * rate / rx.valid_samples_per_buffer / rx.nof_streams;

  if (rx.nof_streams > 0) {
    LOG("Initializing Rx stream...");
    CHECK(bladerf_init_stream(&rx.stream, device, rx_stream_cb, &rx.buffers, rx.nof_buffers, format,
                              rx.samples_per_buffer, rx.nof_transfers, nullptr));

    CHECK(bladerf_set_stream_timeout(device, bladerf_direction::BLADERF_RX, 1000));

    LOG("Enabling Rx...");
    for (unsigned i = 0; i < rx.nof_streams; i++) {
      CHECK(bladerf_enable_module(device, BLADERF_CHANNEL_RX(i), true));
    }

    LOG("Starting Rx stream...");
    rx.thread = std::thread([]() {
      ::pthread_setname_np(::pthread_self(), "bladeRF-Rx");

      ::sched_param param{::sched_get_priority_max(SCHED_FIFO) - 2};
      ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &param);

      cpu_set_t cpu_set{0};
      CPU_SET(19, &cpu_set);
      ::pthread_setaffinity_np(::pthread_self(), sizeof(cpu_set), &cpu_set);

      bladerf_stream(rx.stream, rx.nof_streams == 1 ? BLADERF_RX_X1 : BLADERF_RX_X2);
    });
  }

  if (tx.nof_streams > 0) {
    LOG("Initializing Tx stream...");
    CHECK(bladerf_init_stream(&tx.stream, device, tx_stream_cb, &tx.buffers, tx.nof_buffers, format,
                              tx.samples_per_buffer, tx.nof_transfers, nullptr));

    CHECK(bladerf_set_stream_timeout(device, bladerf_direction::BLADERF_TX, 1000));

    LOG("Enabling Tx...");
    for (unsigned i = 0; i < tx.nof_streams; i++) {
      CHECK(bladerf_enable_module(device, BLADERF_CHANNEL_TX(i), true));
    }

    LOG("Starting Tx stream...");
    tx.thread = std::thread([]() {
      ::pthread_setname_np(::pthread_self(), "bladeRF-Tx");

      ::sched_param param{::sched_get_priority_max(SCHED_FIFO) - 2};
      ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &param);

      cpu_set_t cpu_set{0};
      CPU_SET(20, &cpu_set);
      ::pthread_setaffinity_np(::pthread_self(), sizeof(cpu_set), &cpu_set);

      bladerf_stream(tx.stream, tx.nof_streams == 1 ? BLADERF_TX_X1 : BLADERF_TX_X2);
    });

    for (unsigned i = 0; i < tx.nof_transfers; i++) {
      bladerf_submit_stream_buffer_nb(tx.stream, tx.buffers[tx.buffer_index]);
      tx.buffer_index = (tx.buffer_index + 1) % tx.nof_buffers;
    }
  }
}

void device_fini() {
  if (rx.nof_streams > 0) {
    LOG("Stopping Rx streams...");
    bladerf_submit_stream_buffer_nb(rx.stream, BLADERF_STREAM_SHUTDOWN);
    rx.shutdown = true;
    if (rx.thread.joinable()) {
      rx.thread.join();
    }
  }
  if (tx.nof_streams > 0) {
    LOG("Stopping Tx streams...");
    bladerf_submit_stream_buffer_nb(tx.stream, BLADERF_STREAM_SHUTDOWN);
    tx.shutdown = true;
    if (tx.thread.joinable()) {
      tx.thread.join();
    }
  }

  if (rx.nof_streams > 0) {
    LOG("Deinitializing Rx streams...");
    bladerf_deinit_stream(rx.stream);

    LOG("Disabling Rx...");
    for (unsigned i = 0; i < rx.nof_streams; i++) {
      CHECK(bladerf_enable_module(device, BLADERF_CHANNEL_RX(i), false));
    }
  }
  if (tx.nof_streams > 0) {
    LOG("Deinitializing Tx streams...");
    bladerf_deinit_stream(tx.stream);

    LOG("Disabling Tx...");
    for (unsigned i = 0; i < tx.nof_streams; i++) {
      CHECK(bladerf_enable_module(device, BLADERF_CHANNEL_TX(i), false));
    }
  }

  LOG("Closing bladeRF...");
  bladerf_close(device);
}

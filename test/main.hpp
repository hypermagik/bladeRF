#pragma once

#include <libbladeRF.h>

#include <stdio.h>
#include <thread>

inline int verbose_count = 0;
inline bladerf_log_level rf_log_level = bladerf_log_level::BLADERF_LOG_LEVEL_INFO;
inline bladerf_tuning_mode tuning_mode = BLADERF_TUNING_MODE_HOST;
inline unsigned int rate = 61.44e6;
inline unsigned int bw = 56e6;
inline int gain = -99;
inline unsigned int frequency = 768e6;
inline bladerf_format format = bladerf_format::BLADERF_FORMAT_SC16_Q11_PACKED;
inline bool has_meta = false;
inline unsigned bytes_per_sample = 0;
inline bool rx_all_events = false;
inline bladerf_rx_mux rx_mux = bladerf_rx_mux::BLADERF_RX_MUX_12BIT_COUNTER;
inline bool tx_timestamps = false;
inline unsigned duration = 5;
inline int skip_buffers = 0;
inline double start_delay = 0.1;
inline double tx_delay = 0.01;

inline struct bladerf *device = nullptr;

inline struct Rx {
  unsigned nof_streams = 1;

  const unsigned nof_buffers = 64;
  const unsigned samples_per_buffer = 8192;
  unsigned valid_samples_per_buffer = 8192;
  const unsigned nof_transfers = 32;

  bool shutdown = false;

  std::thread thread;
  struct bladerf_stream *stream;

  void **buffers;
  unsigned buffer_index = 0;

  uint64_t ts = 0;

  uint64_t t_first = 0;
  uint64_t t_last = 0;
  uint64_t n_total = 0;
  uint64_t n_valid = 0;
  uint64_t n_lost = 0;
  uint64_t n_dup = 0;
  uint64_t n_disc = 0;
} rx;

inline struct Tx {
  unsigned nof_streams = 1;

  const unsigned nof_buffers = 64;
  const unsigned samples_per_buffer = 8192;
  unsigned valid_samples_per_buffer = 8192;
  const unsigned nof_transfers = 32;

  bool shutdown = false;

  std::thread thread;
  struct bladerf_stream *stream;

  void **buffers;
  unsigned buffer_index = 0;

  uint64_t ts = 0;

  uint64_t t_first = 0;
  uint64_t t_last = 0;
  uint64_t n_total = 0;
  uint64_t n_valid = 0;
  uint64_t n_disc = 0;
} tx;

#define LOG_PREFIX "\033[1m\033[32m[bladeRF]\033[0m "
#define LOG(fmt, ...)                                                                                              \
  printf("[%ld] " LOG_PREFIX fmt "\n", std::chrono::steady_clock::now().time_since_epoch().count(), ##__VA_ARGS__)

extern void parse_opts(int argc, char **argv);

extern void device_init();
extern void device_fini();

extern void *rx_stream_cb(struct bladerf *dev, struct bladerf_stream *stream, struct bladerf_metadata *meta,
                          void *samples, size_t nof_samples, void *user_data);
extern void *tx_stream_cb(struct bladerf *dev, struct bladerf_stream *stream, struct bladerf_metadata *meta,
                          void *samples, size_t nof_samples, void *user_data);

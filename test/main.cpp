#include "main.hpp"

#include <cassert>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
  parse_opts(argc, argv);

  device_init();

  std::this_thread::sleep_for(std::chrono::seconds(duration));

  device_fini();

  const uint64_t t_rx = rx.t_last - rx.t_first;
  const double rx_rate = t_rx == 0 ? 0 : rx.n_total * 1e9 / t_rx;
  const double rx_rate_without_meta = t_rx == 0 ? 0 : rx.n_valid * 1e9 / t_rx;

  printf("Rx:\n");
  printf("- %ju bytes\n", rx.n_total * bytes_per_sample);
  printf("- %ju samples\n", rx.n_total);
  if (has_meta) {
    printf("- %ju samples lost (%.2f%%)\n", rx.n_lost, 100.0 * rx.n_lost / (rx.n_lost + rx.n_total));
    printf("- %ju samples duplicated (%.2f%%)\n", rx.n_dup, 100.0 * rx.n_dup / (rx.n_lost + rx.n_total));
    printf("- %ju discontinuities\n", rx.n_disc);
    printf("- %.2f Msps (%.2f Msps raw)\n", rx_rate_without_meta / 1e6, rx_rate / 1e6);
  } else {
    printf("- %.2f Msps\n", rx_rate / 1e6);
  }
  printf("- %.2f MBps / %.2f Mbps\n", rx_rate * bytes_per_sample / 1e6, 8 * rx_rate * bytes_per_sample / 1e6);

  const uint64_t t_tx = tx.t_last - tx.t_first;
  const double tx_rate = t_tx == 0 ? 0 : tx.n_total * 1e9 / t_tx;
  const double tx_rate_without_meta = t_tx == 0 ? 0 : tx.n_valid * 1e9 / t_tx;

  printf("Tx:\n");
  printf("- %ju bytes\n", tx.n_total * bytes_per_sample);
  printf("- %ju samples\n", tx.n_total);
  if (has_meta) {
    printf("- %ju discontinuities\n", tx.n_disc);
    printf("- %.2f Msps (%.2f Msps raw)\n", tx_rate_without_meta / 1e6, tx_rate / 1e6);
  } else {
    printf("- %.2f Msps\n", tx_rate / 1e6);
  }
  printf("- %.2f MBps / %.2f Mbps\n", tx_rate * bytes_per_sample / 1e6, 8 * tx_rate * bytes_per_sample / 1e6);

  return 0;
}

void *rx_stream_cb(struct bladerf *dev, struct bladerf_stream *stream, struct bladerf_metadata *meta, void *samples,
                   size_t nof_samples, void *user_data) {
  if (rx.shutdown) {
    LOG("Rx stream shutdown");
    return BLADERF_STREAM_SHUTDOWN;
  }

  if (samples == nullptr) {
    LOG("Received no data");
    return BLADERF_STREAM_NO_DATA;
  }

  if (skip_buffers > 0) {
    if (--skip_buffers == 0) {
      LOG("Running...");
    }

    void *result = rx.buffers[rx.buffer_index];
    rx.buffer_index = (rx.buffer_index + 1) % rx.nof_buffers;

    return result;
  }

  rx.t_last = std::chrono::high_resolution_clock::now().time_since_epoch().count();

  if (rx.t_first == 0) {
    rx.t_first = rx.t_last;
  }

  assert(nof_samples == rx.samples_per_buffer);

  rx.n_total += rx.samples_per_buffer;
  rx.n_valid += rx.valid_samples_per_buffer;

  auto *bytes = reinterpret_cast<uint8_t *>(samples);

  const uint64_t ts = has_meta ? *reinterpret_cast<const uint64_t *>(bytes + 4) : 0;
  const uint16_t flags = has_meta ? *reinterpret_cast<const uint16_t *>(bytes + 12) : 0;

  if (flags & BLADERF_META_FLAG_RX_HW_UNDERFLOW) {
    if (tx.ts != 0) {
      LOG("Tx underflow detected");
      tx.ts = ts + rate * tx_delay;
      tx.n_disc += 1;
    }
  }

  if (verbose_count > 0) {
    LOG("Received %zu samples, ts=%ju - "
        "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"
        "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x...",
        nof_samples, ts, bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8],
        bytes[9], bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15], bytes[16], bytes[17], bytes[18],
        bytes[19], bytes[20], bytes[21], bytes[22], bytes[23], bytes[24], bytes[25], bytes[26], bytes[27], bytes[28],
        bytes[29], bytes[30], bytes[31]);

#if 0
    for (int i = 0; i < nof_samples * bytes_per_sample; i++) {
      printf("%02x ", bytes[i]);
      if (i % 32 == 31) {
        printf("\n");
      }
    }
#endif
  }

  if (has_meta) {
    if (rx.ts != 0) {
      const uint64_t next_ts = rx.ts + rx.valid_samples_per_buffer / rx.nof_streams;
      if (ts != next_ts) {
        if (ts > next_ts) {
          rx.n_lost += ts - next_ts;
          rx.n_disc += 1;
          if (verbose_count > 0) {
            LOG("Lost %ld samples", ts - next_ts);
          }
        } else {
          rx.n_dup += next_ts - ts;
          rx.n_disc += 1;
          if (verbose_count > 0) {
            LOG("Duplicated %ld samples", next_ts - ts);
          }
        }
      }
    }

    rx.ts = ts;

    if (tx.ts == 0) {
      tx.ts = ts + tx.nof_streams * rate * tx_delay;
    }
  }

  if (verbose_count > 0) {
    verbose_count--;
  }

  void *result = rx.buffers[rx.buffer_index];
  rx.buffer_index = (rx.buffer_index + 1) % rx.nof_buffers;

  return result;
}

void *tx_stream_cb(struct bladerf *dev, struct bladerf_stream *stream, struct bladerf_metadata *meta, void *samples,
                   size_t nof_samples, void *user_data) {
  if (tx.shutdown) {
    LOG("Tx stream shutdown");
    return BLADERF_STREAM_SHUTDOWN;
  }

  if (samples == nullptr) {
    return BLADERF_STREAM_NO_DATA;
  }

  if (skip_buffers > 0) {
    void *result = tx.buffers[tx.buffer_index];
    tx.buffer_index = (tx.buffer_index + 1) % tx.nof_buffers;

    return result;
  }

  tx.t_last = std::chrono::high_resolution_clock::now().time_since_epoch().count();

  if (tx.t_first == 0) {
    tx.t_first = tx.t_last;
  }

  assert(nof_samples == tx.samples_per_buffer);

  tx.n_total += tx.samples_per_buffer;
  tx.n_valid += tx.valid_samples_per_buffer;

  void *result = tx.buffers[tx.buffer_index];
  tx.buffer_index = (tx.buffer_index + 1) % tx.nof_buffers;

  if (has_meta && tx_timestamps) {
    const size_t num_messages = tx.samples_per_buffer * bytes_per_sample / message_size;

    auto *bytes = reinterpret_cast<uint8_t *>(result);

    for (size_t i = 0; i < num_messages; i++) {
      auto *ts_ptr = reinterpret_cast<uint64_t *>(bytes + 4);

      *ts_ptr = tx.ts;

      if (tx.ts != 0) {
        tx.ts += (message_size - meta_size) / bytes_per_sample / tx.nof_streams;
      }

      bytes += message_size;
    }
  }

  return result;
}

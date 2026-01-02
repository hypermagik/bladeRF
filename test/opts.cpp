#include "main.hpp"

#include <getopt.h>
#include <stdio.h>
#include <string_view>

static const struct option long_options[] = {
    {"duration", required_argument, NULL, 'd'},
    {"rate", required_argument, NULL, 'R'},
    {"bitmode", required_argument, NULL, 'B'},
    {"rx-streams", required_argument, NULL, 'r'},
    {"tx-streams", required_argument, NULL, 't'},
    {"sync-tx", no_argument, NULL, 's'},
    {"verbose", no_argument, NULL, 'v'},
    {"buf-verbose", required_argument, NULL, 'V'},
    {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0},
};

static void usage(const char *argv0) {
  printf("Usage: %s [options]\n", argv0);
  printf("  -d, --duration <n>        Duration, in seconds\n");
  printf("  -R, --rate <n>            Sampling rate, in MHz\n");
  printf("                              default: 61.44\n");
  printf("  -B, --bitmode <mode>      Sample size, in bits\n");
  printf("                              <16|16m|12 (default)|12m|8|8m>\n");
  printf("  -r, --rx-streams <n>      Number of Rx streams\n");
  printf("                              <0|1 (default)|2>\n");
  printf("  -t, --tx-streams <n>      Number of Tx streams\n");
  printf("                              <0|1 (default)|2>\n");
  printf("  -s, --sync-tx             Use timestamps for transmission\n");
  printf("  -v, --verbose             Enable verbose bladeRF log level\n");
  printf("  -V, --buf-verbose <n>     Set verbosity for first n buffers\n");
  printf("  -h, --help                Show this text.\n");
}

void parse_opts(int argc, char **argv) {
  int opt = 0;
  int opt_ind = 0;
  while (opt != -1) {
    opt = getopt_long(argc, argv, "d:R:B:r:t:sv:V:h", long_options, &opt_ind);

    switch (opt) {
    case 'd':
      duration = atoi(optarg);
      break;
    case 'R':
      rate = atof(optarg) * 1e6;
      break;
    case 'B':
      if (std::string_view(optarg) == "16") {
        format = BLADERF_FORMAT_SC16_Q11;
      } else if (std::string_view(optarg) == "16m") {
        format = BLADERF_FORMAT_SC16_Q11_META;
        has_meta = true;
      } else if (std::string_view(optarg) == "12") {
        format = BLADERF_FORMAT_SC16_Q11_PACKED;
      } else if (std::string_view(optarg) == "12m") {
        format = BLADERF_FORMAT_SC16_Q11_PACKED_META;
        has_meta = true;
      } else if (std::string_view(optarg) == "8") {
        format = BLADERF_FORMAT_SC8_Q7;
      } else if (std::string_view(optarg) == "8m") {
        format = BLADERF_FORMAT_SC8_Q7_META;
        has_meta = true;
      } else {
        printf("Unknown bitmode: %s\n", optarg);
        exit(1);
      }
      break;
    case 'r':
      rx.nof_streams = atoi(optarg);
      if (rx.nof_streams > 2) {
        printf("Invalid number of Rx streams: %u\n", rx.nof_streams);
        exit(1);
      }
      break;
    case 't':
      tx.nof_streams = atoi(optarg);
      if (tx.nof_streams > 2) {
        printf("Invalid number of Tx streams: %u\n", tx.nof_streams);
        exit(1);
      }
      break;
    case 's':
      tx_timestamps = true;
      break;
    case 'v':
      rf_log_level = bladerf_log_level::BLADERF_LOG_LEVEL_VERBOSE;
      break;
    case 'V':
      verbose_count = atoi(optarg);
      break;
    case 'h':
      usage(argv[0]);
      exit(0);
    default:
      break;
    }
  }
}

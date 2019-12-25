#define MS_PER_HOUR 3600000UL
#define S_PER_HOUR 3600U

#define DIV_ROUND_CLOSEST(n, d) \
  ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d) / 2) / (d)) : (((n) + (d) / 2) / (d)))

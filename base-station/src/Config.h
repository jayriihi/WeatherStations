#ifndef CONFIG_H
#define CONFIG_H

#ifndef LORA_FREQ
#define LORA_FREQ 915000000UL
#endif

#ifndef LORA_TX_POWER
#define LORA_TX_POWER 14
#endif

// Convert center frequency from Hz to MHz for RadioLib APIs.
static inline float loraFreqMHz() {
  return (float)LORA_FREQ / 1000000.0f;
}

#endif  // CONFIG_H

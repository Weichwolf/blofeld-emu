// SPDX-License-Identifier: GPL-3.0-only
#include "runtime/instrument.hpp"

namespace blofeld::runtime {
void Instrument::publish_io() {
  update_u0_drive();
  observe_lcd_epoch();
  update_loopback();
  if (native_->gpio_peer) { native_->gpio_peer->synchronize(); }
}
}

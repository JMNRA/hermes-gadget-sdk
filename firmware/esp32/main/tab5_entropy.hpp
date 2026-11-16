#pragma once

namespace hgp {
// The hardware entropy source is only available while seeding.
template<class Enable, class Seed, class Disable>
bool tab5_seed_entropy(Enable enable, Seed seed, Disable disable) {
  enable();
  const bool ready = seed();
  disable();
  return ready;
}
}

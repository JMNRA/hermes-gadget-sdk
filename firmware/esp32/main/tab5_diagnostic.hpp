#pragma once

#include <cstddef>
#include <cstdint>

#include "hg/ui.hpp"

namespace hgp {

// Local model only: never construct App or imply a gateway connection.
inline hg::UiModel tab5_diagnostic_preview() {
  hg::UiModel model;
  model.screen = hg::Screen::Offline;
  model.link = hg::Link::Offline;
  model.title = "Hermes - LOCAL PREVIEW";
  model.hero = true;
  model.headline = "Tab5 diagnostic";
  model.detail = "Offline preview - no network or audio";
  model.hint = "Touch to show coordinates";
  model.settings_hold = false;
  return model;
}

inline void tab5_diagnostic_touch(hg::UiModel& model, bool touching, int x, int y) {
  model.hint = std::string("Touch ") + (touching ? "down/move" : "up") +
               " x=" + std::to_string(x) + " y=" + std::to_string(y);
}

// Caller owns a width*height RGB565 framebuffer. A white perimeter makes
// clipping visible; red/green/blue bars expose channel and orientation errors.
inline bool tab5_diagnostic_pattern(uint16_t* pixels, uint16_t width, uint16_t height) {
  if (!pixels || !width || !height) return false;
  constexpr uint16_t colors[] = {0xf800, 0x07e0, 0x001f};
  for (size_t y = 0; y < height; ++y)
    for (size_t x = 0; x < width; ++x)
      pixels[y * width + x] =
          (x == 0 || y == 0 || x + 1 == width || y + 1 == height)
              ? 0xffff : colors[x * 3 / width];
  return true;
}

}  // namespace hgp

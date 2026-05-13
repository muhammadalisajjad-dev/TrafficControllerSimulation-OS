/*
 * SFML 2.6 real-time view — dedicated render thread (pollEvent + ~50 FPS pace).
 * Does not touch parking semaphores or pipes; reads traffic phases under shm->mtx briefly.
 */
#include "gui.h"

#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>

#include <pthread.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr unsigned WIN_W = 1000;
constexpr unsigned WIN_H = 720;
constexpr float F10_CX = 190.f;
constexpr float F10_CY = 310.f;
constexpr float F11_CX = 810.f;
constexpr float F11_CY = 310.f;
constexpr float IX_HALF = 110.f;

std::thread g_render_thread;
std::atomic<bool> g_run_render{true};
std::atomic<bool> g_thread_started{false};

struct shared_state *g_shm = nullptr;

std::mutex g_gui_mtx;
struct VizVeh {
  float x = -500.f;
  float y = -500.f;
  float tx = -500.f;
  float ty = -500.f;
  int vt = 0;
  int visible = 0;
  int need_snap = 1;
} g_veh[NUM_VEHICLES + 1];

std::deque<std::string> g_emergency_lines;
constexpr size_t kMaxEmLines = 8;

void push_emergency_line(const std::string &s) {
  std::lock_guard<std::mutex> lk(g_gui_mtx);
  g_emergency_lines.push_back(s);
  while (g_emergency_lines.size() > kMaxEmLines)
    g_emergency_lines.pop_front();
}

float ix_cx(enum intersection_id ix) { return ix == IX_F10 ? F10_CX : F11_CX; }
float ix_cy(enum intersection_id ix) { return ix == IX_F10 ? F10_CY : F11_CY; }

} /* namespace */

extern "C" void gui_layout_spawn(enum intersection_id ix, enum approach_dir ap, float *out_x,
                                 float *out_y) {
  const float cx = ix_cx(ix);
  const float cy = ix_cy(ix);
  switch (ap) {
  case AP_N:
    *out_x = cx;
    *out_y = cy - IX_HALF - 50.f;
    break;
  case AP_S:
    *out_x = cx;
    *out_y = cy + IX_HALF + 50.f;
    break;
  case AP_E:
    *out_x = cx + IX_HALF + 50.f;
    *out_y = cy;
    break;
  case AP_W:
    *out_x = cx - IX_HALF - 50.f;
    *out_y = cy;
    break;
  default:
    *out_x = cx;
    *out_y = cy;
    break;
  }
}

extern "C" void gui_layout_center(enum intersection_id ix, float *out_x, float *out_y) {
  *out_x = ix_cx(ix);
  *out_y = ix_cy(ix);
}

extern "C" void gui_layout_post_cross(enum intersection_id ix, enum approach_dir origin,
                                      enum dest_rel dr, float *out_x, float *out_y) {
  const float cx = ix_cx(ix);
  const float cy = ix_cy(ix);
  if (dr == DEST_STRAIGHT) {
    switch (origin) {
    case AP_N:
      *out_x = cx;
      *out_y = cy + IX_HALF + 60.f;
      break;
    case AP_S:
      *out_x = cx;
      *out_y = cy - IX_HALF - 60.f;
      break;
    case AP_E:
      *out_x = cx - IX_HALF - 60.f;
      *out_y = cy;
      break;
    case AP_W:
      *out_x = cx + IX_HALF + 60.f;
      *out_y = cy;
      break;
    default:
      *out_x = cx;
      *out_y = cy;
      break;
    }
  } else if (dr == DEST_LEFT) {
    switch (origin) {
    case AP_N:
      *out_x = cx - IX_HALF - 60.f;
      *out_y = cy;
      break;
    case AP_S:
      *out_x = cx + IX_HALF + 60.f;
      *out_y = cy;
      break;
    case AP_E:
      *out_x = cx;
      *out_y = cy - IX_HALF - 60.f;
      break;
    case AP_W:
      *out_x = cx;
      *out_y = cy + IX_HALF + 60.f;
      break;
    default:
      *out_x = cx;
      *out_y = cy;
      break;
    }
  } else {
    switch (origin) {
    case AP_N:
      *out_x = cx + IX_HALF + 60.f;
      *out_y = cy;
      break;
    case AP_S:
      *out_x = cx - IX_HALF - 60.f;
      *out_y = cy;
      break;
    case AP_E:
      *out_x = cx;
      *out_y = cy + IX_HALF + 60.f;
      break;
    case AP_W:
      *out_x = cx;
      *out_y = cy - IX_HALF - 60.f;
      break;
    default:
      *out_x = cx;
      *out_y = cy;
      break;
    }
  }
}

extern "C" void gui_layout_parking_spot(enum intersection_id ix, int slot_index, float *out_x,
                                      float *out_y) {
  const float cx = ix_cx(ix);
  const float cy = ix_cy(ix);
  int s = slot_index;
  if (s < 0)
    s = 0;
  if (s >= PARKING_SPOTS)
    s = PARKING_SPOTS - 1;
  *out_x = cx - 100.f + (float)s * 22.f;
  *out_y = cy + IX_HALF + 95.f;
}

extern "C" void gui_layout_connector(float t, float *out_x, float *out_y) {
  if (t < 0.f)
    t = 0.f;
  if (t > 1.f)
    t = 1.f;
  *out_x = F10_CX + (F11_CX - F10_CX) * t;
  *out_y = F10_CY - 20.f;
}

extern "C" void gui_layout_parking_queue(enum intersection_id ix, float *out_x, float *out_y) {
  *out_x = ix_cx(ix) - 40.f;
  *out_y = ix_cy(ix) + IX_HALF + 125.f;
}

static sf::Color color_for_vt(int vt) {
  switch (vt) {
  case VT_AMBULANCE:
  case VT_FIRETRUCK:
    return sf::Color(220, 40, 40);
  case VT_BUS:
    return sf::Color(230, 200, 40);
  case VT_CAR:
    return sf::Color(60, 180, 80);
  case VT_BIKE:
    return sf::Color(80, 200, 220);
  case VT_TRACTOR:
    return sf::Color(139, 90, 43);
  default:
    return sf::Color(180, 180, 180);
  }
}

/* Values must match enum light_phase in shared_state.h (0=NS, 1=EW, 2=ALL_RED). */
static void draw_lights(sf::RenderWindow &win, enum intersection_id ix, int phase_int) {
  const float cx = ix_cx(ix);
  const float cy = ix_cy(ix);
  const float r = 10.f;

  auto bulb = [&](float x, float y, sf::Color c) {
    sf::CircleShape csh(r);
    csh.setOrigin(r, r);
    csh.setPosition(x, y);
    csh.setFillColor(c);
    win.draw(csh);
  };

  const int ph = phase_int;
  const bool ns_on = (ph == 0);
  const bool ew_on = (ph == 1);
  const bool all_red = (ph == 2);

  sf::Color ns_col = all_red ? sf::Color::Red : (ns_on ? sf::Color::Green : sf::Color::Red);
  sf::Color ew_col = all_red ? sf::Color::Red : (ew_on ? sf::Color::Green : sf::Color::Red);

  bulb(cx, cy - IX_HALF - 15.f, ns_col);
  bulb(cx, cy + IX_HALF + 15.f, ns_col);
  bulb(cx + IX_HALF + 15.f, cy, ew_col);
  bulb(cx - IX_HALF - 15.f, cy, ew_col);
}

static void draw_ix_box(sf::RenderWindow &win, sf::Font *font, enum intersection_id ix,
                        const char *label) {
  const float cx = ix_cx(ix);
  const float cy = ix_cy(ix);
  sf::RectangleShape box(sf::Vector2f(IX_HALF * 2.f, IX_HALF * 2.f));
  box.setOrigin(IX_HALF, IX_HALF);
  box.setPosition(cx, cy);
  box.setFillColor(sf::Color(50, 55, 70, 120));
  box.setOutlineColor(sf::Color(200, 200, 200));
  box.setOutlineThickness(2.f);
  win.draw(box);

  if (font) {
    sf::Text t;
    t.setFont(*font);
    t.setString(label);
    t.setCharacterSize(18);
    t.setFillColor(sf::Color::White);
    t.setPosition(cx - 22.f, cy - IX_HALF - 40.f);
    win.draw(t);
  }
}

static void draw_parking(sf::RenderWindow &win, sf::Font *font, struct shared_state *shm,
                         enum intersection_id ix) {
  int spots_free = gui_c_park_spots_free(shm, ix);
  int qfree = gui_c_park_queue_slots_free(shm, ix);
  int occ = PARKING_SPOTS - spots_free;
  if (occ < 0)
    occ = 0;
  if (occ > PARKING_SPOTS)
    occ = PARKING_SPOTS;
  int qwait = PARKING_QUEUE_SLOTS - qfree;
  if (qwait < 0)
    qwait = 0;

  const float cx = ix_cx(ix);
  const float base_y = ix_cy(ix) + IX_HALF + 70.f;

  for (int i = 0; i < PARKING_SPOTS; i++) {
    sf::RectangleShape slot(sf::Vector2f(18.f, 14.f));
    slot.setPosition(cx - 100.f + (float)i * 22.f, base_y);
    bool filled = i < occ;
    slot.setFillColor(filled ? sf::Color(200, 60, 60) : sf::Color(60, 180, 90));
    slot.setOutlineColor(sf::Color::White);
    slot.setOutlineThickness(1.f);
    win.draw(slot);
  }

  if (font) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "Queue waiting: %d / %d", qwait, PARKING_QUEUE_SLOTS);
    sf::Text t;
    t.setFont(*font);
    t.setString(buf);
    t.setCharacterSize(14);
    t.setFillColor(sf::Color(230, 230, 150));
    t.setPosition(cx - 100.f, base_y + 22.f);
    win.draw(t);
  }
}

static void render_loop() {
  /* Avoid crashing the whole simulation when headless (CI, SSH without X11). */
  if (!std::getenv("DISPLAY")) {
    std::cerr << "[GUI] DISPLAY not set — SFML window disabled (simulation continues).\n";
    return;
  }

  sf::RenderWindow window;
  try {
    window.create(sf::VideoMode(WIN_W, WIN_H), "F10 / F11 Traffic (SFML 2.6)");
  } catch (const std::exception &e) {
    std::cerr << "[GUI] Window create failed: " << e.what() << "\n";
    return;
  }
  if (!window.isOpen()) {
    std::cerr << "[GUI] Window not open — SFML disabled (simulation continues).\n";
    return;
  }
  window.setFramerateLimit(120);

  sf::Font font;
  bool font_ok = font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
  sf::Font *fp = font_ok ? &font : nullptr;

  g_run_render = true;

  while (g_run_render.load() && window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        if (g_shm)
          gui_c_broadcast_shutdown(g_shm);
        window.close();
      }
    }

    if (g_shm && gui_c_shutdown_requested(g_shm))
      window.close();

    int ph0 = 0, ph1 = 0;
    if (g_shm) {
      ph0 = gui_c_ix_phase(g_shm, IX_F10);
      ph1 = gui_c_ix_phase(g_shm, IX_F11);
    }

    {
      std::lock_guard<std::mutex> lk(g_gui_mtx);
      for (int i = 1; i <= NUM_VEHICLES; i++) {
        if (!g_veh[i].visible)
          continue;
        g_veh[i].x += (g_veh[i].tx - g_veh[i].x) * 0.18f;
        g_veh[i].y += (g_veh[i].ty - g_veh[i].y) * 0.18f;
      }
    }

    window.clear(sf::Color(30, 32, 40));

    sf::RectangleShape road(sf::Vector2f(F11_CX - F10_CX, 36.f));
    road.setPosition(F10_CX, F10_CY - 18.f);
    road.setFillColor(sf::Color(70, 70, 80));
    window.draw(road);

    if (g_shm) {
      draw_ix_box(window, fp, IX_F10, "F10");
      draw_ix_box(window, fp, IX_F11, "F11");
      draw_lights(window, IX_F10, ph0);
      draw_lights(window, IX_F11, ph1);
      draw_parking(window, fp, g_shm, IX_F10);
      draw_parking(window, fp, g_shm, IX_F11);
    }

    {
      std::lock_guard<std::mutex> lk(g_gui_mtx);
      for (int i = 1; i <= NUM_VEHICLES; i++) {
        if (!g_veh[i].visible)
          continue;
        sf::CircleShape c(11.f);
        c.setOrigin(11.f, 11.f);
        c.setPosition(g_veh[i].x, g_veh[i].y);
        c.setFillColor(color_for_vt(g_veh[i].vt));
        c.setOutlineColor(sf::Color::White);
        c.setOutlineThickness(1.f);
        window.draw(c);
        if (fp) {
          char idb[8];
          std::snprintf(idb, sizeof(idb), "%d", i);
          sf::Text lab;
          lab.setFont(*fp);
          lab.setString(idb);
          lab.setCharacterSize(11);
          lab.setFillColor(sf::Color::White);
          lab.setPosition(g_veh[i].x - 4.f, g_veh[i].y - 6.f);
          window.draw(lab);
        }
      }

      float y = 12.f;
      if (fp) {
        for (const auto &ln : g_emergency_lines) {
          sf::Text te;
          te.setFont(*fp);
          te.setString(ln);
          te.setCharacterSize(13);
          te.setFillColor(sf::Color(255, 120, 120));
          te.setPosition(12.f, y);
          window.draw(te);
          y += 16.f;
        }
      }
    }

    if (fp) {
      sf::Text hint;
      hint.setFont(*fp);
      hint.setString("Close window or Ctrl+C in terminal to end.");
      hint.setCharacterSize(12);
      hint.setFillColor(sf::Color(160, 160, 160));
      hint.setPosition(12.f, (float)WIN_H - 28.f);
      window.draw(hint);
    }

    window.display();
    sf::sleep(sf::milliseconds(20)); /* ~50 FPS; keeps GUI responsive */
  }
}

extern "C" void gui_start_render_thread(struct shared_state *shm) {
  g_shm = shm;
  g_run_render = true;
  g_render_thread = std::thread([] { render_loop(); });
  g_thread_started = true;
}

extern "C" void gui_stop_join(void) {
  g_run_render = false;
  if (g_thread_started.load()) {
    if (g_render_thread.joinable())
      g_render_thread.join();
    g_thread_started = false;
  }
}

extern "C" void gui_vehicle_target(int vehicle_id, float x, float y, int vehicle_type, int visible) {
  if (vehicle_id < 1 || vehicle_id > NUM_VEHICLES)
    return;
  std::lock_guard<std::mutex> lk(g_gui_mtx);
  VizVeh &v = g_veh[vehicle_id];
  v.tx = x;
  v.ty = y;
  v.vt = vehicle_type;
  if (visible) {
    v.visible = 1;
    if (v.need_snap) {
      v.x = x;
      v.y = y;
      v.need_snap = 0;
    }
  } else {
    v.visible = 0;
    v.need_snap = 1;
  }
}

extern "C" void gui_emergency_line(const char *text) {
  if (!text)
    return;
  push_emergency_line(std::string(text));
}

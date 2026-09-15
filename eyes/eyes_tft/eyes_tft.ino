/*
 * Robot Eyes — 2x SPI TFT on Arduino Uno
 * ---------------------------------------
 * Draws one cartoon eye on each of two SPI TFT displays and moves the
 * pupils toward a target sent over serial by the Raspberry Pi 5.
 *
 * Serial protocol (115200 baud, ASCII lines):
 *   T <px> <py>     look toward point; px,py in 0..100 (50,50 = center)
 *   T -1 -1         no person in view; pupils return to center
 *   PING            reply "PONG" (link sanity check)
 *   TYPE <l> <r>    set display controller per eye (3=GC9A01A round on the
 *                   bit-bang port, 4=GC9A01A round on the hw SPI bus),
 *                   re-initialize and re-run the color test
 *   TEST            re-run the boot color test + redraw
 *
 * Boot: prints detected controller IDs + fill timings, then paints LEFT eye
 * RED and RIGHT eye GREEN for a few seconds (the color test), then switches
 * to eye mode. During tracking a "REPAINT" line is printed every 25 iris
 * moves so the repaint cost (and thus the achievable tracking rate) is
 * measurable over serial.
 *
 * ROUND-PANEL GEOMETRY (GC9A01A 240x240 round, the boards in use):
 *   All radii are computed from the panel size at init. On the round panel
 *   the drawing is sized from the inscribed circle: the goggle ring (r = 114 on
 *   GC9A01A) is the outermost band, then the yellow eyelid, then the sclera the
 *   iris moves in. Nothing is sized for the 128x160 rectangle anymore. The iris
 *   offset is clamped so the iris always stays on the white sclera.
 *
 *   The look is the Minion goggle: metal ring with two bolts, yellow eyelid,
 *   large white sclera, small brown iris with a black pupil and a white
 *   highlight, black strap knuckle at each side.
 *
 * LEFT-EYE SPEED: the left eye has its own port (D4=SCLK, D3=MOSI) and is
 * driven by FastGc9a01 — a direct-PORTD bit-bang driver with the Adafruit
 * GC9A01A init sequence copied verbatim. That cuts a full 240x240 fill from
 * ~10 s (Adafruit software SPI) to ~0.4 s, and the incremental iris/pupil
 * repaint means a gaze move repaints only the iris region, so tracking stays
 * fluid. (Hardware MSPIM would be faster still but steals USART0 — the same
 * hardware that carries the gaze protocol — so it was rejected.)
 *
 * Wiring (see eyes/wiring.md) — NO shared signal wires:
 *   TFT #1 (LEFT eye)  own port: SCLK=D4, MOSI=D3, CS=10, DC=9,  RST=8
 *   TFT #2 (RIGHT eye) own port: SCK=D13, MOSI=D11, CS=7, DC=6,  RST=5
 *   MISO (D12) -> right eye MISO  OPTIONAL — carried, but nothing reads it now
 *   VCC + LED -> 3.3V, GND -> GND (power is common; signals are separate)
 *
 * Libraries: Adafruit GFX + Adafruit GC9A01A.  The ST7735 and ILI9341 drivers
 * were dropped to reclaim flash: they were 498 + 486 bytes of linked driver code
 * (measured with avr-nm on the ELF) for panels this robot does not have, and the
 * goggle drawing needs the room.  Nothing that talks to the robot changed.
 */

// ---- default controller (3 = GC9A01A round 240x240, the boards in use) ----
#ifndef DEFAULT_TYPE
#define DEFAULT_TYPE 3   // 3 = GC9A01A round 1.28" 240x240 on the bit-bang port
                         // 4 = the same GC9A01A with its MOSI/SCLK on the hw SPI bus (D11/D13)
#endif

#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <SPI.h>
#include <math.h>

// ---- pins: LEFT eye on its own software-SPI port, RIGHT eye on hardware SPI ----
const int L_CS = 10, L_DC = 9, L_MOSI = 3, L_SCLK = 4, L_RST = 8;   // LEFT  eye (MOSI/SCLK are PD3/PD4)
const int R_CS = 7,  R_DC = 6, R_MOSI = 11, R_SCLK = 13, R_RST = 5; // RIGHT eye (hardware SPI 11/13)

// ---- colors (RGB565): the Minion goggle ----
// The reference art is a metal goggle ring with two bolts, a yellow eyelid ring
// inside it, a large white sclera, and a small brown iris with a black pupil and
// a white highlight - plus the black strap knuckle at each side.
const uint16_t BACKG = 0x0000;   // behind the goggles, and the corners on a square panel
const uint16_t STRAP = 0x0000;   // the black strap knuckles
// The RGB565 values below were computed from the reference's colours, not
// rounded by eye: 0xBDF8 renders as (189,190,198), 0x8C92 as (140,146,148),
// 0xF648 as (247,203,66) and 0x9A44 as (156,73,33).
const uint16_t METAL = 0xBDF8;   // goggle ring, light grey   (#B9BEC2)
const uint16_t STEEL = 0x8C92;   // its outer edge              (#8A9095)
const uint16_t LID   = 0xF648;   // the yellow eyelid ring      (#F5C842)
const uint16_t WHITE = 0xFFFF;   // sclera
const uint16_t IRIS  = 0x9A44;   // brown iris                  (#9B4A24)
const uint16_t PUPIL = 0x0000;
const uint16_t RED   = 0xF800;
const uint16_t GREEN = 0x07E0;

// =============================================================================
// FastGc9a01 — GC9A01A 240x240 on direct-PORTD bit-bang SPI (left eye).
// Same init sequence as Adafruit_GC9A01A (copied verbatim), ~20x faster than
// the Adafruit software-SPI path because per-bit writes hit PORTD directly.
//
// CS DISCIPLINE: the panel ignores every byte sent while CS is high, so CS is
// asserted once per *transaction* (the init sequence, and each fill), never
// per byte.  Adafruit's driver pulses CS around every byte; holding it low for
// a whole transaction is equally correct and faster on a bit-banged bus.
// =============================================================================
class FastGc9a01 {
public:
  static const int16_t W = 240, H = 240;

  FastGc9a01(int8_t cs, int8_t dc, int8_t rst)
    : _cs(cs), _dc(dc), _rst(rst),
      _csMask(digitalPinToBitMask(cs)), _dcMask(digitalPinToBitMask(dc)) {}

  void begin() {
    pinMode(_cs, OUTPUT); pinMode(_dc, OUTPUT); pinMode(_rst, OUTPUT);
    pinMode(L_MOSI, OUTPUT); pinMode(L_SCLK, OUTPUT);
    digitalWrite(_cs, HIGH);
    digitalWrite(_rst, HIGH);
    delay(20);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(120);

    // Assert CS for the WHOLE init sequence.  Without this the GC9A01A discards
    // every command below (CS high = interface idle): no SLPOUT, no COLMOD, no
    // DISPON — the panel stays asleep and the eye shows nothing at all, no
    // matter how much pixel data is pushed afterwards.
    digitalWrite(_cs, LOW);

    // ---- Adafruit_GC9A01A init sequence (verbatim) ----
    static const uint8_t initcmd[] PROGMEM = {
      0xEF, 0,                                    // INREGEN2
      0xEB, 1, 0x14,
      0xFE, 0,                                    // INREGEN1
      0xEF, 0,
      0xEB, 1, 0x14,
      0x84, 1, 0x40,
      0x85, 1, 0xFF,
      0x86, 1, 0xFF,
      0x87, 1, 0xFF,
      0x88, 1, 0x0A,
      0x89, 1, 0x21,
      0x8A, 1, 0x00,
      0x8B, 1, 0x80,
      0x8C, 1, 0x01,
      0x8D, 1, 0x01,
      0x8E, 1, 0xFF,
      0x8F, 1, 0xFF,
      0xB6, 2, 0x00, 0x00,
      0x36, 1, 0x48,                              // MADCTL: MX | BGR
      0x3A, 1, 0x05,                              // COLMOD 16-bit
      0x90, 4, 0x08, 0x08, 0x08, 0x08,
      0xBD, 1, 0x06,
      0xBC, 1, 0x00,
      0xFF, 3, 0x60, 0x01, 0x04,
      0xC3, 1, 0x13,                              // POWER2
      0xC4, 1, 0x13,                              // POWER3
      0xC9, 1, 0x22,                              // POWER4
      0xBE, 1, 0x11,
      0xE1, 2, 0x10, 0x0E,
      0xDF, 3, 0x21, 0x0C, 0x02,
      0xF0, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A, // GAMMA1
      0xF1, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F, // GAMMA2
      0xF2, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A, // GAMMA3
      0xF3, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F, // GAMMA4
      0xED, 2, 0x1B, 0x0B,
      0xAE, 1, 0x77,
      0xCD, 1, 0x63,
      0xE8, 1, 0x34,                              // FRAMERATE
      0x62, 12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70,
      0x63, 12, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70,
      0x64, 7, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07,
      0x66, 10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00,
      0x67, 10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98,
      0x74, 7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00,
      0x98, 2, 0x3E, 0x07,
      0x35, 0,                                    // TEON
      0x21, 0,                                    // INVON
      0x11, 0x80,                                 // SLPOUT (delay 150)
      0x29, 0x80,                                 // DISPON (delay 150)
      0x00                                        // end
    };
    uint8_t cmd, x, n;
    const uint8_t *a = initcmd;
    while ((cmd = pgm_read_byte(a++)) > 0) {
      x = pgm_read_byte(a++);
      n = x & 0x7F;
      writeCmd(cmd);
      for (uint8_t i = 0; i < n; i++) writeData(pgm_read_byte(a++));
      if (x & 0x80) delay(150);
    }
    digitalWrite(_cs, HIGH);        // end of init transaction
  }

  // ---- one byte, MSB first, SPI mode 0, direct PORTD (MOSI=PD3, SCLK=PD4) ----
  // Callers must have CS already low (begin(), setAddrWindow()).
  inline void write8(uint8_t b) {
    for (uint8_t m = 0x80; m; m >>= 1) {
      if (b & m) PORTD |= (1 << 3); else PORTD &= ~(1 << 3);
      PORTD |= (1 << 4);
      PORTD &= ~(1 << 4);
    }
  }

  inline void write16(uint16_t v) { write8(v >> 8); write8(v & 0xFF); }

  void writeCmd(uint8_t c) {
    dcLow();
    write8(c);
    dcHigh();
  }

  void writeData(uint8_t b) {
    dcHigh();
    write8(b);
  }

  // CS and DC are on PORTB (they are D10 and D9), so one masked write replaces
  // digitalWrite's port lookup.  The repaint makes about ten of those calls per
  // changed span, which measured as a third of a frame's cost.
  inline void csLow()  { PORTB &= ~_csMask; }
  inline void csHigh() { PORTB |= _csMask; }
  inline void dcLow()  { PORTB &= ~_dcMask; }
  inline void dcHigh() { PORTB |= _dcMask; }

  // A batch keeps CS low across the whole frame's spans: each span writes a
  // complete window and its pixels, so the controller does not need CS raised
  // between them, and the (measured) per-span CS toggling is pure overhead.
  void beginBatch() { _batch = true; csLow(); }
  void endBatch()   { if (_batch) { _batch = false; csHigh(); } }

  void setAddrWindow(int16_t x, int16_t y, int16_t w, int16_t h) {
    if (!_batch) csLow();
    writeCmd(0x2A);           // CASET
    write16(x); write16(x + w - 1);
    writeCmd(0x2B);           // RASET
    write16(y); write16(y + h - 1);
    writeCmd(0x2C);           // RAMWR
  }

  void pushPixels(uint32_t n, uint16_t c) {
    uint8_t hi = c >> 8, lo = c & 0xFF;
    for (uint32_t i = 0; i < n; i++) { write8(hi); write8(lo); }
  }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) {
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x >= W || y >= H) return;
    if (x + w > W) w = W - x;
    if (y + h > H) h = H - y;
    if (w <= 0 || h <= 0) return;
    setAddrWindow(x, y, w, h);
    pushPixels((uint32_t)w * h, c);
    if (!_batch) csHigh();
  }

  void fillScreen(uint16_t c) { fillRect(0, 0, W, H, c); }

  void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t c) {
    if (r < 0) return;
    for (int16_t dy = -r; dy <= r; dy++) {
      int16_t dx = (int16_t)sqrt((long)r * r - (long)dy * dy);
      fillRect(x0 - dx, y0 + dy, dx * 2 + 1, 1, c);
    }
  }

  void drawPixel(int16_t x, int16_t y, uint16_t c) { fillRect(x, y, 1, 1, c); }

  void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t c) {
    int16_t f = 1 - r, ddF_x = 1, ddF_y = -2 * r, x = 0, y = r;
    drawPixel(x0, y0 + r, c); drawPixel(x0, y0 - r, c);
    drawPixel(x0 + r, y0, c); drawPixel(x0 - r, y0, c);
    while (x < y) {
      if (f >= 0) { y--; ddF_y += 2; f += ddF_y; }
      x++; ddF_x += 2; f += ddF_x;
      drawPixel(x0 + x, y0 + y, c); drawPixel(x0 - x, y0 + y, c);
      drawPixel(x0 + x, y0 - y, c); drawPixel(x0 - x, y0 - y, c);
      drawPixel(x0 + y, y0 + x, c); drawPixel(x0 - y, y0 + x, c);
      drawPixel(x0 + y, y0 - x, c); drawPixel(x0 - y, y0 - x, c);
    }
  }

private:
  int8_t _cs, _dc, _rst;
  uint8_t _csMask, _dcMask;
  bool _batch = false;
};

// =============================================================================
// Panel abstraction — the eye renderer draws through this so both eyes share
// one code path: LEFT = FastGc9a01 (fast bit-bang), RIGHT = Adafruit GFX (hw SPI).
// =============================================================================
struct Panel {
  int16_t W, H;
  virtual void fillScreen(uint16_t c) = 0;
  virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) = 0;
  virtual void fillCircle(int16_t x, int16_t y, int16_t r, uint16_t c) = 0;
  virtual void drawCircle(int16_t x, int16_t y, int16_t r, uint16_t c) = 0;
  // A batch pays the bus/transaction setup once for a whole frame's spans.
  virtual void beginBatch() {}
  virtual void endBatch() {}
  virtual ~Panel() {}
};

class FastPanel : public Panel {
public:
  FastPanel(FastGc9a01 *d) : _d(d) { W = d->W; H = d->H; }
  void fillScreen(uint16_t c) override { _d->fillScreen(c); }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override { _d->fillRect(x, y, w, h, c); }
  void fillCircle(int16_t x, int16_t y, int16_t r, uint16_t c) override { _d->fillCircle(x, y, r, c); }
  void drawCircle(int16_t x, int16_t y, int16_t r, uint16_t c) override { _d->drawCircle(x, y, r, c); }
  void beginBatch() override { _d->beginBatch(); }
  void endBatch() override { _d->endBatch(); }
private:
  FastGc9a01 *_d;
};

class GfPanel : public Panel {
public:
  GfPanel(Adafruit_SPITFT *g) : _g(g) { W = g->width(); H = g->height(); }
  void fillScreen(uint16_t c) override { _g->fillScreen(c); }
  // Adafruit's fillRect opens and closes a transaction per call; the repaint
  // makes hundreds of tiny calls a frame, so write the window directly and let
  // the batch own the transaction.
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c) override {
    if (w <= 0 || h <= 0) return;
    if (!_batch) _g->startWrite();
    _g->setAddrWindow(x, y, w, h);
    _g->writeColor(c, (uint32_t)w * (uint32_t)h);
    if (!_batch) _g->endWrite();
  }
  void beginBatch() override { if (!_batch) { _g->startWrite(); _batch = true; } }
  void endBatch() override { if (_batch) { _batch = false; _g->endWrite(); } }
  // Same integer rule as FastGc9a01, and the same rule the incremental iris
  // update assumes.  Adafruit's own fillCircle is a pixel wider on the
  // diagonals, which would leave a rim of the old iris behind on every move.
  void fillCircle(int16_t x, int16_t y, int16_t r, uint16_t c) override {
    if (r < 0) return;
    for (int16_t dy = -r; dy <= r; dy++) {
      int16_t dx = (int16_t)sqrt((long)r * r - (long)dy * dy);
      _g->fillRect(x - dx, y + dy, dx * 2 + 1, 1, c);
    }
  }
  void drawCircle(int16_t x, int16_t y, int16_t r, uint16_t c) override { _g->drawCircle(x, y, r, c); }
private:
  Adafruit_SPITFT *_g;
  bool _batch = false;
};

// ---- display objects ----
FastGc9a01       fastL(L_CS, L_DC, L_RST);                    // LEFT  eye, GC9A01A, fast bit-bang
Adafruit_GC9A01A gcR(R_CS, R_DC, R_RST);                      // RIGHT eye, GC9A01A, hardware SPI
Adafruit_GC9A01A gcL(L_CS, L_DC, L_RST);                      // LEFT  eye, GC9A01A, hardware SPI (TYPE 4)

GfPanel  gfGcR(&gcR), gfGcL(&gcL);
FastPanel fpL(&fastL);

Panel *eye[2] = {nullptr, nullptr};   // active panel per eye (0=left, 1=right)
int eyeType[2] = {DEFAULT_TYPE, DEFAULT_TYPE};

int W[2], H[2], CX[2], CY[2];         // per-eye panel geometry
int EYE_R[2], LID_R[2], SCLERA_R[2];   // goggle ring, eyelid ring, sclera
int IRIS_R[2], PUPIL_R[2], HL_R[2];    // iris, pupil, highlight
int IRIS_RR[2], PUPIL_RR[2], HL_RR[2];          // their squares, for the row walk

// ---- gaze state ----
// The app owns the smoothing: it walks the aim toward each new detection and
// this loop draws what it is sent.  Easing here as well put two filters in
// series: measured on the wire with both, the drawn pupil trailed the aim it had
// been sent by 4.2 aim units (5.9 px) while the aim ramped at a head's speed;
// without this one the same test leaves 3.1 units (4.3 px) - one frame of
// travel.  A head-sized step costs 25 units of it either way (2.32 s vs 2.15 s
// to come within 1 unit), because the cap below, not the easing, governs that.
#define GAZE_SPEED 1.0f

// Cap on how far one frame may move.  This is not a second filter: a frame's
// cost is proportional to the distance it paints (12 px of iris travel changes
// ~3,100 pixels and cost ~340 ms on the bit-banged left panel, measured; 3 px
// costs ~46 ms), so a long move is spread over several cheap frames.  5.5 aim
// units is ~7.6 px, ~2,400 px - the most a frame can afford - so what the wearer
// sees behind the commanded aim is at most one frame of travel.
#define GAZE_STEP_MAX 5.5f

// Work done by the last repaint, reported with the cadence: the panels are
// throughput-bound, so "how much did it draw" is the number that explains the
// frame rate.
uint16_t repaintFills = 0;
uint32_t repaintPixels = 0;
#define TEST_MS 4000
float gx = 50.0, gy = 50.0;
int tx = 50, ty = 50;
bool targetSet = false;
float px[2] = {0, 0}, py[2] = {0, 0};  // last-drawn iris offset per eye

// ---- repaint timing report (every N moves, so tracking rate is measurable) ----
unsigned long repaintAccum = 0;
unsigned long lastRepaintTime = 0;
int repaintCount = 0;

// Types the iris repaint passes between its helpers.  Declared here because the
// Arduino prototype inserter writes its prototypes above the first function in
// the sketch, which is before these are defined.
struct DiscWalk;
struct IrisRow;
struct IrisRun;

// The two controllers in use, both GC9A01A: type 3 is the left panel on its
// bit-bang port, type 4 the same panel moved to the hardware SPI bus (MOSI=D11,
// SCLK=D13, shared with the right eye, its own CS/DC/RST).  Type 4 is the same
// library path the right eye runs, so it delivers the right eye's byte rate
// instead of the bit-bang's - the one change that makes a moving gaze smooth.
const char *typeName(int t) {
  if (t == 4) return "GC9A01A (hw SPI)";
  return "GC9A01A (bit-bang)";
}

// ---- init one eye with a given controller type ----
void initEye(int i, int type) {
  eyeType[i] = type;
  if (i == 0)
    eye[0] = (type == 4) ? (Panel *)&gfGcL : (Panel *)&fpL;
  else
    eye[1] = &gfGcR;
  if (i == 0 && type == 4)      gcL.begin();
  else if (i == 0)              fastL.begin();
  else                          gcR.begin();

  W[i] = eye[i]->W;
  H[i] = eye[i]->H;
  CX[i] = W[i] / 2;
  CY[i] = H[i] / 2;

  // Round-panel-aware geometry, sized off the panel's inscribed circle: on
  // 240x240 that is r=114, the *outer* edge of the goggle ring.  The ring and
  // the yellow eyelid are bands inside it, and the sclera is what is left, so
  // the iris sits in a big white eye with a coloured rim - the reference's
  // proportion - rather than filling the panel.
  int m = min(W[i], H[i]);
  EYE_R[i]   = m / 2 - 6;                   // 114 on the 240x240 round panel
  LID_R[i]   = EYE_R[i] - max(2, EYE_R[i] * 8 / 100);   // 105: inside the metal band
  SCLERA_R[i] = LID_R[i] - max(2, EYE_R[i] * 9 / 100);  //  95: the white the iris moves in
  IRIS_R[i] = SCLERA_R[i] * 34 / 100;       //  32
  PUPIL_R[i] = IRIS_R[i] * 50 / 100;        //  16
  HL_R[i]   = max(3, IRIS_R[i] / 5);        //   6
  IRIS_RR[i]  = IRIS_R[i]  * IRIS_R[i];
  PUPIL_RR[i] = PUPIL_R[i] * PUPIL_R[i];
  HL_RR[i]    = HL_R[i]    * HL_R[i];

  Serial.print(F("EYE"));
  Serial.print(i + 1);
  Serial.print(F(" init: "));
  Serial.print(typeName(type));
  Serial.print(F("  "));
  Serial.print(W[i]);
  Serial.print('x');
  Serial.print(H[i]);
  Serial.print(F(" r="));
  Serial.print(EYE_R[i]);
  Serial.print(F(" lid="));
  Serial.print(LID_R[i]);
  Serial.print(F(" sclera="));
  Serial.print(SCLERA_R[i]);
  Serial.print(F(" iris="));
  Serial.println(IRIS_R[i]);
}

// ---- color test: left RED, right GREEN; fill timings prove the fast path ----
void colorTest() {
  unsigned long t0 = millis();
  eye[0]->fillScreen(RED);
  unsigned long t1 = millis();
  eye[1]->fillScreen(GREEN);
  unsigned long t2 = millis();
  Serial.print(F("FILL L: "));
  Serial.print(t1 - t0);
  Serial.print(F("ms  R: "));
  Serial.print(t2 - t1);
  Serial.println(F("ms  (was ~10s with Adafruit soft-SPI)"));
  Serial.println(F("TEST: L=RED R=GREEN (4s)"));
  Serial.println(F("  BLUE for RED -> BGR panel: flip MADCTL in the init table"));
  Serial.println(F("  white/garbage -> wrong controller: TYPE 3 3"));
  Serial.println(F("  rerun: TEST"));
  unsigned long start = millis();
  while (millis() - start < TEST_MS) {
    handleSerial();  // keep the link responsive during the test
    delay(10);
  }
}

// ---- full scene: the goggle, the eyelid, the sclera and the iris ----
void drawScene() {
  for (int i = 0; i < 2; i++) drawEye(i, 0, 0);
  px[0] = py[0] = px[1] = py[1] = 0;
}

// ---- the goggle ----
// One lens per panel: the black strap knuckle at each side, the metal ring with
// its two bolts, the yellow eyelid band, then the sclera the iris moves in.
// Only the knuckles are outside the ring, so nothing here is hidden by the round
// panel's corners - on a 240x240 the inscribed circle is the whole drawing.
//
// This runs at boot and on TEST/TYPE, never per gaze step: the iris moves
// incrementally (see moveIris), so a richer scene costs nothing while tracking.
void drawStrap(int i) {
  int h = EYE_R[i] * 44 / 100;
  int y = CY[i] - h / 2;
  int inner = CX[i] - EYE_R[i] * 86 / 100;        // tucks under the ring
  if (inner > 0) eye[i]->fillRect(0, y, inner, h, STRAP);
  int right = CX[i] + EYE_R[i] * 86 / 100;
  if (right < W[i]) eye[i]->fillRect(right, y, W[i] - right, h, STRAP);
}

void drawEye(int i, float ox, float oy) {
  eye[i]->fillScreen(BACKG);
  drawStrap(i);
  eye[i]->fillCircle(CX[i], CY[i], EYE_R[i], METAL);          // goggle ring
  eye[i]->drawCircle(CX[i], CY[i], EYE_R[i], STEEL);          // its outer edge
  eye[i]->fillCircle(CX[i], CY[i], LID_R[i], LID);            // yellow eyelid
  eye[i]->fillCircle(CX[i], CY[i], SCLERA_R[i], WHITE);       // sclera
  drawIris(i, ox, oy);
}

void drawIris(int i, float ox, float oy) {
  int ix = CX[i] + (int)ox, iy = CY[i] + (int)oy;
  eye[i]->fillCircle(ix, iy, IRIS_R[i], IRIS);
  eye[i]->fillCircle(ix, iy, PUPIL_R[i], PUPIL);
  eye[i]->fillCircle(ix - HL_R[i], iy - HL_R[i], HL_R[i], WHITE);   // highlight
}

// ---- incremental iris motion ------------------------------------------------
// The iris is three concentric discs (iris, pupil, highlight) on white sclera,
// so a move changes only the pixels near the edges that moved.  Erasing and
// redrawing both discs costs ~12,900 pixels per eye per step, which the left
// eye's bit-bang SPI turns into 341 ms frames - measured, and visible as a
// gaze that steps instead of gliding.  Repainting just the difference costs a
// few hundred pixels, so the eyes can actually run at the speed the easing in
// loop() was written for.
#define IRIS_RUNS 10

// The half-width walk for one disc, carried down the rows of a repaint.  A
// sweep asks where the disc's edge is ~90 times per eye per frame, and a 16-bit
// multiply on an Uno costs ~80 cycles, so the walk keeps the half-width *and its
// square* and adjusts by additions: only the row's dy*dy is a multiply.  With
// the search form the row diff measured 52 ms against the 46 ms of panel writes
// it feeds, i.e. the geometry cost more than the drawing.
struct DiscWalk { int limit, hw, sq; };

// Point the walk at row dy of a disc of radius r (rr = r*r), -1 when it misses.
static inline void discRow(DiscWalk &w, int rr, int dy, int r) {
  if (dy > r || dy < -r) { w.limit = -1; w.hw = 0; w.sq = 0; return; }
  if (dy < 0) dy = -dy;
  w.limit = rr - dy * dy;
}

// Largest hw with hw*hw <= limit, reached from the row above's answer.
static inline int discHalf(DiscWalk &w, int r) {
  if (w.limit < 0) return -1;
  while (w.hw > 0 && w.sq > w.limit) { w.sq -= 2 * w.hw - 1; w.hw--; }
  while (w.hw < r && w.sq + 2 * w.hw + 1 <= w.limit) { w.sq += 2 * w.hw + 1; w.hw++; }
  return w.hw;
}

// The three discs of one eye at one row, as inclusive x spans.
struct IrisSpan { int lo, hi; bool has; };
struct IrisRow { IrisSpan iris, pupil, hl; };

// Same layering drawIris() paints: highlight over pupil over iris over sclera.
static inline __attribute__((always_inline)) uint16_t irisRowColour(const IrisRow &row, int x) {
  if (row.hl.has && x >= row.hl.lo && x <= row.hl.hi) return WHITE;
  if (row.pupil.has && x >= row.pupil.lo && x <= row.pupil.hi) return PUPIL;
  if (row.iris.has && x >= row.iris.lo && x <= row.iris.hi) return IRIS;
  return WHITE;                                            // sclera
}

// The x positions where this row's colours change, ascending.  The three discs
// are nested - the highlight sits inside the pupil, which sits inside the iris -
// so the left edges already ascend and the right edges already descend, and the
// list needs no sorting (sorting it per row cost as much as painting the frame).
static inline int irisRowEdges(const IrisRow &row, int xLo, int xHi, int *out) {
  const IrisSpan *lo[3] = {&row.iris, &row.pupil, &row.hl};
  const IrisSpan *hi[3] = {&row.hl, &row.pupil, &row.iris};
  int n = 0;
  for (int k = 0; k < 3; k++)
    if (lo[k]->has && lo[k]->lo >= xLo && lo[k]->lo <= xHi) out[n++] = lo[k]->lo;
  for (int k = 0; k < 3; k++)
    if (hi[k]->has && hi[k]->hi + 1 >= xLo && hi[k]->hi + 1 <= xHi + 1) out[n++] = hi[k]->hi + 1;
  return n;
}

static inline void irisRowAt(int i, int cx, int cy, int y,
                             DiscWalk &wi, DiscWalk &wp, DiscWalk &wh, IrisRow &row) {
  const int r = IRIS_R[i], pr = PUPIL_R[i], hr = HL_R[i];
  const int dy = y - cy;
  discRow(wi, IRIS_RR[i], dy, r);
  int hw = discHalf(wi, r);
  row.iris = {cx - hw, cx + hw, hw >= 0};
  discRow(wp, PUPIL_RR[i], dy, pr);
  hw = discHalf(wp, pr);
  row.pupil = {cx - hw, cx + hw, hw >= 0};
  discRow(wh, HL_RR[i], dy + hr, hr);                      // centred hr up and left
  hw = discHalf(wh, hr);
  row.hl = {cx - hr - hw, cx - hr + hw, hw >= 0};
}

// One changed span, held open while the rows below repeat it so a run of
// identical rows costs one address window instead of one per row.
struct IrisRun { int x, w, y0; uint16_t c; bool live; };

// Every changed span goes through here, which is also where the repaint's work is
// counted: the panels are throughput-bound, so the fills and pixels a frame drew
// are what explain its cadence (see the REPAINT report in loop()).  Counting is
// two increments - timing each fill with micros() measured ~11 ms a frame here,
// more than the whole right-eye paint.
static inline __attribute__((always_inline)) void moveIrisFill(int i, int x, int y, int w, int h, uint16_t c) {
  if (h <= 0) return;
  repaintFills++;
  repaintPixels += (uint32_t)w * (uint32_t)h;
  eye[i]->fillRect(x, y, w, h, c);
}

// Open a changed span for this row: continue the run from the row above when it
// has the same geometry and colour, otherwise start a new one (and write out
// whatever that row did not continue).
static void pushIrisRun(int i, IrisRun *open, int &nopen, int x, int y, int w, uint16_t c) {
  for (int k = 0; k < nopen; k++)
    if (!open[k].live && open[k].x == x && open[k].w == w && open[k].c == c) {
      open[k].live = true;
      return;
    }
  if (nopen < IRIS_RUNS) {
    open[nopen++] = {x, w, y, c, true};
    return;
  }
  moveIrisFill(i, x, y, w, 1, c);                          // table full: paint it now
}

void moveIris(int i, float fromX, float fromY, float toX, float toY) {
  const int r = IRIS_R[i];
  const int ax = CX[i] + (int)fromX, ay = CY[i] + (int)fromY;
  const int bx = CX[i] + (int)toX,   by = CY[i] + (int)toY;
  const int pad = r + 1;
  const int xLo = max(0, min(ax, bx) - pad);
  const int xHi = min(W[i] - 1, max(ax, bx) + pad);
  const int yTop = max(0, min(ay, by) - pad);
  const int yBot = min(H[i] - 1, max(ay, by) + pad);
  if (xLo > xHi || yTop > yBot) return;

  IrisRun open[IRIS_RUNS];
  int nopen = 0, lastY = yTop;
  DiscWalk wiA = {0, 0, 0}, wpA = {0, 0, 0}, whA = {0, 0, 0};
  DiscWalk wiB = {0, 0, 0}, wpB = {0, 0, 0}, whB = {0, 0, 0};
  for (int y = yTop; y <= yBot; y++) {
    lastY = y;
    IrisRow was, now;
    irisRowAt(i, ax, ay, y, wiA, wpA, whA, was);
    irisRowAt(i, bx, by, y, wiB, wpB, whB, now);

    // Every x where the colour could change, from either the old or the new
    // discs; the change is constant between consecutive boundaries, so the row
    // is diffed with a handful of comparisons instead of a test per pixel.
    int ea[6], eb[6];
    const int na = irisRowEdges(was, xLo, xHi, ea);
    const int nb2 = irisRowEdges(now, xLo, xHi, eb);
    int b[16];                                           // row ends + 6 edges each
    int nb = 0, ia = 0, ib = 0;
    b[nb++] = xLo;
    while (ia < na || ib < nb2) {                        // merge of two sorted lists
      if (ib >= nb2 || (ia < na && ea[ia] <= eb[ib])) b[nb++] = ea[ia++];
      else b[nb++] = eb[ib++];
    }
    b[nb++] = xHi + 1;

    for (int k = 0; k < nopen; k++) open[k].live = false;
    int runX = -1, runEnd = -1;
    uint16_t runC = 0;
    for (int k = 0; k + 1 < nb; k++) {
      const int x0 = b[k], x1 = b[k + 1];
      if (x1 <= x0) continue;
      const uint16_t wasC = irisRowColour(was, x0);
      const uint16_t nowC = irisRowColour(now, x0);
      if (wasC == nowC) {
        if (runX >= 0) pushIrisRun(i, open, nopen, runX, y, runEnd - runX, runC);
        runX = -1;
        continue;
      }
      if (runX >= 0 && runEnd == x0 && nowC == runC) { runEnd = x1; continue; }
      if (runX >= 0) pushIrisRun(i, open, nopen, runX, y, runEnd - runX, runC);
      runX = x0;
      runEnd = x1;
      runC = nowC;
    }
    if (runX >= 0) pushIrisRun(i, open, nopen, runX, y, runEnd - runX, runC);

    for (int k = 0; k < nopen; ) {
      if (open[k].live) { k++; continue; }
      moveIrisFill(i, open[k].x, open[k].y0, open[k].w, y - open[k].y0, open[k].c);
      open[k] = open[--nopen];
    }
  }
  for (int k = 0; k < nopen; k++)
    moveIrisFill(i, open[k].x, open[k].y0, open[k].w, lastY + 1 - open[k].y0, open[k].c);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("EYES FW v5.3 - Minion goggle (GC9A01A)"));
  Serial.println(F("PINS L: CS=10 DC=9 MOSI=3 SCK=4 RST=8 (bit-bang)"));
  Serial.println(F("PINS R: CS=7 DC=6 MOSI=11 SCK=13 RST=5 (hw SPI)"));

  SPI.begin();               // the Adafruit panels share this bus (D11/D13)
  initEye(0, eyeType[0]);
  initEye(1, eyeType[1]);

  colorTest();               // LEFT=RED RIGHT=GREEN (fill timings printed)
  drawScene();
  lastRepaintTime = millis();
  Serial.println(F("EYES READY - PING / T <px> <py> / GAZE?"));
}

void loop() {
  handleSerial();

  float aimX = targetSet ? (float)tx : 50.0;
  float aimY = targetSet ? (float)ty : 50.0;
  float ddX = aimX - gx, ddY = aimY - gy;
  float dd = sqrt(ddX * ddX + ddY * ddY);
  float k = GAZE_SPEED;
  if (dd * k > GAZE_STEP_MAX) k = GAZE_STEP_MAX / dd;
  gx += ddX * k;
  gy += ddY * k;

  // Map the smoothed aim (0..100 square) onto a UNIT vector, so the iris
  // center travels inside a circle of radius maxOff — a per-axis clamp alone
  // would push the iris outside the sclera at diagonal gazes.
  float vx = (gx - 50.0) / 50.0;
  float vy = (gy - 50.0) / 50.0;
  float mag = sqrt(vx * vx + vy * vy);
  if (mag > 1.0) { vx /= mag; vy /= mag; }

  float off[2] = {0, 0}, offY[2] = {0, 0};
  for (int i = 0; i < 2; i++) {
    float conv = (i == 1) ? 0.85f : 1.0f;        // right eye slightly more converged
    // The iris stays on the white: the sclera shrank by the goggle's two bands,
    // so this radius is the sclera's, not the panel's.
    float maxOff = SCLERA_R[i] - IRIS_R[i] - 2;
    off[i]  = vx * maxOff * conv;
    offY[i] = vy * maxOff * conv;
  }

  bool moved = false;
  for (int i = 0; i < 2; i++) {
    if (fabs(off[i] - px[i]) + fabs(offY[i] - py[i]) > 1.0) {
      eye[i]->beginBatch();
      moveIris(i, px[i], py[i], off[i], offY[i]);   // only the pixels that change
      eye[i]->endBatch();
      px[i] = off[i]; py[i] = offY[i];
      moved = true;
    }
  }

  if (moved) {
    unsigned long now = millis();
    unsigned long gap = now - lastRepaintTime;
    lastRepaintTime = now;
    if (gap < 1000) {                // ignore idle gaps: report the active repaint cost
      repaintAccum += gap;
      repaintCount++;
      if (repaintCount >= 25) {      // report the measured repaint cost
        Serial.print(F("REPAINT n=25 avg="));
        Serial.print(repaintAccum / 25);
        Serial.print(F("ms fills="));
        Serial.print(repaintFills / 25);
        Serial.print(F(" px="));
        Serial.print(repaintPixels / 25);
        Serial.println();
        repaintAccum = 0;
        repaintCount = 0;
        repaintFills = 0;
        repaintPixels = 0;
      }
    }
  }

  // The repaint itself paces the loop now that it only touches changed pixels;
  // a long idle pause here would cap the gaze at 50 Hz and add lag for nothing.
  delay(5);
}

void handleSerial() {
  static String line = "";
  static unsigned long lastChar = 0;
  while (Serial.available()) {
    char c = Serial.read();
    lastChar = millis();
    if (c == '\n') {
      line.trim();
      if (line.length() > 0) {
        if (line.startsWith("PING")) {
          Serial.println("PONG");
        } else if (line.startsWith("TEST")) {
          colorTest();
          drawScene();
          Serial.println(F("TEST DONE"));
        } else if (line.startsWith("TYPE ")) {
          int sp = line.indexOf(' ', 5);
          if (sp > 0) {
            int l = line.substring(5, sp).toInt();
            int r = 3;                             // the right eye is always on the bus
            if (l == 3 || l == 4) {
              Serial.print(F("TYPE OK: LEFT="));
              Serial.print(typeName(l));
              Serial.print(F(" RIGHT="));
              Serial.println(typeName(r));
              initEye(0, l);
              initEye(1, r);
              colorTest();
              drawScene();
              Serial.println(F("REINIT DONE"));
            } else {
              Serial.println(F("TYPE: 3=GC9A01A bit-bang 4=GC9A01A hwSPI"));
            }
          }
        } else if (line.startsWith("GAZE?")) {
          // Where the gaze is and what has been drawn, so a move can be timed
          // from the Pi instead of guessed at from the panels.
          Serial.print(F("GAZE aim "));
          Serial.print(gx, 1);
          Serial.print(' ');
          Serial.print(gy, 1);
          Serial.print(F("  drawn "));
          Serial.print(px[0], 1);
          Serial.print(' ');
          Serial.println(py[0], 1);
        } else if (line.startsWith("T ")) {
          int sp = line.indexOf(' ', 2);
          if (sp > 0) {
            int a = line.substring(2, sp).toInt();
            int b = line.substring(sp + 1).toInt();
            if (a >= 0 && b >= 0) {
              tx = constrain(a, 0, 100);
              ty = constrain(b, 0, 100);
              targetSet = true;
            } else {
              targetSet = false;  // T -1 -1
            }
          }
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
  // defensive: a partial line that never got its \n (e.g. bytes lost during a
  // long fill) is discarded after 200 ms so the parser can't wedge
  if (line.length() > 0 && millis() - lastChar > 200) line = "";
}
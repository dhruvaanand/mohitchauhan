# ESP32 Dynamic Light Show

A digital art installation that displays an animated light show using an ESP32 microcontroller and the FastLED library. The project brings together creative animation techniques to produce an engaging, music-inspired visual experience.

---

## Hardware Requirements

- ESP32 microcontroller board (Doit DevKit V1 recommended)
- LED strip compatible with FastLED
- USB cable for flashing

## Software Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) or PlatformIO
- [FastLED library](https://fastled.io/)
- ESP32 board support package

---

## Installation

1. Clone this repository:
   ```bash
   git clone git@github.com:dhruvaanand/mohitchauhan.git
   cd mohitchauhan
   ```

2. Open the project in Arduino IDE or PlatformIO.

3. Install the FastLED library via the Arduino Library Manager.

4. Select your board: **ESP32 Doit DevKit V1**.

5. Upload the code to your board.

---

## Scenes

The light show runs for approximately 20 seconds and is divided into three scenes:

### Scene 1 — Buildup (0–7s)
A slow-motion opening sequence with gradual color and pattern transitions, building anticipation for the scenes ahead.

### Scene 2 — Tunnel (7–13.4s)
A fast-paced sequence featuring a tunnel effect with moving lights and rapidly shifting colors.

### Scene 3 — Chorus & Breakdancer (13.4–20s)
A dance-inspired finale featuring an animated breakdancer figure that moves and changes pose, accompanied by flashing lights and vibrant color effects.

---

## Visual Techniques

- **Sinusoidal movement** — smooth, wave-based animation for the dancer's arms and legs
- **HSV color shifting** — dynamic hue, saturation, and value transitions for rich color effects
- **Gradient effects** — seamless color blending across the LED strip
- **Pose animation** — frame-by-frame movement of the breakdancer figure

---

## Project Structure

```
mohitchauhan/
├── src/
│   └── main.cpp       # Main animation logic
├── README.md
└── platformio.ini     # Board and library config (if using PlatformIO)
```

---

## License

This project is open source. Feel free to remix and build on it for your own light art installations.

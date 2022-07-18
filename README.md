# ESP32-Stereo-Audio-Visualizer-LED-Matrix-with-FreeRTOS
This project is an upgrade from a [previous project](https://www.hackster.io/chef_jeff69/esp32-stereo-audio-spectrum-visualizer-with-amplifier-47bbff) that works on the same Fast Fourier Transform Algorithim. 

[Link to Video](https://youtu.be/zim4bbnrGHo)

[Link to CAD files](https://grabcad.com/library/esp32-stereo-audio-visualizer-led-matrix-1)

<img src="https://user-images.githubusercontent.com/83417790/179507445-08b6988e-9674-46d6-80ce-213af23f7e45.png" width="650px" height="480px">

The upgrades in this project compared to the previous one:
- 2D LED Matrix
- Bluetooth support
- High accuracy external ADC (MCP 3002)
- Built in 20W stereo speakers 
- High clarity audio driven by class AB amplifier with ground loop isolator
- Digital touch controls
- Runs tasks using both cores of the ESP32 for higher frame rates
- Wide range of voltage input (9V - 40V) and USB type-C PD ** works with power banks **

Many Thanks to [Michiel Steltman](https://github.com/MichielfromNL) for providing the [ESPfft library](https://github.com/yash-sanghvi/ESP32/issues/1) used in this project

To access the brightness, mode and palette controls,

There are 3 screws on the speaker linked to the touch enabled GPIOs of the ESP32.

####Brightness controls
Touch left button first then brightness controls will come up.
<img src="https://user-images.githubusercontent.com/83417790/179508867-1a27617a-086e-490c-9e15-cf0a1236bef1.jpg" width="650px" height="480px">

Touch the middle button while touching the left button to decrease brightness, touch and hold to further decrease brightness

<img src="https://user-images.githubusercontent.com/83417790/179509072-212a2ebf-5d6e-4597-bdc9-2de61df2862f.jpg" width="650px" height="480px">

Touch the right button while touching the left button to increase brightness, touch and hold to further increase brightness
<img src="https://user-images.githubusercontent.com/83417790/179509220-4771b2dc-2343-4439-953d-3a26d2abbc54.jpg" width="650px" height="480px">

#### Mode and Palette controls
Touch middle button to bring up mode and palette controls menu.
<img src="https://user-images.githubusercontent.com/83417790/179509369-017431fb-279d-4ca7-b493-1b13bec2b69e.jpg" width="650px" height="480px">

Touch left button while touching middle button to change modes, there will be an animation to indicate the changing of modes
<img src="https://user-images.githubusercontent.com/83417790/179509494-61eb689b-3bbe-49ca-bd22-f50bc54e95d6.jpg" width="650px" height="480px">

Touch right button while touching middle button to change color palettes.

#### White lights mode

Touch right button first then the other two buttons and white lights mode will be triggered. 

** mode and palette controls are disabled when white lights are on **
<img src="https://user-images.githubusercontent.com/83417790/179511024-93f701b1-e6c4-4c1e-90bf-eeadef21dc95.jpg" width="650px" height="480px">

Do the same to turn off white lights mode.

#### Turn off/on speakers

Touch right button then the other two buttons and hold all buttons for 3 seconds to toggle off/on for the speakers.

This mode is added in so the speakers can be solely as light source while conserving power.


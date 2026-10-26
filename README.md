# Aircraft Landing Gear PID Demo

This is my Control Systems project. It uses an Arduino Uno, an SG90 servo, a potentiometer and a webpage to compare P, PI, PD and PID control.

The Arduino runs a simple second-order model. The SG90 moves to show the model angle, and the webpage draws the response graph. The servo's actual shaft angle is not measured by this setup, so the graphs are model results rather than real aircraft or servo test data.

## Wiring

Turn off/unplug the circuit while changing wires.

**SG90 servo**

| Servo wire | Connect it to |
|---|---|
| Orange/yellow (signal) | Arduino D9 |
| Red (power) | Arduino 5V for a light, unloaded demo |
| Brown/black (ground) | Arduino GND |

**Potentiometer (three pins)**

| Potentiometer pin | Connect it to |
|---|---|
| One outside pin | Arduino 5V |
| Middle pin | Arduino A0 |
| Other outside pin | Arduino GND |

The two outside potentiometer pins can be swapped. That only changes which way you turn the knob to increase the angle. The breadboard can be used to share the 5V and GND connections.

If the Arduino resets or the servo jitters, use a regulated 5V supply for the servo. Connect that supply's GND to Arduino GND. Do **not** connect the external supply's +5V to the Arduino 5V pin while USB power is connected. Do not connect a 9V battery directly to the SG90.

## Upload the Arduino code

1. Open `firmware/LandingGearPID/LandingGearPID.ino` in Arduino IDE.
2. In **Tools**, choose **Arduino Uno** and the COM port for your board.
3. Click **Upload**.
4. Close Serial Monitor and Serial Plotter. The webpage needs to use the COM port itself.

## Open the webpage

1. Open the project folder in File Explorer — the folder that contains `index.html`.
2. Click the address bar, type `cmd`, and press Enter.
3. In the black terminal window, run:

   ```cmd
   python -m http.server 8000
   ```

   If Windows says Python was not found, try:

   ```cmd
   py -m http.server 8000
   ```

4. Leave that window open.
5. Open Chrome or Edge and go to **http://localhost:8000/**.
6. Click **Connect Arduino**, select the Uno's COM port, and approve the connection.
7. While idle, turn the knob to move the servo indicator by hand. Set the actual target angle in the webpage, then choose P, PI, PD or PID. **Compare** runs all four in order.

To upload new code later, first disconnect the Arduino from the webpage and close any serial windows, then upload from Arduino IDE again.

## If something doesn't work

- **COM port access denied:** disconnect the webpage and close Serial Monitor/Plotter, then try again.
- **Page won't connect:** use desktop Chrome or Edge, make sure the server window is still open, and check that the board is connected by USB.
- **The server can't find the page:** run the command from the folder where `index.html` is saved.

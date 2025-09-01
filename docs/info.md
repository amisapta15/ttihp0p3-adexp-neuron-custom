## 🧠 AdEx Spiking Neuron Core

This project is a digital hardware implementation of the Adaptive Exponential (AdEx) Integrate-and-Fire neuron model. It's designed to run on an ASIC, simulating the behavior of a biological neuron, including its membrane potential and adaptation mechanisms. The core is highly configurable, allowing it to model various neural firing patterns like regular spiking, bursting, and fast spiking.

## How it works ⚙️

The system operates based on two primary components: the **Neuron Core** and the **Parameter Loader**.

### 1. The Neuron Core

The core solves two coupled differential equations in real-time using Q4.12 fixed-point arithmetic. These equations govern the neuron's two main state variables:

*   **V:** The membrane potential, which simulates the voltage across the neuron's cell membrane.
*   **w:** The adaptation current, which models cellular fatigue and is responsible for spike-frequency adaptation.

The behavior is defined by the following equations, which are a direct representation of the hardware's operation:

```math
\frac{dV}{dt} = \frac{-g_L(V - E_L) + g_L\Delta_T\exp\left(\frac{V - V_T}{\Delta_T}\right) - w + I}{C} \\
\frac{dw}{dt} = \frac{a(V - E_L) - w}{\tau_w}
```
## How it works ⚙️

The system operates based on two primary components: the **Neuron Core** and the **Parameter Loader**.

### 1. The Neuron Core

The core solves two coupled differential equations in real-time using Q4.12 fixed-point arithmetic. These equations govern the neuron's two main state variables:

*   **V:** The membrane potential, which simulates the voltage across the neuron's cell membrane.
*   **w:** The adaptation current, which models cellular fatigue and is responsible for spike-frequency adaptation.

The behavior is defined by the following equations:

```math
\tau \frac{dV}{dt} = -g_L(V - E_L) + g_L\Delta_T\exp\left(\frac{V - V_T}{\Delta_T}\right) - w + I
```
```math
\tau_w\frac{dw}{dt} = a(V - E_L) - w
```

When the membrane potential `V` crosses a threshold `VT`, the core outputs a digital **spike** ⚡. After a spike, `V` is reset to `Vreset` and the adaptation current `w` is increased by a value `b`.

### 2. The Parameter Loader 📥

The neuron's behavior is defined by 7 distinct parameters. To configure the core, these parameters must be loaded serially via a simple 4-bit interface.

The parameters are loaded in the following order:

| Index | Parameter | Description                               |
| :---: | :-------: | :---------------------------------------- |
|   0   | `DeltaT`  | Sharpness of the spike initiation         |
|   1   |  `TauW`   | Adaptation time constant                  |
|   2   |    `a`    | Subthreshold adaptation level             |
|   3   |    `b`    | Spike-triggered adaptation increment      |
|   4   | `Vreset`  | Voltage to reset to after a spike         |
|   5   |   `VT`    | Firing threshold voltage                  |
|   6   |  `Ibias`  | Constant input current                    |

The loading process is controlled by the `ui_in` pins:
*   `ui_in[4]` (`load_mode`): Must be high to enable loading.
*   `ui_in[3]` (`load_enable`): A rising edge on this pin latches the 4-bit value present on `uio_in[3:0]`.

Each 8-bit parameter is sent as two 4-bit nibbles (high nibble first). After all 14 nibbles have been sent, a special **footer nibble (`0xF`)** must be sent to commit the new parameters to the core.

### Inputs and Outputs

*   **Inputs**:
    *   `ui_in[6]` (`clk`): Main clock signal.
    *   `ui_in[5]` (`reset`): Active-high reset.
    *   `ui_in[4]` (`load_mode`): Set to `1` to enable the parameter loader.
    *   `ui_in[3]` (`load_enable`): Pulse high to load a 4-bit nibble from `uio_in`.
    *   `ui_in[2]` (`enable_core`): Set to `1` to run the neuron simulation.
    *   `ui_in[1]` (`debug_mode`): Selects the debug output on `uo_out[6:1]`.
    *   `uio_in[3:0]`: 4-bit data bus for loading parameter nibbles.
*   **Outputs**:
    *   `uo_out[0]` (**`spike`**): The primary output. Pulses high for one clock cycle when the neuron fires.
    *   `uo_out[6:1]`: A 6-bit debug bus showing the most significant bits of either `V` (if `debug_mode=0`) or `w` (if `debug_mode=1`).

## Firing Modes and How to Trigger Them 🧠⚡️

The AdEx model's strength is its ability to reproduce different neural behaviors. The firing pattern is primarily determined by the interplay between the adaptation parameters (`a`, `b`, `TauW`) and the input current (`Ibias`). By loading different parameter sets, you can make the neuron behave in specific ways.

*Note: The 8-bit encoded value is what you need to send to the hardware. For signed values (`Vreset`, `VT`, `Ibias`), the encoding is `Real Value + 128`. For unsigned values, the encoding is just the `Real Value`.*

---
### 📈 Regular Spiking (Adapting)
This is the "default" behavior for many excitatory neurons. The firing rate is initially high and then slows down as the adaptation current `w` builds up.

*   **Mechanism**: A non-zero spike-triggered adaptation (`b`) increases `w` with every spike, making it harder for the neuron to reach its firing threshold again.

*   **Parameter Values**:
| Parameter | Real-World Value | 8-bit Encoded Value | Hex Value |
| :---      | :---             | :---                | :---      |
| `a`       | 2 nS             | `2`                 | `0x02`    |
| `b`       | 40 pA            | `40`                | `0x28`    |
| `Vreset`  | -65 mV           | `63`                | `0x3F`    |
| `Ibias`   | 50 pA            | `178`               | `0xB2`    |
*(Other parameters can be left at their default values)*

---
### 💥 Bursting
This behavior is characterized by clusters of high-frequency spikes separated by periods of silence (hyperpolarization).

*   **Mechanism**: Strong subthreshold adaptation (`a`) and a less-negative reset potential (`Vreset`) are key. The adaptation current `w` builds up slowly, eventually stopping the burst. As `w` decays, the membrane potential depolarizes again, initiating the next burst.

*   **Parameter Values**:
| Parameter | Real-World Value | 8-bit Encoded Value | Hex Value |
| :---      | :---             | :---                | :---      |
| `a`       | 4 nS             | `4`                 | `0x04`    |
| `b`       | 0 pA             | `0`                 | `0x00`    |
| `Vreset`  | -50 mV           | `78`                | `0x4E`    |
| `Ibias`   | 25 pA            | `153`               | `0x99`    |
*(Other parameters can be left at their default values)*

---
### 💨 Fast Spiking
Typical of inhibitory interneurons, this mode involves sustained high-frequency firing with little to no adaptation or slowdown.

*   **Mechanism**: This is achieved by simply turning off all adaptation mechanisms (`a` and `b` are zero). The neuron behaves like a simple leaky integrate-and-fire model, with its firing rate determined solely by the input current.

*   **Parameter Values**:
| Parameter | Real-World Value | 8-bit Encoded Value | Hex Value |
| :---      | :---             | :---                | :---      |
| `a`       | 0 nS             | `0`                 | `0x00`    |
| `b`       | 0 pA             | `0`                 | `0x00`    |
| `Vreset`  | -65 mV           | `63`                | `0x3F`    |
| `Ibias`   | 80 pA            | `208`               | `0xD0`    |
*(Other parameters can be left at their default values)*

---

## How to test 🧪

The provided `cocotb` test (`test_neuron_spike`) verifies the core's functionality by performing a complete load-and-run sequence.

The test procedure is as follows:

1.  **Reset**: The chip is held in reset for 10 clock cycles to initialize all internal states.

2.  **Load Parameters**: To provoke a spike, the test injects a constant positive input current. This is done by loading a new value for the `Ibias` parameter.
    *   The test enters `load_mode`.
    *   It sends 12 dummy nibbles (for the first 6 parameters).
    *   It sends the two nibbles for `Ibias` (a value of `150`, which is a supra-threshold current).
    *   It sends the `0xF` footer nibble to commit the new `Ibias` value.
    *   The test exits `load_mode`.

3.  **Run and Verify**:
    *   The test asserts `enable_core` to start the neuron simulation.
    *   It then monitors the `uo_out[0]` (spike) pin on every clock cycle.
    *   The test **passes** ✅ if a spike is detected within 1000 clock cycles.
    *   The test **fails** ❌ if no spike occurs in that time, indicating a problem with the simulation logic or the loading process.

## External hardware

N/A. This project is a self-contained digital core and requires no external components.
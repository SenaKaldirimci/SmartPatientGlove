# SMART PATIENT MONITORING GLOVE 

**Team Members:**
* **Fatma Sena KALDIRIMCI** (Department of Electrical & Electronics Engineering)
* **Zeynep Sude ŞAHİN** (Department of Electrical & Electronics Engineering)
* **Hakan SOLAK** (Department of Electrical & Electronics Engineering)
  
The pulse sensor is placed in the glove so that it matches the patient's pulse. The patient's pulse information is measured at regular intervals. The measured pulse value is continuously displayed on the glove's OLED screen as "Pulse: xxx bpm." The glove's primary purpose is to communicate the patient's requests to the caregiver. Each finger has a button for the patient's needs.

- **Index finger:** Press and release -> Need water; press and hold  ->  Need food  

- **Middle finger:** Press and release  ->  Need toilet; press and hold  ->  Cancel request  

- **Ring finger:** Press and release  -->  Feeling of pain; press and hold  -> Emergency  

- **Little finger:** Press and release -> Request to change bed position  

**Sample Scenario:**  
The patient felt pain and pressed and released the button on their ring finger. The MCU determined whether the button was pressed and released or held down. Based on this, it sent the appropriate messages to the patient's glove screen and the transmitter LoRa. The transmitter LoRa sent the message to the receiving LoRa. The receiving lora then transmits the message to the MCU inside the caregiver's box. The MCU processes the data and, based on the patient's request, produces the appropriate tone through the speaker and displays the request on the screen.

# Hardware Architecture

### Central Processing Unit (MCU): STM32F407

In this project, the STM32F407 microcontroller is used as the central processing unit in both the smart glove and the caregiver's box.

The MCU, acting as the system's control mechanism, executes all logical operations:
- It reads analog data from the pulse sensor and converts it into a digital pulse value.
- It processes interrupt-based digital inputs from the buttons on the fingers.
- It controls output units such as the OLED display and speaker.
- It manages wireless communication with transmitter and receiver LoRa modules.

## Input Units

### Buttons (4)
Four buttons, positioned for each finger, enable the patient to communicate their basic needs. The buttons are connected to the STM32's GPIO pins, and thanks to interrupt-based monitoring, it can distinguish between short presses (press and release) and long presses (hold down the button), ensuring instantaneous response time.

### Pulse Sensor
The analog pulse sensor, which measures the patient's heart rate, is connected to the ADC input of the STM32, and the raw analog signal is converted to BPM (beats per minute) by the MCU.

## Output Devices
### OLED Display
The displays are used to display pulse information and patient requests. The displays communicate with the STM32F407 via I²C protocol. 

### Speaker (Caregiver Box)
The speaker generates an audible alert to attract the nurse's attention. The speaker is driven by amplifying the signal from the STM32's PWM output using the PAM8403 mini amplifier. By varying the frequency and duty cycle of the PWM signal, different audio tones and alert patterns are created for each button type. For example: short and medium-frequency beeps for normal requests; a higher-frequency, frequent alarm tone for an emergency call. The speaker automatically plays the appropriate tone pattern based on the packets received over LoRa.

## Communication and Network Unit
### LoRa Modules (Transmitter and Receiver)
The LoRa transmitter on the glove receives small data packets containing the measured pulse and button events via UART/SPI from the STM32 and transmits them wirelessly to the caregiver box. The LoRa receiver on the box transmits the received data to the MCU inside the caregiver's box. The MCU processes the data and, based on the patient's request, produces the appropriate tone through the speaker and displays the request on the screen. This structure ensures reliable communication with low power consumption without the need for an external internet connection.

## Power Unit
The system is designed to work both while plugged into a wall adapter and when powered by a power bank. In normal use, the glove and the caregiver unit receive 5 V from a USB power adapter. When needed, the same USB cable can be connected to a power bank, and the system continues working without any changes. The STM32F407 operates in low-power modes to reduce energy consumption, especially during portable use. It wakes up only for button interrupts or periodic timers to perform measurements and LoRa communication.


# System Line Diagram
<img width="836" height="839" alt="image" src="https://github.com/user-attachments/assets/c5aea69e-b75d-42bf-a5b4-f582cb4bb1dd" />

# itclab-05 - iTCLab Kit Reading Using the Arduino+Python Programming Language
About Advanced iTCLab Research

# iTCLab Kit Reading Using the Arduino+Python Programming Language

<p align="center">
  <img src="https://github.com/bsrahmat/itclab-05/blob/main/jupyter0.jpg" alt="" class="img-responsive" width="700">
</p>

PWM stands for Pulse Width Modulation. So, PWM is a modulation technique that changes the pulse width with a fixed frequency and amplitude value. PWM can be considered the opposite of ADC (Analog to Digital Converter), which converts Analog signals to Digital. PWM or Pulse Width Modulation generates analog signals from Digital devices (iTCLab Kits).

The PWM signal will remain ON for a specific time and then stalled or OFF for the rest of the period. What makes this PWM unique and more valuable is that we can specify how long the ON state should last by controlling the PWM duty cycle.

The percentage of time the PWM signal remains ON time is known as the "duty cycle." The condition where the signal is always ON is called a 100% Duty Cycle, while the state where the movement is always OFF is called a 0% Duty Cycle.

The formula for calculating the work cycle or duty cycle can be shown as the equation below.

Duty Cycle = tON / (tON + tOFF)

Or

Duty Cycle = ton / ttotal

Where :

tON = ON time or time when the output voltage is high (high or 1)

tOFF = OFF time or time when the output voltage is low (low or 0)

ttotal = time of one cycle or the sum of tON and tOFF or also called the "one wave period."

Duty Cycle = ON Time / (ON Time + OFF Time)

The following image represents a PWM signal with a 60% duty cycle. As we can see, considering the entire time period (ON time + OFF time), the PWM signal is only ON for 60% of the time period.

<p align="center">
  <img src="https://github.com/bsrahmat/itclab-05/blob/main/jupyter01.jpg" alt="" class="img-responsive" width="700">
</p>

<p align="center">
  <img src="https://github.com/bsrahmat/itclab-05/blob/main/jupyter02.jpg" alt="" class="img-responsive" width="700">
</p>


To easily understand how the Pulse Width Modulation (PWM) works, you can read the following tutorial (Indonesian version): https://www.academia.edu/101739949.

In a control system using a PID controller, PWM is the output of the PID controller, where this output is regulated to control the plant. So that the plant can produce results as expected. In practice, to know how the PID controller works with PWM settings, it can be implemented in the Internet-Based Temperature Control Lab (iTCLab) Kit.

iTCLab - Internet-Based Temperature Control Lab. Temperature control kit for feedback control applications with an ESP32 Microcontroller, LED, two heaters, and two temperature sensors. The heating power output is adjusted to maintain the desired temperature setpoint. Heat energy from the heater is transferred by conduction, convection, and radiation to the temperature sensor. Heat is also transferred from the device to the environment.

About these iTCLab kits :
- It is inspired by <a href="https://apmonitor.com/pdc/index.php/Main/ArduinoTemperatureControl" target="_blank">The TCLab Product of Brigham Young University (BYU), one of the private campuses in Provo, Utah, United States of America.</a>
- Miniature Control System in a Pocket.
- Practical IoT Learning Package Tools.
- IoT programming Kits.
- IoT-Based Control System Practice Devices.
- It can be used to learn System Dynamics and Control Systems.
- It can be used to learn Arduino and Python programming.
- It can be used to learn AI and Machine Learning Programming.
- And others.

The underlying difference between TCLab and BYU's TCLab product is replacing the Arduino Uno microcontroller with ESP32. Using this ESP32, iTCLab can connect to the Internet of Things (IoT).

Tutorial for <b>iTCLab-03 PID Control System Introduction</b>, can be accessed at the address: https://www.academia.edu/101176526.


<p align="center">
  <img src="https://github.com/bsrahmat/itclab-01/blob/main/itclab01a.jpg" alt="" class="img-responsive" width="700">
</p>

<br>
</br>

## An example of a publication related to this Kits:

<a href="https://www.igi-global.com/pdf.aspx?tid=319461&ptid=296384&ctid=4&oa=true&isxn=9781668456293" target="_blank">iTCLab Temperature Monitoring and Control System Based on PID and Internet of Things (IoT)</a>

<br>
</br>

## Other publications by the researcher:

https://fitri.academia.edu/BasukiRahmat

https://www.researchgate.net/profile/Basuki-Rahmat-2

https://scholar.google.com/citations?user=BjCi4AgAAAAJ&hl=en

<br>
</br>

## Buy the iTCLab Kits at Shopee:

<p align="center">
<a href="https://shopee.co.id/product/78709625/11589970517" target="_blank"><img src="https://github.com/bsrahmat/itclab-01/blob/main/shopee_kit1.jpg" alt="" class="img-responsive" width="700">
</a>
</p>

<br>
</br>

## Other research by the IO-T.Net research team can be found at:

https://www.io-t.net/

<p align="center">
<a href="https://www.io-t.net/" target="_blank"><img src="https://github.com/bsrahmat/robot-bnu/blob/main/iot.png" alt="" class="img-responsive" width="500">
</a>
</p>

<br>
</br>

## Other research by the I-OT.Net research team can be found also at:

http://www.i-ot.net/

<p align="center">
<a href="http://www.i-ot.net/" target="_blank"><img src="https://github.com/bsrahmat/fuzzy-neural/blob/main/iot_logo.png" alt="" class="img-responsive" width="500">
</a>
</p>



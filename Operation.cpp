import time
from machine import Pin, PWM
from hx711 import HX711


# Vars
#Dev mode test
timeToExtend = 25  # in s
maxPlantarAngle = -20
pylonHeight = 0.3  # in meters
angle_tolerance = 0.5
force_tolerance = 0.5
ready = False
nextAngleReached = False
end = False
forceReached = False


# Body weight in Kilograms (replace with the actual value or allow input)
body_weight = 75  # Example: 100 kg


# CSV File Name
csv_filename = "angleVScompression.csv"




# pins


# LOADCELL BENDING
LCbend_OUT = Pin(5, Pin.IN, Pin.PULL_DOWN)
LCbend_SCK = Pin(4, Pin.OUT)


LCbend = HX711(LCbend_SCK, LCbend_OUT)
LCbend.set_scale(200)


# LOADCELL COMPRESSIVE
LCcomp_OUT = Pin(6, Pin.IN, Pin.PULL_DOWN)
LCcomp_SCK = Pin(7, Pin.OUT)


LCcomp = HX711(LCcomp_SCK, LCcomp_OUT)
LCcomp.set_scale(200)


time.sleep(1)  # time to initialize


# LOAD CELL CALIBRATION VARS
b_bend = 2.2517e+04
m_bend = -42.4024
b_comp = 2.6843e+04
m_comp = 3.4283e+03


# Define motor control pins


# LINEAR ACTUATOR BENDING PINS
in3 = Pin(17, Pin.OUT)  # H-Bridge Input 3
in4 = Pin(18, Pin.OUT)  # H-Bridge Input 4


# LINEAR ACTUATOR COMPRESSIVE PINS
in5 = Pin(11, Pin.OUT)  # H-Bridge Input 3
in6 = Pin(12, Pin.OUT)  # H-Bridge Input 4


# Set motor pwm
pwm3 = PWM(in3, freq=1000)  # PWM frequency 1kHz on IN1
pwm4 = PWM(in4, freq=1000)  # PWM frequency 1kHz on IN3


pwm5 = PWM(in5, freq=1000)  # PWM frequency 1kHz on IN1
pwm6 = PWM(in6, freq=1000)  # PWM frequency 1kHz on IN3


button_start = Pin(38, Pin.IN, Pin.PULL_UP)
button_lower = Pin(40, Pin.IN, Pin.PULL_UP)
button_ready = Pin(39, Pin.IN, Pin.PULL_UP)




# PID Controller
class PID:
   def __init__(self):
       self.kp = 300.0
       self.ki = 0.01
       self.kd = 0.01
       self.prev_error = 0
       self.integral = 0




   def compute(self, setpoint, measured_value):
       error = setpoint - measured_value
       self.integral += error
       derivative = error - self.prev_error
       self.prev_error = error
       return self.kp * error + self.ki * self.integral + self.kd * derivative




# Encoder class
class Encoder:
   def __init__(self, pin_a, pin_b, scale=1, counts_per_rev=600):
       self.scale = scale
       self.position = 0
       self.minDeg = 10
       self.last_a = 0
       self.last_b = 0
       self.counts_per_rev = counts_per_rev
       self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
       self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
       # Attach interrupts
       self.pin_a.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self._handle_interrupt)
       self.pin_b.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self._handle_interrupt)




   def _handle_interrupt(self, pin):
       # Read the state of the encoder pins
       a = self.pin_a.value()
       b = self.pin_b.value()
       if a != self.last_a:
           # Encoder moved
           if b != a:
               self.position += 1
           else:
               self.position -= 1
       self.last_a = a
       self.last_b = b




   def value(self):
       return self.position * self.scale




   def angle(self):
       # Calculate the angle in degrees
       return (-self.position / self.counts_per_rev) * 360
  
   def offsetInit(self):
     # set to known angle
     self.position = (self.minDeg / 360) * self.counts_per_rev


# Helper functions


# Function to read the CSV file and load the curve
def load_comp_data_from_csv(filename, body_weight):
   global percent  # Indicate that we are modifying the global variable
   data = []
   try:
       with open(filename, 'r') as file:
           for line in file:
               if line.strip():  # Ignore empty lines
                   angle, percent = map(float, line.strip().split(','))
                   desired_force = percent * body_weight  # Convert to force
                   data.append((angle, desired_force))
                   #global percent
   except Exception as e:
       print(f"Error reading CSV file: {e}")
       return []
   return data


# Function to actuate bending linear actuator
def driveLAbend(direction, duty_cycle):
   if direction == 1:
       pwm3.duty(duty_cycle)  # Set PWM duty cycle for IN1
       pwm4.duty(0)
   elif direction == 0:
       pwm4.duty(duty_cycle)  # Set PWM duty cycle for IN1
       pwm3.duty(0)


# Function to actuate compressive linear actuator
def driveLAcomp(direction, duty_cycle):
   if direction == 0:
       pwm5.duty(duty_cycle)
       pwm6.duty(0)
   elif direction == 1:
       pwm6.duty(duty_cycle)
       pwm5.duty(0)


# Function to actuate compressive linear actuator
def stopLAbend():
   pwm5.duty(0)
   pwm6.duty(0)


# Function to actuate compressive linear actuator
def stopLAcomp():
   pwm5.duty(0)
   pwm6.duty(0)


# Returns current angle
def get_angle():
   angle = encoder.angle()
   return angle  # current angle


LCcomp.tare(times=15)
def get_compressive_force():
   LCcomp_val = LCcomp.get_value()
   forceCOMP = (LCcomp_val - b_comp) / m_comp / 9.8
   return forceCOMP  # current compressive force


LCbend.tare(times=15)
def get_bending_force():
   LCbend_val = -LCbend.get_value()
   forceBEND = (LCbend_val - b_bend) / m_bend / 9.8
   return forceBEND  # current bending force


# Main function
def main():
   global encoder


   try:
       # Load curve from CSV
       compression_data = load_comp_data_from_csv(csv_filename, body_weight)
       if not compression_data:
           print("Error: No data loaded from CSV.")
           return


       # Print loaded curve for debugging
       print("Loaded Compression Curve:")
       for angle, force in compression_data:
           print(f"Angle: {angle}, Desired Force: {force} N")


       data_array = []  # To store [angle, bending force, compressive force]


       encoder = Encoder(10, 9)  # Initialize encoder
       pid = PID()  # Adjust PID constants as needed


       # Step 1: Initialize and calibrate
       print("Press button_start to start setup.")


       # Waits until button is pressed to continue
       while button_start.value() == 1:
           time.sleep(0.1)


       print("Extending compressive actuator...")
       driveLAcomp(1, 1000)
       driveLAbend(0, 1000)
       time.sleep(timeToExtend)  # Simulate full extension
       stopLAbend()
       stopLAcomp()
       LCcomp.tare(times=15)
       LCbend.tare(times=15)


       encoder.position = maxPlantarAngle / 360 * encoder.counts_per_rev
       ready = False


       print("Lower compressive actuator until pylon touches slider.")
       while not ready:
           # Check if the ready button is pressed
           if button_ready.value() == 0:
               ready = True  # Exit the loop when the ready button is pressed
               print("Ready button pressed. Moving to the next step.")
               continue
           # Check if the lower button is pressed
           if button_lower.value() == 0:  # Button is held
               print("Lower button held. Lowering compression actuator.")
               driveLAcomp(0, 1000)  # Lower the actuator
               # Wait until the button is released
               while button_lower.value() == 0:
                   time.sleep(0.1)
               print("Lower button released. Stopping compression actuator.")
               stopLAcomp()
           # Add a small delay to avoid excessive CPU usage
           time.sleep(0.1)
       print("Attach bending actuator. Press the ready button to confirm.")
       # Step 3: calibrate encoder
     
       encoder.offsetInit()
       current_angle = get_angle()


       # Step 2: Main loop
       print("Starting test...")
       for desired_angle, desired_force in compression_data:
           current_angle = get_angle()
           if desired_angle >= current_angle:




               nextAngleReached = False
               forceReached = False


               # Adjust compressive force using PID
               while forceReached != True:
                   current_force = get_compressive_force()
                   bending_force = get_bending_force()
                   output = pid.compute(desired_force, current_force)
                   print(output)
                   output = max(0, min(1000, output))  # restrict PWM to valid range
                   driveLAcomp(0, int(output))
                   print("output:")
                   print(output)
                   print("load cell reading:")
                   print(current_force)
                   print ("percent")
                   print (percent)
                   print ("desired force")
                   print (desired_force)
                   print ("current force")
                   print (current_force)
                   print ("desired angle")
                   print (desired_angle)
                   print ("current angle")
                   print (current_angle)
                   print ("bending force")
                   print (bending_force)
                  
          
                   if abs(current_force - desired_force) < force_tolerance:
                       forceReached = True
                   elif (current_force - desired_force) > force_tolerance:
                       forceReached = True
                   time.sleep(0.1)


               # Read current values
               current_angle = get_angle()
               print(current_angle)
               bending_force = get_bending_force()
               compressive_force = get_compressive_force()


               # Store data
               data_array.append(current_angle, bending_force, compressive_force)


               # Move bending actuator
               print(f"Moving to next angle: {desired_angle}")


               driveLAbend(1, 1000)
               while nextAngleReached != True:
                   current_angle = get_angle()
                   if abs(current_angle - desired_angle) < angle_tolerance:
                       nextAngleReached = True
                       stopLAbend()
                   elif current_angle > desired_angle:
                      nextAngleReached = True
                      stopLAbend()


               # Stop loop if max angle is reached
               if current_angle >= max(compression_data, key=lambda x: x[0])[0]:
                   break


       # Step 3: Extend compression linear actuators
       while not end:
           driveLAcomp(1, 1000)
           compressive_force = get_compressive_force()
           if compressive_force < 0:
               end = True


       stopLAcomp()


       # Step 4: Post-process and save data
       print("Post-processing data...")
       with open("moment_angle_RAW_DATA.csv", "w") as f:
           f.write("Angle (deg),Moment (Nm),Compressive Force (N)\n")
           for angle, bend_force, comp_force in data_array:
               moment = bend_force * pylonHeight  # Example pylon height = 1.0 m
               f.write(f"{angle},{moment},{comp_force}\n")
       print("Data saved to moment_angle_RAW_DATA.csv")
   except KeyboardInterrupt:
       if 'f' in locals():
           #data_array.append((current_angle, bending_force, compressive_force))
           f.close()
       data_array.append((current_angle, bending_force, compressive_force))
       LCbend.power_down()  # Optionally power down the HX711
       LCcomp.power_down()  # Optionally power down the HX711


   print("\nInterrupted by user. Exiting...")
   f=open('DEMOdata.csv','w')
   f.write(data_array)
   f.close()
   print(data_array)


# Run the main function
main()

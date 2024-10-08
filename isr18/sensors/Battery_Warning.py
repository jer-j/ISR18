import time
import board
import adafruit_ina260

i2c = board.I2C()  # uses board.SCL and board.SDA
ina260 = adafruit_ina260.INA260(i2c)

# Battery Values
warning_voltage = 11.5  # warn the user when voltage drops below this
shutoff_voltage = 11  # shutoff or sternly warn when the voltage drops below this

# Constants
sample_time_delay = 1  # read every 1 second
voltage_sample_size = 10  # store and check last 10 voltage readings

# Initialize lists to store data
time_data = []
voltage_data = [0] * voltage_sample_size  # Initialize with zeros
voltage_data_index = 0

def check_for_low_battery(voltage_data):
    for voltage in voltage_data:
        if voltage > warning_voltage:
            return False
    return True

def check_for_dead_battery(voltage_data):
    for voltage in voltage_data:
        if voltage > shutoff_voltage:
            return False
    return True

def low_battery_warning():
    print("Hey man your battery is low")

def dead_battery_warning():
    print("your battery is dead bro")

try:
    while True:
        voltage_data[voltage_data_index] = ina260.voltage
        voltage_data_index = (voltage_data_index + 1) % voltage_sample_size
        
        if check_for_low_battery(voltage_data):
            low_battery_warning()
        
        if check_for_dead_battery(voltage_data):
            dead_battery_warning()
        
        time.sleep(sample_time_delay)
        
except KeyboardInterrupt:
    # Save the final plot when interrupted by the user
    # Assuming you're planning to plot the voltage data at some point
    plt.savefig('voltage_plot.png')  # You need to import matplotlib.pyplot as plt
    print("Program stopped by user. Saving final plot as 'voltage_plot.png'.")

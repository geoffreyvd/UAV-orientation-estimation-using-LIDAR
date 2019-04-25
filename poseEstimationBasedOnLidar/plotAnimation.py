#!/usr/bin/env python
import threading
import numpy as np
# import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time


class SerialPortManager:
    def __init__(self, port_dir, baudrate):
        """
        Class that handles all Serial communication with the
        external Nucleoo and internal Arduino chip
        :param port_dir: Directory for Serial port
        :param baudrate: Baudrate for Serial connection
        """
        self.connected = False

        # try:
        # self.port = serial.Serial(port_dir, baudrate, timeout=1)
        self.connected = True
        # except:
        #     print("could not open: ", port_dir)
        #     exit()

    def write(self, message):
        """
        Writing to Serial port
        :param message: Message to be send
        :return:
        """
        # print("Writing data to serial port: " + message)
        if self.connected:
            self.port.write(message)
            self.port.flushOutput()

    def readx(self):
        """
        Reading from Serial port
        :return: Received message
        """
        if self.connected:
            return self.port.readline()

    def close(self):
        """
        Function to close Serial port
        :return:
        """
        if self.connected:
            return self.port.close()


class Nodo(object):
    def __init__(self):
        # Params
        self.running = True
        self.fps = 20
        self.graphtime = 100
        self.plot_data = [0] * self.graphtime
        self.time_data = np.linspace(0, 60, self.graphtime)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(411)
        self.ax2 = self.fig.add_subplot(412)
        self.ax3 = self.fig.add_subplot(413)
        self.ax4 = self.fig.add_subplot(414)
        # Serial connections
        # self.arduino_connection = SerialPortManager("COM13", 9600)

        self.previous_measurement = 0
        self.current_measurement = 0
        self.reaction = 0
        self.reaction_data = [0.0] * self.graphtime
        self.difference_reaction_data = [0.0] * self.graphtime
        self.distance_reaction_data = [0.0] * self.graphtime

        # Encoder runs in a thread so sending and receiving data will go simultaniously
        self.encoder_thread = threading.Thread(target=self.receive_data, args=())
        self.encoder_thread_running = True
        self.encoder_thread.start()

    def receive_data(self):
        """
        :return:
        """
        while self.encoder_thread_running:
            # data = self.arduino_connection.readx()
            # print("data: ", data)
            if data:
                try:
                    self.current_measurement = int(data)
                    self.plot_data.append(self.current_measurement)
                    self.calculate_avoidance()
                    if len(self.plot_data) > 100:
                        self.plot_data.pop(0)
                    self.previous_measurement = self.current_measurement
                except ValueError:
                    print("value error, skipping it")

    def calculate_avoidance(self):
        # difference_reaction = max(0, self.previous_measurement - self.current_measurement) / 10
        difference = max(0, self.previous_measurement - self.current_measurement)
        difference_reaction = min(0.001 * pow(difference, 3), 1500)
        difference_reaction = min(124872300 + (0.7154315 - 124872300) / (1 + (difference / 210097.6) ** 1.481022), 1500)
        distance_reaction = max(0, (5000.0 / np.exp(0.004 * self.current_measurement)) - 1)
        print("difference", self.previous_measurement - self.current_measurement, "\tdifference reaction", difference_reaction, "\t distance reaction", distance_reaction)
        self.reaction = difference_reaction + distance_reaction
        self.reaction_data.append(self.reaction)
        if len(self.reaction_data) > 100:
            self.reaction_data.pop(0)

        self.difference_reaction_data.append(difference_reaction)
        if len(self.difference_reaction_data) > 100:
            self.difference_reaction_data.pop(0)

        self.distance_reaction_data.append(distance_reaction)
        if len(self.distance_reaction_data) > 100:
            self.distance_reaction_data.pop(0)

    def animate(self, i):
        self.ax.clear()
        self.ax.set_title("Field of view")
        self.ax.set_xlabel("time(s)")
        self.ax.set_ylabel("distance(mm)")
        self.ax.grid(True)
        if len(self.time_data) == len(self.plot_data):
            self.ax.plot(self.time_data, self.plot_data, 'r', linewidth=1.5)

        self.ax2.clear()
        self.ax2.set_title("Distance reaction")
        self.ax2.set_xlabel("time(s)")
        self.ax2.set_ylabel("reaction")
        self.ax2.grid(True)
        if len(self.time_data == len(self.distance_reaction_data)):
            self.ax2.plot(self.time_data, self.distance_reaction_data, 'r', linewidth=1.5)

        self.ax3.clear()
        self.ax3.set_title("Difference reaction")
        self.ax3.set_xlabel("time(s)")
        self.ax3.set_ylabel("reaction")
        self.ax3.grid(True)
        if len(self.time_data == len(self.difference_reaction_data)):
            self.ax3.plot(self.time_data, self.difference_reaction_data, 'r', linewidth=1.5)

        self.ax4.clear()
        self.ax4.set_title("Total reaction")
        self.ax4.set_xlabel("time(s)")
        self.ax4.set_ylabel("reaction")
        self.ax4.grid(True)
        if len(self.time_data == len(self.reaction_data)):
            self.ax4.plot(self.time_data, self.reaction_data, 'r', linewidth=1.5)

    def stop(self):
        print("shutting down")
        self.arduino_connection.close()
        self.encoder_thread_running = False
        self.running = False
        print("shut down complete")


if __name__ == '__main__':
    my_node = Nodo()
    ani = animation.FuncAnimation(my_node.fig, my_node.animate, interval=int(1000/my_node.fps))
    plt.title("test")
    plt.show()
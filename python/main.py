import threading
import time
import numpy as np
import argparse
from CanineSim import *
from sharedMemory import SharedMemoryManager
from TCPClinet import CanineTCPClient
from colorama import Fore, Style

shm = SharedMemoryManager()

D2R = 3.141592/180

parser = argparse.ArgumentParser(description="robot type")
parser.add_argument("--robot", type=int, help="Robot type: 0-canine, 1-canine_gadget")
args = parser.parse_args()

def simulationThread():
    if args.robot == 0:
        sim = Canine(shm)
    elif args.robot == 1:
        sim = CanineGadget(shm)
    elif args.robot == 2:
        sim = CanineGadgetLight(shm)
    else:
        print("Invalid robot type.")
        return
    sim.startSimulation()

def TCPCommunicationThread(args=None):
    tcp = CanineTCPClient(shm)
    while True:
        tcp.doCommunication()
        time.sleep(0.001)

thread_simulation = threading.Thread(target=simulationThread)
thread_simulation.start()

thread_tcp_communication = threading.Thread(target=TCPCommunicationThread)
thread_tcp_communication.start()
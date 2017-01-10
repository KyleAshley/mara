import can



class can_bus():
	bus = can.interface.Bus('test')
	rx_buff = []
	tx_buff = []

	def transmit(self, data):
		bus = can.interface.Bus()
		msg = can.Message(arbitration_id=0x000000,
							data=data,
							extended_id=False)
		try:
			bus.send(msg)
			print("Message sent on {}".format(bus.channel_info))
		except can.CanError:
			print("Message NOT sent")

	def receive(self, timeout=None):
		
		msg = can.Message()
		try:
			msg = bus.recv(msg)
			print("Message recieved on {}".format(bus.channel_info))
		except can.CanError:
			print("Message NOT sent")

if __name__ == "__main__":
	can = can_bus()
    can.transmit(data=[0, 25, 0, 1, 3, 1, 4, 1])
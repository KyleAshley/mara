#!/usr/bin/env python

import rospy
import can

# CAN config
can.rc['interface'] = 'socketcan_ctypes'
can.rc['channel'] = 'can0'

from can.interfaces.interface import Bus

# ON
# mara docs says: sudo ip link set can0 up type can bitrate 1000000 sample-point 0.875
# python can says: sudo ip link set can0 up type can bitrate 500000 restart-ms 10000

# OFF
# ip link set can0 down
# rmmod usb_8dev

CONT_REG = 0x300
COMM_REG = 0x301
STAT_REG = 0x302
INT_REG  = 0x303
ACC_REG  = 0x304
ACM_REG  = 0x305
BT0_REG  = 0x306
BT1_REG  = 0x307
OUTP_REG = 0x308
TEST_REG = 0x309

TXID_REG = 0x30A
TRTR_REG = 0x30B
TXB1     = 0x30C
TXB2     = 0x30D
TXB3     = 0x30E
TXB4     = 0x30F
TXB5     = 0x310
TXB6     = 0x311
TXB7     = 0x312
TXB8     = 0x313

RXID_REG = 0x314
RRTR_REG = 0x315
RXB1     = 0x316
RXB2     = 0x317
RXB3     = 0x318
RXB4     = 0x319
RXB5     = 0x31A
RXB6     = 0x31B
RXB7     = 0x31C
RXB8     = 0x31D

CLKD_REG = 0x31F

ERROR_ID  = 1
EXIT      = 'o'

CARTESIAN  = 1
SET_ZERO   = 2
JOINT      = 4
FOLD_OUT   = 5
FOLD_IN    = 6

STUCK_GRIPPER        = 0
NOT_ATTACHE          = 1
ARM_FOLDED_STRECHED  = 2
BLOCKED_DOF          = 3
MAX_M1_ROTATION      = 4
MOVEMENT_ERROR       = 5
MEMORY186_FULL       = 6

MAX_CART             = 70
MAX_JOINT            = 2
MAX_JOINT_GRIP       = 10
MAX_CART_GRIP        = 12
MAX_JOINT_YPR        = 3
MAX_CART_YPR         = 6


class can_bus():
	def __init__(self):
		self.bus = Bus('can0', bustype='socketcan_ctypes')
		self.rx_buff = []
		self.tx_buff = []

	def transmit(self, data):
		msg = can.Message(arbitration_id=0x000,
							data=data,
							extended_id=False)
		try:
			self.bus.send(msg)
			print("Message sent on {}".format(self.bus.channel_info))
		except can.CanError:
			print("Message NOT sent")

	def recieve(self, timeout=None):
		
		try:
			msg = self.bus.recv(timeout=1.0)
			print("Message recieved on {}".format(self.bus.channel_info))
			print msg
		except can.CanError:
			print("Message NOT received")

if __name__ == "__main__":
	can_interface = can_bus()

	try:
	    while True:
	        can_interface.recieve()
	except KeyboardInterrupt:
	    pass



def decode(rcvmessage, xmitmessage):

	echo = 1
	print rcvmessage
	manus_status   = rcvmessage[0];
	manus_message  = rcvmessage[1];

	if ((manus_status & 0x80) == 0x80):
		if ((manus_message == 2) and (echo == 1)):
			print "MANUS gripper is ready"
			echo = 2

		if ((manus_message == 3) and (echo == 2)):
			print "MANUS absolute angles are ready"
			echo = 0

	"""
      pos[1] = ((unsigned int)rcvmessage->dat[2] << 8) +
               (unsigned char)rcvmessage->dat[3];
      pos[2] = ((unsigned int)rcvmessage->dat[4] << 8) +
               (unsigned char)rcvmessage->dat[5];
      pos[3] = ((unsigned int)rcvmessage->dat[6] << 8) +
               (unsigned char)rcvmessage->dat[7];

      xmitmessage->ident = rcvmessage->ident;
      xmitmessage->rtr   = 0;
      xmitmessage->len   = 0;
      break;

    case 0x360:
      pos[4] = ((unsigned int)rcvmessage->dat[0] << 8) +
               (unsigned char)rcvmessage->dat[1];
      pos[5] = ((unsigned int)rcvmessage->dat[2] << 8) +
               (unsigned char)rcvmessage->dat[3];
      pos[6] = ((unsigned int)rcvmessage->dat[4] << 8) +
               (unsigned char)rcvmessage->dat[5];
      pos[7] = ((unsigned int)rcvmessage->dat[6] << 8) +
               (unsigned char)rcvmessage->dat[7];

      xmitmessage->ident = rcvmessage->ident;
      xmitmessage->rtr   = 0;
      xmitmessage->len   = 0;
      break;

    case 0x37F:
      xmitmessage->rtr = 0;    /* for all cboxes */
      switch(cbox)
      { case CARTESIAN:
          xmitmessage->ident = 0x371;
          xmitmessage->len = 8;
          for(i = 0; i < xmitmessage->len; i++)
            xmitmessage->dat[i] = speed[i];
          break;

        case JOINT:
          xmitmessage->ident = 0x374;
          xmitmessage->len = 8;
          for(i = 0; i < xmitmessage->len; i++)
            xmitmessage->dat[i] = speed[i];
          break;

        case FOLD_OUT:
          xmitmessage->ident = 0x375;
          xmitmessage->len = 0;
          break;

        case FOLD_IN:
          xmitmessage->ident = 0x376;
          xmitmessage->len = 0;
          break;

        default:     /* do not move */
          xmitmessage->ident = 0x370;
          xmitmessage->len = 0;
          break;
      } /* end of switch cbox */
      break;
  }  /* end of switch rcvmessage */
}
"""